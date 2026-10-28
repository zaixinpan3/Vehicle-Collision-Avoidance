"""Render recorded VDB truth through CARLA top and attached chase cameras."""
import carla,json,queue,subprocess,math,argparse
import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation
from pathlib import Path
parser=argparse.ArgumentParser();parser.add_argument('--input',type=Path,required=True);parser.add_argument('--port',type=int,default=2200);parser.add_argument('--camera',choices=['rgb','semantic'],default='rgb');args=parser.parse_args();root=args.input;c=carla.Client('localhost',args.port);c.set_timeout(60);w=c.get_world();assert 'Town10' in w.get_map().name
for name in ['straight','curve','intersection']:
 file=root/(name+'.json')
 if not file.exists():continue
 data=json.loads(file.read_text());poses=np.asarray(data['pose']);times=np.atleast_1d(data['time']);poses=poses.reshape(len(times),6);q=data['targetInitial']
 w.set_weather(carla.WeatherParameters.ClearNoon)
 try:old=w.get_settings()
 except RuntimeError:old=None
 settings=carla.WorldSettings(synchronous_mode=True,no_rendering_mode=False,fixed_delta_seconds=.05);w.apply_settings(settings);actors=[];enc=[]
 try:
  bp=w.get_blueprint_library().find('vehicle.lincoln.mkz')
  for _ in range(2):
   actor=None
   for spawn in w.get_map().get_spawn_points():
    actor=w.try_spawn_actor(bp,spawn)
    if actor is not None:break
   if actor is None:raise RuntimeError('No free visualization actor spawn point')
   actors.append(actor);actor.set_simulate_physics(False)
  ego,target=actors
  queues=[]
  for view in ['top','chase']:
   cam=w.get_blueprint_library().find('sensor.camera.rgb' if args.camera=='rgb' else 'sensor.camera.semantic_segmentation');cam.set_attribute('image_size_x','1024');cam.set_attribute('image_size_y','512');cam.set_attribute('fov','85');cam.set_attribute('sensor_tick','0');
   trans=carla.Transform(carla.Location(z=35),carla.Rotation(pitch=-90)) if view=='top' else carla.Transform(carla.Location(x=-9,z=5),carla.Rotation(pitch=-15))
   camera=w.spawn_actor(cam,trans,attach_to=ego);actors.append(camera);qu=queue.Queue();camera.listen(qu.put);queues.append(qu)
   cmd=['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','1024x512','-r','20','-i','-','-an','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p','-vf','tpad=stop_mode=clone:stop_duration=3',str(root/f'{name}_{view}.mp4')]
   enc.append(subprocess.Popen(cmd,stdin=subprocess.PIPE))
  z0=poses[0,2]
  for t,p in zip(times,poses):
   # SAE FRD to CARLA x-forward/y-right/z-up, including roll and pitch.
   flip=np.diag([1,1,-1]);matrix=flip@Rotation.from_euler('xyz',p[3:6]).as_matrix()@flip
   candidates=[]
   for sr in [-1,1]:
    for sp in [-1,1]:
     rotation=carla.Rotation(roll=float(sr*np.degrees(p[3])),pitch=float(sp*np.degrees(p[4])),yaw=float(np.degrees(p[5])))
     err=np.max(np.abs(np.asarray(carla.Transform(rotation=rotation).get_matrix())[:3,:3]-matrix));candidates.append((err,rotation))
   err,rotation=min(candidates,key=lambda item:item[0]);assert err<1e-5

   ego.set_transform(carla.Transform(carla.Location(x=float(p[0]),y=float(p[1]),z=float(data['roadHeight']-(p[2]-z0)+.2)),rotation))
   s=q[3]*t+.5*q[4]*t*t;k=math.sin(q[5])/q[6];a=k*s/2;ds=s*(math.sin(a)/a if abs(a)>1e-10 else 1);angle=q[2]+q[5]+a
   target.set_transform(carla.Transform(carla.Location(x=float(q[0]+ds*math.cos(angle)),y=float(-q[1]-ds*math.sin(angle)),z=float(data['roadHeight']+.2)),carla.Rotation(yaw=float(-math.degrees(q[2]+k*s)))))
   if t==times[0]:
    for _ in range(15):
     f=w.tick()
     for qu in queues:
      image=qu.get(timeout=40)
      while image.frame<f:image=qu.get(timeout=40)
   frame=w.tick()
   for qu,en in zip(queues,enc):
    image=qu.get(timeout=40)
    while image.frame<frame:image=qu.get(timeout=40)
    assert image.frame==frame
    if args.camera=='semantic':image.convert(carla.ColorConverter.CityScapesPalette)
    rgb=np.frombuffer(image.raw_data,np.uint8).reshape(image.height,image.width,4)[:,:,:3][:,:,::-1].copy()
    if args.camera=='rgb' and t==times[0] and np.percentile(rgb.max(axis=2),99)<3:
     raise RuntimeError('CARLA RGB readback is nearly black; do not publish this video as valid camera footage')
    canvas=Image.fromarray(rgb);draw=ImageDraw.Draw(canvas);draw.rectangle((0,0,1024,34),fill=(8,18,30));draw.text((12,10),f'{name} | {args.camera} | VDB 14DOF + NRMM/PCBF | t={t:.2f}s'+(' | FAILED / simulation stopped' if data['failure'] and t>=times[-1]-.001 else ''),fill='white');en.stdin.write(canvas.tobytes())
  print('RENDERED',name,'frames',len(times),'failure',bool(data['failure']),flush=True)
 finally:
  for a in reversed(actors):
   if isinstance(a,carla.Sensor):a.stop()
   a.destroy()
  for en in enc:en.stdin.close();en.wait()
  w.apply_settings(old if old is not None else carla.WorldSettings(synchronous_mode=False,fixed_delta_seconds=.05))
