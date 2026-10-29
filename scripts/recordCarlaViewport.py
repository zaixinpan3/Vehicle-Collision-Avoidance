"""Record only a CARLA native viewport window on niri, replaying online logs.

The screenshots are restricted to the explicitly supplied CARLA window ID.
This bypasses camera-sensor readback; all scene pixels come from Unreal.
"""
import argparse, json, math, subprocess, time
from pathlib import Path
import carla
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation


def record(folder, port, window_id):
    client=carla.Client('localhost',port);client.set_timeout(60);world=client.get_world()
    assert 'Town10' in world.get_map().name
    windows=json.loads(subprocess.check_output(['niri','msg','--json','windows'],text=True))
    chosen=next(w for w in windows if w['id']==window_id)
    assert 'carla' in (chosen.get('app_id','')+chosen.get('title','')).lower()
    old=world.get_settings();world.apply_settings(carla.WorldSettings(synchronous_mode=True,fixed_delta_seconds=.05,no_rendering_mode=False));world.set_weather(carla.WeatherParameters.ClearNoon)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18);playlist=[]
    try:
        for name in ['straight','curve','intersection']:
            data=json.loads((folder/(name+'.json')).read_text());times=np.atleast_1d(data['time']);poses=np.array(data['pose']).reshape(len(times),6);q0=data['targetInitial'];actors=[]
            try:
                for _ in range(2):
                    actor=None
                    for spawn in world.get_map().get_spawn_points():
                        actor=world.try_spawn_actor(world.get_blueprint_library().find('vehicle.lincoln.mkz'),spawn)
                        if actor is not None:break
                    assert actor is not None;actor.set_simulate_physics(False);actors.append(actor)
                ego,target=actors
                for view in ['top','chase']:
                    path=folder/f'{name}_{view}_native.mp4';playlist.append(str(path))
                    encoder=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','1024x576','-r','20','-i','-','-an','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p',str(path)],stdin=subprocess.PIPE)
                    try:
                        for index,(t,p) in enumerate(zip(times,poses)):
                            flip=np.diag([1,1,-1]);matrix=flip@Rotation.from_euler('xyz',p[3:6]).as_matrix()@flip;options=[]
                            for sr in [-1,1]:
                                for sp in [-1,1]:
                                    rot=carla.Rotation(roll=float(sr*np.degrees(p[3])),pitch=float(sp*np.degrees(p[4])),yaw=float(np.degrees(p[5])))
                                    error=np.max(np.abs(np.array(carla.Transform(rotation=rot).get_matrix())[:3,:3]-matrix));options.append((error,rot))
                            error,rotation=min(options,key=lambda x:x[0]);assert error<1e-5
                            egopose=carla.Transform(carla.Location(x=float(p[0]),y=float(p[1]),z=float(data['roadHeight']+.2-(p[2]-poses[0,2]))),rotation);ego.set_transform(egopose)
                            q=np.array(q0);s=q[3]*t+.5*q[4]*t*t;k=math.sin(q[5])/q[6];a=k*s/2;ds=s*(math.sin(a)/a if abs(a)>1e-10 else 1);angle=q[2]+q[5]+a
                            target.set_transform(carla.Transform(carla.Location(x=float(q[0]+ds*math.cos(angle)),y=float(-q[1]-ds*math.sin(angle)),z=float(data['roadHeight']+.2)),carla.Rotation(yaw=float(-np.degrees(q[2]+k*s)))))
                            camera=carla.Transform(egopose.location+carla.Location(z=35),carla.Rotation(pitch=-90,yaw=rotation.yaw)) if view=='top' else carla.Transform(egopose.transform(carla.Location(x=-9,z=5)),carla.Rotation(pitch=-15,yaw=rotation.yaw))
                            world.get_spectator().set_transform(camera)
                            for _ in range(10 if index==0 else 1):world.tick()
                            time.sleep(.08)
                            screenshot=folder/'viewportFrame.png';screenshot.unlink(missing_ok=True);subprocess.run(['niri','msg','action','screenshot-window','--id',str(window_id),'--path',str(screenshot)],check=True,stdout=subprocess.DEVNULL)
                            deadline=time.monotonic()+5
                            while True:
                                try:
                                    with Image.open(screenshot) as raw:image=raw.convert('RGB').resize((1024,576),Image.Resampling.LANCZOS)
                                    break
                                except (FileNotFoundError,OSError):
                                    if time.monotonic()>deadline:raise
                                    time.sleep(.01)
                            draw=ImageDraw.Draw(image);draw.rectangle((0,0,1024,32),fill='#08121f');draw.text((12,6),f'Town10 | {name} | {view.upper()} | online 14DOF run replay | t={t:.2f}s',font=font,fill='white')
                            if index==len(times)-1 and data['failure']:
                                draw.rectangle((0,535,1024,576),fill='#541c23');draw.text((12,545),'OPTIMIZATION FAILED - simulation ended here',font=font,fill='white')
                            frame=image.tobytes();encoder.stdin.write(frame)
                        for _ in range(60):encoder.stdin.write(frame)
                    finally:
                        encoder.stdin.close();assert encoder.wait()==0
                    print('NATIVE_VIEWPORT',name,view,len(times),flush=True)
            finally:
                for actor in actors:actor.destroy()
    finally:world.apply_settings(old)
    (folder/'native.m3u').write_text('\n'.join(playlist)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--input',type=Path,required=True);parser.add_argument('--port',type=int,default=2800);parser.add_argument('--window-id',type=int,required=True);args=parser.parse_args();record(args.input,args.port,args.window_id)
