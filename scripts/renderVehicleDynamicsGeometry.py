"""Render explicit geometric reconstructions, never claim CARLA camera footage."""
import argparse, json, math, subprocess
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation
from shapely.geometry import Polygon, box


def render(folder, map_file):
    data=json.loads(map_file.read_text())
    roads=[Polygon(r['outer'],r['holes']) for r in data['roadRegions']]
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16)
    paths=[]
    for name in ['straight','curve','intersection']:
        trial=json.loads((folder/(name+'.json')).read_text())
        times=np.atleast_1d(trial['time']);poses=np.array(trial['pose']).reshape(len(times),6)
        q=trial['targetInitial'];output=folder/(name+'_geometry.mp4');paths.append(str(output))
        encoder=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','1600x700','-r','20','-i','-','-an','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p',str(output)],stdin=subprocess.PIPE)
        last=None
        for index,(time,pose) in enumerate(zip(times,poses)):
            image=Image.new('RGB',(1600,700),'#101c29');draw=ImageDraw.Draw(image)
            ego=np.array([pose[0],-pose[1],0.8-(pose[2]-poses[0,2])]);heading=-pose[5]
            forward=np.array([math.cos(heading),math.sin(heading),0.])
            s=q[3]*time+.5*q[4]*time*time;k=math.sin(q[5])/q[6];angle=k*s/2
            travel=s*(math.sin(angle)/angle if abs(angle)>1e-10 else 1)
            target=np.array([q[0]+travel*math.cos(q[2]+q[5]+angle),q[1]+travel*math.sin(q[2]+q[5]+angle),.8])
            for view in range(2):
                panel=Image.new('RGB',(800,700),'#101c29');draw=ImageDraw.Draw(panel);offset=0
                if view==0:
                    def project(vertices):
                        v=np.atleast_2d(vertices)-ego
                        return np.column_stack([offset+400+v[:,0]*12,350-v[:,1]*12])
                else:
                    eye=ego-14*forward+np.array([0,0,8]);look=ego+12*forward
                    axis=(look-eye)/np.linalg.norm(look-eye);right=np.cross(axis,[0,0,1]);right/=np.linalg.norm(right);up=np.cross(right,axis)
                    def project(vertices):
                        v=np.atleast_2d(vertices)-eye;depth=v@axis
                        if np.any(depth<.49999):return None
                        return np.column_stack([offset+400+620*(v@right)/depth,350-620*(v@up)/depth])
                def polygon(vertices,color):
                    vertices=np.asarray(vertices)
                    if view==1:
                        clipped=[]
                        for a,b in zip(vertices,np.roll(vertices,-1,axis=0)):
                            da=(a-eye)@axis-.5;db=(b-eye)@axis-.5
                            if da>=0:clipped.append(a)
                            if (da>=0)!=(db>=0):clipped.append(a+da/(da-db)*(b-a))
                        if len(clipped)<3:return
                        vertices=np.asarray(clipped)
                    points=project(vertices)
                    if points is not None:draw.polygon([tuple(p) for p in points],fill=color)
                clip=box(ego[0]-45,ego[1]-45,ego[0]+45,ego[1]+45)
                for road in roads:
                    local=road.intersection(clip)
                    parts=list(local.geoms) if local.geom_type=='MultiPolygon' else [local]
                    for part in parts:
                        if part.is_empty or part.geom_type!='Polygon':continue
                        rings=[(part.exterior,'#49525b')]+[(h,'#101c29') for h in part.interiors]
                        for ring,color in rings:
                            xy=np.array(ring.coords);polygon(np.c_[xy,np.zeros(len(xy))],color)
                def car(center,rotation,color):
                    vertices=np.array([[x,y,z] for z in [-.65,.65] for y in [-.95,.95] for x in [-2.4,2.4]])@rotation.T+center
                    faces=[[0,1,3,2],[0,1,5,4],[0,2,6,4],[1,3,7,5],[2,3,7,6],[4,5,7,6]]
                    order=sorted(faces,key=lambda f:np.mean(vertices[f,2])) if view==0 else sorted(faces,key=lambda f:np.linalg.norm(np.mean(vertices[f],axis=0)-eye),reverse=True)
                    for n,face in enumerate(order):polygon(vertices[face],tuple(max(0,int(c*(.65+.06*n))) for c in color))
                    p=project([center,center+rotation[:,0]*2])
                    if p is not None:draw.line([tuple(v) for v in p],fill='white',width=3)
                flip=np.diag([1,-1,-1]);rotation=flip@Rotation.from_euler('xyz',pose[3:6]).as_matrix()@flip
                car(target,Rotation.from_euler('z',q[2]+k*s).as_matrix(),[245,133,111])
                car(ego,rotation,[70,220,205])
                draw.rectangle((offset,0,offset+800,68),fill='#08121f')
                draw.text((offset+15,12),('TOP' if view==0 else 'CHASE')+' | Geometry reconstruction; not CARLA camera',font=font,fill='white')
                draw.text((offset+15,39),f'{name} | 14DOF truth + Town10 map | t={time:.2f}s',font=font,fill='white')
                if index==len(times)-1 and trial['failure']:
                    draw.rectangle((offset,640,offset+800,700),fill='#541c23');draw.text((offset+15,657),'OPTIMIZATION FAILED - simulation ends here',font=font,fill='white')
                image.paste(panel,(view*800,0))
            last=image.tobytes();encoder.stdin.write(last)
        for _ in range(60):encoder.stdin.write(last)
        encoder.stdin.close();assert encoder.wait()==0
        print('RECONSTRUCTED',name,len(times),flush=True)
    (folder/'geometry.m3u').write_text('\n'.join(paths)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--input',type=Path,required=True);parser.add_argument('--map',type=Path,required=True);args=parser.parse_args();render(args.input,args.map)
