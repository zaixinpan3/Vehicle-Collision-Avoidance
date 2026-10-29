"""Synchronously couple a running CARLA world to the external VDB plant.

One JSON-line transaction per VDB controller sample. CARLA physics is disabled
for the externally integrated ego. Target motion is explicitly scripted NRMM;
this version does not claim a second 14DOF target vehicle.
"""
import argparse, json, math, socket
import carla
import numpy as np
from scipy.spatial.transform import Rotation


def run(port, bridge_port):
    client=carla.Client('localhost',port);client.set_timeout(60)
    world=client.get_world();assert 'Town10' in world.get_map().name
    original=world.get_settings();settings=carla.WorldSettings(synchronous_mode=True,fixed_delta_seconds=.05,no_rendering_mode=original.no_rendering_mode)
    world.apply_settings(settings);actors=[];records=[];initial=None
    try:
        with socket.socket() as listener:
            listener.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);listener.bind(('127.0.0.1',bridge_port));listener.listen(1)
            print('READY',world.get_map().name,'bridge',bridge_port,flush=True)
            with listener.accept()[0] as connection:
                stream=connection.makefile('rwb')
                for line in stream:
                    request=json.loads(line);operation=request['operation']
                    if operation=='begin':
                        for actor in actors:actor.destroy()
                        actors=[];initial=np.array(request['target'],float)
                        for _ in range(2):
                            actor=None
                            for spawn in world.get_map().get_spawn_points():
                                actor=world.try_spawn_actor(world.get_blueprint_library().find('vehicle.lincoln.mkz'),spawn)
                                if actor is not None:break
                            if actor is None:raise RuntimeError('No free vehicle spawn')
                            actors.append(actor);actor.set_simulate_physics(False)
                        reply={'map':world.get_map().name,'ready':True}
                    elif operation=='step':
                        t=request['time'];pose=np.array(request['pose']);ego,target=actors
                        flip=np.diag([1,1,-1]);expected=flip@Rotation.from_euler('xyz',pose[3:6]).as_matrix()@flip
                        choices=[]
                        for sr in [-1,1]:
                            for sp in [-1,1]:
                                rot=carla.Rotation(roll=float(sr*np.degrees(pose[3])),pitch=float(sp*np.degrees(pose[4])),yaw=float(np.degrees(pose[5])))
                                error=np.max(np.abs(np.array(carla.Transform(rotation=rot).get_matrix())[:3,:3]-expected));choices.append((error,rot))
                        error,rotation=min(choices,key=lambda p:p[0]);assert error<1e-5
                        z=world.get_map().get_waypoint(carla.Location(x=float(pose[0]),y=float(pose[1]))).transform.location.z
                        ego.set_transform(carla.Transform(carla.Location(x=float(pose[0]),y=float(pose[1]),z=float(z+.2-pose[2])),rotation))
                        q=initial.copy();stop=t
                        if q[4]<0:stop=min(stop,max(0,-q[3]/q[4]))
                        arc=q[3]*stop+.5*q[4]*stop*stop;k=math.sin(q[5])/q[6];a=k*arc/2;travel=arc*(math.sin(a)/a if abs(a)>1e-10 else 1)
                        q[:2]+=travel*np.array([math.cos(q[2]+q[5]+a),math.sin(q[2]+q[5]+a)]);q[2]+=k*arc;q[3]=max(0,q[3]+q[4]*t)
                        target.set_transform(carla.Transform(carla.Location(x=float(q[0]),y=float(-q[1]),z=float(z+.2)),carla.Rotation(yaw=float(-np.degrees(q[2])))))
                        frame=world.tick();actual=target.get_transform();q[:2]=[actual.location.x,-actual.location.y];q[2]=-math.radians(actual.rotation.yaw)
                        reply={'frame':frame,'time':t,'target':q.tolist()};records.append(reply)
                    elif operation=='end':
                        reply={'frames':len(records)}
                    else:raise ValueError(operation)
                    stream.write((json.dumps(reply)+'\n').encode());stream.flush()
    finally:
        for actor in actors:actor.destroy()
        world.apply_settings(original)
        print('CLOSED transactions',len(records),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--port',type=int,default=2500);parser.add_argument('--bridge-port',type=int,default=2605);args=parser.parse_args();run(args.port,args.bridge_port)
