"""Export Town10 driving-lane surface geometry and three reproducible routes."""
import carla,json,math,argparse
import numpy as np
from pathlib import Path
from shapely.geometry import Polygon
from shapely.ops import unary_union
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--port',type=int,default=2200);args=parser.parse_args();c=carla.Client('localhost',args.port);c.set_timeout(40);w=c.get_world();m=w.get_map();assert 'Town10' in m.name
waypoints=m.generate_waypoints(2.)
polys=[]
for a in waypoints:
 if a.lane_type!=carla.LaneType.Driving:continue
 for b in a.next(2.):
  pts=[]
  for wp,side in [(a,1),(b,1),(b,-1),(a,-1)]:
   p=wp.transform.location;r=wp.transform.get_right_vector();pts.append([p.x+side*r.x*wp.lane_width/2,-p.y-side*r.y*wp.lane_width/2])
  poly=Polygon(pts)
  if poly.is_valid and poly.area>1e-4:polys.append(poly)
road=unary_union(polys).buffer(.02).simplify(.08,preserve_topology=True)
regions=list(road.geoms) if road.geom_type=='MultiPolygon' else [road]
# Preserve holes as separate subtractive rings, never fill traffic islands.
rings=[{'outer':list(p.exterior.coords),'holes':[list(r.coords) for r in p.interiors]} for p in regions]
candidates={}
for a in waypoints:
 if a.is_junction:continue
 route=[a]
 for j in range(65):
  nxt=route[-1].next(2)
  if not nxt:break
  nxt=sorted(nxt,key=lambda x:abs(x.transform.rotation.yaw-route[-1].transform.rotation.yaw))[0];route.append(nxt)
 if len(route)<60:continue
 yaw=np.unwrap(np.deg2rad([p.transform.rotation.yaw for p in route]));change=max(abs(yaw-yaw[0]));junction=any(p.is_junction for p in route[5:20])
 name='intersection' if junction else ('curve' if .2<change<1 else 'straight' if change<.03 else None)
 if name and name not in candidates:
  candidates[name]=[[p.transform.location.x,-p.transform.location.y,-math.radians(p.transform.rotation.yaw),p.transform.location.z] for p in route]
 if len(candidates)==3:break
ws=[w for w in m.generate_waypoints(1)];pos=np.array([[w.transform.location.x,-w.transform.location.y] for w in ws]);ang=np.deg2rad([-w.transform.rotation.yaw for w in ws]);vel4=4*np.c_[np.cos(ang),np.sin(ang)]
found=None
for i,w in enumerate(ws):
 e=pos[i];v=8*np.array([math.cos(ang[i]),math.sin(ang[i])]);rel=e-pos;vel=v-vel4;t=-np.sum(rel*vel,1)/np.sum(vel*vel,1);gap=np.linalg.norm(rel+t[:,None]*vel,axis=1);dist=np.linalg.norm(rel,axis=1)
 ids=np.where((abs(np.cos(ang-ang[i]))<.2)&(t>.5)&(t<3)&(gap<1)&(dist<29)&(dist>8))[0]
 for j in ids:
  meet=e+t[j]*v;mp=m.get_waypoint(carla.Location(x=float(meet[0]),y=float(-meet[1])),project_to_road=True)
  if mp.is_junction:found=(i,j,float(t[j]),float(gap[j]));break
 if found:break
assert found
route=[ws[found[0]]]
for _ in range(65):
 nxt=route[-1].next(2)
 if not nxt:break
 route.append(min(nxt,key=lambda w:abs(math.atan2(math.sin(math.radians(w.transform.rotation.yaw-route[-1].transform.rotation.yaw)),math.cos(math.radians(w.transform.rotation.yaw-route[-1].transform.rotation.yaw))))))

candidates['intersection']=[[w.transform.location.x,-w.transform.location.y,-math.radians(w.transform.rotation.yaw),w.transform.location.z] for w in route]
j=found[1];intersectionTarget=[*pos[j],float(ang[j]),4,0,0,1.65,2.4,.95,0,0]
out={'intersectionTarget':intersectionTarget,'map':m.name,'coordinateConvention':'x CARLA x, y minus CARLA y, yaw minus CARLA yaw','roadRegions':rings,'routes':candidates}
args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(out));print(m.name,len(polys),len(regions),list(candidates))
