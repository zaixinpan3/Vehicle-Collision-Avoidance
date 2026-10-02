from pathlib import Path
import json,sys,numpy as np
p=Path(__file__).parent;sys.path.insert(0,str(p/'source/scripts'))
from auditJointPredictiveSafety import target_state,rectangle,distance
r=json.loads(Path(sys.argv[1]).read_text())['results'];q=r['targetInitialState'];v=r['configuration']['vehicle'];shape=[v['length']/2,v['width']/2,*v['rectangleOffset']]
worst=(1e10,None);bad=[]
for f in r['trace']:
 for dt,x in zip(f['auditTimes'],f['auditStates']):
  time=f['time']+dt;t=target_state(q,time);a=np.array(rectangle(*x[:3],shape));b=np.array(rectangle(*t[:3],q[7:11]));g=[]
  for angle in [x[2],x[2]+np.pi/2,t[2],t[2]+np.pi/2]:
   n=np.array([np.cos(angle),np.sin(angle)]);aa=a@n;bb=b@n;g.append(max(min(bb)-max(aa),min(aa)-max(bb)))
  gap=max(g)
  if gap<worst[0]:worst=(gap,dict(time=time,holdTime=f['time'],elapsed=dt,state=x,input=f['input'],primary=f['primaryOptimum'],slack=f['predictiveBarrierValue'],clfSlack=f['clfSlack'],stages=f['solverStages']))
  if gap<0:bad.append(time)
print('worst',worst,'collisions',len(bad),'interval', (min(bad),max(bad)) if bad else None)
Path(sys.argv[2]).write_text(json.dumps(dict(worstPenetrationMargin=worst[0],frame=worst[1],collisionSamples=len(bad),collisionInterval=[min(bad),max(bad)] if bad else []),indent=2)+'\n')
