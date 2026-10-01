from pathlib import Path
import json,sys,math
sys.path.insert(0,'/home/zai/Downloads/ResearchProjects/collisionAvoidance/scripts')
from auditJointPredictiveSafety import target_state,rectangle,distance
def axis_margin(a,b):
 gaps=[]
 for polygon in (a,b):
  for i,p in enumerate(polygon):
   q=polygon[(i+1)%4];dx=q[0]-p[0];dy=q[1]-p[1];length=math.hypot(dx,dy)
   normal=(-dy/length,dx/length)
   pa=[normal[0]*v[0]+normal[1]*v[1] for v in a];pb=[normal[0]*v[0]+normal[1]*v[1] for v in b]
   gaps.extend((min(pa)-max(pb),min(pb)-max(pa)))
 return max(gaps)
raw=Path(__file__).parent;records=[]
for p in sorted((raw/'campaign').glob('speed*/*.json')):
 if '-20s' in p.stem:continue
 r=json.loads(p.read_text())['results']
 if r['minimumReplayClearanceMeters'] is None or r['minimumReplayClearanceMeters']>0:continue
 # MATLAB and independent Python rectangles are compared at the same ODE nodes.
 speed=r['configuration']['referenceSpeed'];q=r['targetInitialState'];vehicle=r['configuration']['vehicle']
 first=None
 for hold in r['trace']:
  for t,x in zip(hold['auditTimes'],hold['auditStates']):
   absolute=hold['time']+t;state=target_state(q,absolute)
   ego=rectangle(*x[:3],(vehicle['length']/2,vehicle['width']/2,*vehicle['rectangleOffset']))
   target=rectangle(*state[:3],q[7:11])
   if distance(ego,target)<=0:
    first=dict(speed=speed,scenario=r['scenario'],holdTime=hold['time'],collisionTime=absolute,
               signedAxisMargin=axis_margin(ego,target),
               primaryOptimum=hold['primaryOptimum'],slack=hold['predictiveBarrierValue'],
               solverFlags=[s['exitFlag'] for s in hold['solverStages']],initialization=hold['initialization'],
               flowRestarted=hold['flowRestarted'],input=hold['input'],state=hold['state'],nextState=hold['nextState'])
    break
  if first:break
 records.append(first or dict(speed=speed,scenario=r['scenario'],independentCollisionNotFound=True))
(raw/'collision-evidence.json').write_text(json.dumps(records,indent=2)+'\n')
print(json.dumps(records,indent=2))
