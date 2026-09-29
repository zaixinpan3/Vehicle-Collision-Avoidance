from pathlib import Path
import json,sys,csv,hashlib,collections,math
root=Path('/home/zai/Downloads/ResearchProjects/collisionAvoidance');raw=Path(__file__).parent
sys.path.insert(0,str(root/'scripts'))
from auditJointPredictiveSafety import rectangle,target_state,distance
summaries=[];evaluations=[];iterations=[]
for file in sorted(raw.glob('speed*-budget*.json')):
 r=json.loads(file.read_text());d=r['diagnostic'];seed=d['seedAttempts'];ev=d['evaluations'];solves=d['solves']
 item=dict(scenario=r['scenario'],speed=r['speed'],budget=r['budget'],instrumentedSeconds=r['seconds'],failure=r['failure'],
  solverCalls=len(solves),conicSeconds=sum(s['seconds'] for s in solves),evaluationCount=len(ev),seedSeconds=d['seedSeconds'],
  seedAttempts=len(seed),lastSeedTimeSeconds=seed[-1]['index']*.05,
  minimumEndpointSeparation=min(s['separation'] for s in seed),maximumEndpointSeparation=max(s['separation'] for s in seed),
  solverFlagCounts=dict(collections.Counter(str(s['flag']) for s in solves)),termination=d.get('search',{}).get('terminationReason'),
  initialEvaluation=ev[0] if ev else None,bestEvaluation=min(ev,key=lambda e:e['hard']) if ev else None)
 summaries.append(item)
 for i,e in enumerate(ev):evaluations.append(dict(speed=r['speed'],scenario=r['scenario'],budget=r['budget'],evaluation=i+1,**e))
 for e in d.get('search',{}).get('sequentialIterations',[]):iterations.append(dict(speed=r['speed'],scenario=r['scenario'],budget=r['budget'],**e))
checks=[]
for file in sorted(list(raw.glob('replay-*.json'))+list(raw.glob('original-replay-*.json'))):
 r=json.loads(file.read_text());q=r['targetInitialState'];v=r['configuration']['vehicle'];shape=[v['length']/2,v['width']/2,*v['rectangleOffset']]
 minimum=float('inf');road=float('inf')
 for time,state in zip(r['times'],r['states']):
  body=rectangle(*state[:3],shape);other=rectangle(*target_state(q,time)[:3],q[7:11]);minimum=min(minimum,distance(body,other));road=min(road,4-max(abs(y) for x,y in body))
 assert (minimum==0 if file.name.startswith('original-') else minimum>0)
 assert road>=0 and abs(minimum-r['metrics']['ode45MinimumDistance'])<1e-9
 assert abs(road-r['metrics']['ode45MinimumRoadMargin'])<1e-9
 checks.append(dict(replay=file.name,samples=len(r['times']),minimumDistance=minimum,minimumRoadMargin=road,geometryAgrees=True,collisionFree=minimum>0))
# Audit strict overlap separately from unsigned rectangle distance.
r=json.loads((raw/'original-replay-8-turningCrossing.json').read_text())
q=r['targetInitialState'];v=r['configuration']['vehicle'];shape=[v['length']/2,v['width']/2,*v['rectangleOffset']]
overlaps=[];node_minimum=float('inf')
for time,state in zip(r['times'],r['states']):
 a=rectangle(*state[:3],shape);b=rectangle(*target_state(q,time)[:3],q[7:11]);depth=float('inf')
 for poly in (a,b):
  for i,p in enumerate(poly):
   end=poly[(i+1)%4];nx,ny=p[1]-end[1],end[0]-p[0];length=math.hypot(nx,ny);nx/=length;ny/=length
   pa=[nx*x+ny*y for x,y in a];pb=[nx*x+ny*y for x,y in b]
   depth=min(depth,max(pa)-min(pb),max(pb)-min(pa))
 if depth>1e-9:overlaps.append(dict(time=time,penetration=depth))
 if abs(time/.025-round(time/.025))<1e-8:node_minimum=min(node_minimum,distance(a,b))
assert len(overlaps)==8 and node_minimum>.006
collision=dict(minimumClearance=0.,nodeMidpointMinimum=node_minimum,overlapCount=len(overlaps),
 firstOverlap=overlaps[0],lastOverlap=overlaps[-1],maximumSatPenetration=max(v['penetration'] for v in overlaps))
assert abs(collision['maximumSatPenetration']-.037025450138175486)<1e-10
(raw/'extended-budget-intersample-collision.json').write_text(json.dumps(collision,indent=2)+'\n')
summary=dict(scope='Instrumented replay and controlled diagnostic probes; no production controller mutation',
 sourceCommit='17c5a756bcfa8abb183510abf08e972d48a57bb3',diagnosticRuns=summaries,
 nodeControls=json.loads((raw/'single-node-controls.json').read_text()),
 curveControls=[json.loads(p.read_text()) for p in sorted(raw.glob('curve-*.json'))],
 affineControls=json.loads((raw/'affine-controls.json').read_text()),
 witnessControls=json.loads((raw/'witness-verification.json').read_text()),
 extendedBudgetWitness=json.loads((raw/'extended-budget-verification.json').read_text()),
 intersampleCollision=json.loads((raw/'extended-budget-intersample-collision.json').read_text()),
 independentReplayChecks=checks)
(raw/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
for name,records in [('evaluations.csv',evaluations),('iterations.csv',iterations)]:
 columns=list(dict.fromkeys(k for r in records for k in r))
 with (raw/name).open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=columns);w.writeheader();w.writerows(records)
for name,digest in json.loads((raw/'source-manifest.json').read_text()).items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
(raw/'raw-artifact-hashes.json').write_text(json.dumps([dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in sorted(raw.iterdir()) if p.is_file() and p.suffix in ['.mat','.m','.py','.json','.patch','.csv'] and p.name!='raw-artifact-hashes.json'],indent=2)+'\n')
print('Verified unchanged production sources,',len(summaries),'diagnostic runs,',len(evaluations),'evaluations,',len(iterations),'iterations,',len(checks),'independent nonlinear witness replays.')
print('Original-problem extended-budget witness:',summary['extendedBudgetWitness'])
