from pathlib import Path
import json,csv,math,hashlib,statistics,sys,collections
import numpy as np
from geometry_batch import sampled_geometry
raw=Path(__file__).parent
sys.path.insert(0,str(raw/'source/scripts'))
from auditJointPredictiveSafety import rectangle,distance,target_state,recovery_completed
out=raw/'analysis';out.mkdir(exist_ok=True)
files=sorted((raw/'campaign').glob('speed*/*.json'));assert len(files)==14
source=json.loads((raw/'source-manifest.json').read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert all(sha(raw/'source'/p)==h for p,h in source['files'].items())
old_root=Path('/home/zai/.cache/collisionAvoidance/two-stage-scenarios-20261001/campaign')
rows=[];frames=[];alltimes=[];worst=None;first_collisions=[]
def penetration(a,b):
    gaps=[]
    for p in (a,b):
        for i,v in enumerate(p):
            w=p[(i+1)%4];n=[v[1]-w[1],w[0]-v[0]];l=math.hypot(*n);n=[v/l for v in n]
            aa=[sum(x*y for x,y in zip(n,v)) for v in a];bb=[sum(x*y for x,y in zip(n,v)) for v in b]
            gaps.append(max(min(aa)-max(bb),min(bb)-max(aa)))
    return max(0,-max(gaps))
for f in files:
    r=json.loads(f.read_text())['results'];tr=r['trace'];tr=[tr] if isinstance(tr,dict) else tr;r['trace']=tr
    assert r['baselineCruise']['collisionDetected'] and r['baselineCruise']['initialClearanceMeters']>0
    assert all(x['solverCalls']==2 for x in tr)
    assert all(x['hardResidual'] is None and not x['affineValidationPerformed'] for x in tr)
    speed=r['configuration']['referenceSpeed'];name=r['scenario'];secs=[x['controllerSeconds'] for x in tr];alltimes+=secs
    old=json.loads((old_root/f'speed{speed}'/f.name).read_text())['results']
    q0=r['targetInitialState'];cfg=r['configuration'];shape=[cfg['vehicle']['length']/2,cfg['vehicle']['width']/2,*cfg['vehicle']['rectangleOffset']]
    times,states,targets,distances,penetrations=sampled_geometry(tr,q0,shape)
    checks=np.unique(np.r_[np.linspace(0,len(times)-1,64,dtype=int),np.argmin(distances),np.argmax(penetrations)])
    for k in checks:
        body=rectangle(*states[k,:3],shape);q=target_state(q0,float(times[k]));obstacle=rectangle(*q[:3],q0[7:11])
        assert abs(distance(body,obstacle)-distances[k])<1e-8
        assert abs(penetration(body,obstacle)-penetrations[k])<1e-8
    d_iter=iter(distances);p_iter=iter(penetrations)
    first=None;maxpenetration=0;gapmin=math.inf
    for i,h in enumerate(tr):
        gap=math.inf;interval_pen=0
        for dt,x in zip(h['auditTimes'],h['auditStates']):
            time=h['time']+dt;d=float(next(d_iter));pen=float(next(p_iter))
            gap=min(gap,d);gapmin=min(gapmin,d)
            if d==0:
                maxpenetration=max(maxpenetration,pen);interval_pen=max(interval_pen,pen)
                if first is None and pen>1e-9:
                    first={'speed':speed,'scenario':name,'time':time,'frame':i+1,'frameTime':h['time'],'pcbfSlack':h['predictiveBarrierValue'],'primaryValue':h['primaryOptimum'],'clfSlack':h['clfSlack'],'solverFlags':[s['exitFlag'] for s in h['solverStages']],'egoState':x,'targetState':target_state(q0,time),'penetrationMeters':pen}
        stage=h['solverStages'];z=dict(speed=speed,scenario=name,frame=i+1,time=h['time'],controllerSeconds=h['controllerSeconds'],pcbfSeconds=stage[0]['seconds'],clfSeconds=stage[1]['seconds'],otherControllerSeconds=h['controllerSeconds']-sum(s['seconds'] for s in stage),pcbfFlag=stage[0]['exitFlag'],clfFlag=stage[1]['exitFlag'],pcbfSlack=h['predictiveBarrierValue'],clfSlack=h['clfSlack'],clfInitialValue=h['clfInitialValue'],clfNextValue=h['clfNextValue'],minimumIntervalClearanceMeters=gap,maximumIntervalPenetrationMeters=interval_pen,steering=h['input'][0],brakingRatio=h['input'][1],lateralError=h['transverseError'][0],headingError=h['transverseError'][1],speedError=h['transverseError'][2])
        frames.append(z)
        if worst is None or z['controllerSeconds']>worst['controllerSeconds']:worst=z
    if first:first_collisions.append(first)
    assert abs(gapmin-r['minimumReplayClearanceMeters'])<1e-8
    recovered=recovery_completed(r);assert recovered==r['recovery']['recovered']
    failure=r['failure'];stage=failure.split('(')[-1].rstrip(').') if failure else ''
    row=dict(speed=speed,scenario=name,formerFailure='CLF' if 'clfSolveFailed' in old['failure'] else 'PCBF',frames=len(tr),durationSeconds=len(tr)*r['sampleTimeSeconds'],durationCompleted=r['completed'],recovered=recovered,recoveryConfirmationSeconds=r['recovery']['confirmationTimeSeconds'],avoidanceAndRecovery=bool(recovered and gapmin>0 and not failure),strictPhysicalOverlap=first is not None,firstOverlapSeconds=first['time'] if first else None,minimumClearanceMeters=gapmin,configuredMarginSatisfied=gapmin>=cfg['collision']['safetyMarginMeters']-1e-9,maximumPenetrationMeters=maxpenetration,failureStage=stage,failureTime=r['failureTime'],failedFrameSeconds=r['failedFrameSeconds'],maximumReturnedSeconds=max(secs,default=0),maximumIncludingFailureSeconds=max(secs+([r['failedFrameSeconds']] if r['failedFrameSeconds'] is not None else []),default=0),medianReturnedSeconds=statistics.median(secs) if secs else None,returnedFramesOver100ms=sum(t>.1 for t in secs),nonpositiveFlagFrames=sum(any(s['exitFlag']<=0 for s in h['solverStages']) for h in tr),maximumSafetySlack=max((h['predictiveBarrierValue'] for h in tr),default=0),finalLateralErrorMeters=r['finalTransverseError'][0] if tr else None,finalHeadingErrorRadians=r['finalTransverseError'][1] if tr else None,finalSpeedErrorMetersPerSecond=r['finalTransverseError'][2] if tr else None,finalClfSlack=tr[-1]['clfSlack'] if tr else None,finalClfValue=r['finalClfValue'],failure=failure,rawFile=str(f),rawSha256=sha(f))
    if row['formerFailure']=='PCBF':
        oldtr=old['trace'];oldtr=[oldtr] if isinstance(oldtr,dict) else oldtr
        assert len(tr)==len(oldtr) and abs(r['failureTime']-old['failureTime'])<1e-12
        row['maximumStateDifferenceFromPriorCampaign']=max(abs(a-b) for new,prior in zip(tr,oldtr) for a,b in zip(new['state'],prior['state']))
        assert row['maximumStateDifferenceFromPriorCampaign']<1e-10
    rows.append(row)
    print('AUDIT',speed,name,'duration',row['durationSeconds'],'recovery',recovered,'overlap',first is not None,'gap',gapmin,'error',row['finalLateralErrorMeters'],'failure',stage,flush=True)
failed=[r for r in rows if r['failure']];former=[r for r in rows if r['formerFailure']=='CLF']
summary=dict(numpyVersion=np.__version__,sourceCommit=source['baseCommit'],cases=len(rows),durationCompleted=sum(r['durationCompleted'] for r in rows),recovered=sum(r['recovered'] for r in rows),avoidanceAndRecovery=sum(r['avoidanceAndRecovery'] for r in rows),physicalOverlapCases=len(first_collisions),formerClfCases=11,formerClfRecovered=sum(r['recovered'] for r in former),formerClfAvoidanceAndRecovery=sum(r['avoidanceAndRecovery'] for r in former),returnedFrames=len(frames),medianReturnedSeconds=statistics.median(alltimes),maximumReturnedSeconds=max(alltimes),maximumFailedFrameSeconds=max((r['failedFrameSeconds'] for r in failed),default=None),maximumAllCallsSeconds=max(r['maximumIncludingFailureSeconds'] for r in rows),returnedFramesOver100ms=sum(t>.1 for t in alltimes),failedCallsOver100ms=sum(r['failedFrameSeconds']>.1 for r in failed),failureStages=dict(collections.Counter(r['failureStage'] for r in failed)),worstReturnedFrame=worst,firstCollisions=first_collisions,results=rows)
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
for name,data in [('results.csv',rows),('frames.csv',frames)]:
 with (out/name).open('w') as f:
  fields=[k for k in data[0] if k not in ['rawFile','rawSha256','failure']];w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore',lineterminator='\n');w.writeheader();w.writerows(data)
print(json.dumps({k:v for k,v in summary.items() if k not in ['results','firstCollisions','worstReturnedFrame']},indent=2))
