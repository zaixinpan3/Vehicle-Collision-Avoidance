from pathlib import Path
import json, statistics, hashlib
raw=Path(__file__).parent
rows=[]; all_times=[]; worst=None
for speed in (8,15):
 for name in ('headOn','acceleratingHeadOn','brakingLead','crossing','turningCrossing','curvedHeadOn','curvedCrossing'):
  p=raw/'campaign'/f'speed{speed}'/(name+'.json')
  if not p.exists():continue
  r=json.loads(p.read_text())['results'];trace=r['trace'];trace=trace if isinstance(trace,list) else [trace]
  times=[x['controllerSeconds'] for x in trace];all_times+=times
  positive=[x['time'] for x in trace if x['predictiveBarrierValue']>1e-5]
  restarted=[x for x in trace if x.get('flowRestarted')]
  domain=[x for x in trace if x.get('initializationFailure','').startswith('collisionAvoidanceController:')]
  row=dict(speed=speed,scenario=name,holds=r['executedFrames'],seconds=r['executedFrames']*.05,
           completed=r['completed'],recovered=r['recovery']['recovered'],recovery=r['recovery'],
           minimumClearance=r['minimumReplayClearanceMeters'],failure=r['failure'],failureTime=r['failureTime'],failedFrameSeconds=r['failedFrameSeconds'],
           finalError=r['finalTransverseError'],maxSeconds=max(times) if times else None,
           medianSeconds=statistics.median(times) if times else None,over100ms=sum(t>.1 for t in times),
           solverRestarts=len(restarted),domainReinitializations=len(domain),
           nonconverged=sum(not t['optimizationConverged'] for t in trace),
           positiveSafetySlackHolds=len(positive),firstPositiveSlackTime=positive[0] if positive else None,
           firstRestartTimes=[x['time'] for x in restarted[:10]],baselineCollision=r['baselineCruise']['collisionDetected'])
  rows.append(row)
  if times:
   hold=max(trace,key=lambda t:t['controllerSeconds'])
   if worst is None or hold['controllerSeconds']>worst['totalSeconds']:
    attempts=hold['attempts'];attempts=attempts if isinstance(attempts,list) else [attempts]
    init=sum(a['initializationSeconds'] for a in attempts);form=sum(a['formulationSeconds'] for a in attempts)
    solver=sum(a['seconds'] for a in hold['solverStages'])
    worst=dict(speed=speed,scenario=name,time=hold['time'],totalSeconds=hold['controllerSeconds'],initializationSeconds=init,formulationSeconds=form,solverSeconds=solver,otherSeconds=hold['controllerSeconds']-init-form-solver,hold=hold)
summary=dict(cases=rows,totalHolds=len(all_times),completed=sum(x['completed'] for x in rows),recovered=sum(x['recovered'] for x in rows),
 collisionCases=sum(x['minimumClearance'] is not None and x['minimumClearance']<=0 for x in rows),
 noResultCases=sum(bool(x['failure']) for x in rows),medianSeconds=statistics.median(all_times) if all_times else None,
 maxSeconds=max(all_times) if all_times else None,over100ms=sum(t>.1 for t in all_times),
 restarts=sum(x['solverRestarts'] for x in rows),domainReinitializations=sum(x['domainReinitializations'] for x in rows),
 worst=worst)
(raw/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
for r in rows:print(r['speed'],r['scenario'],r['seconds'],'recovered',r['recovered'],'gap',r['minimumClearance'],'restarts',r['solverRestarts'],'domain',r['domainReinitializations'],'error',r['finalError'],'failure',r['failure'])
print({k:v for k,v in summary.items() if k not in ('cases','worst')})
