"""Summarize an exact-observation, two-stage unified-CLF campaign."""
from pathlib import Path
import json,csv,collections,numpy as np
raw=Path(__file__).parent
rows=[];frames=[];flag_counts=collections.Counter();worst=None
for path in sorted((raw/'campaign').glob('speed*/*.json')):
    r=json.loads(path.read_text())['results'];tr=r['trace'];cfg=r['configuration'];speed=cfg['referenceSpeed']
    runtime=[a['controllerSeconds'] for a in tr];zero=violations=increases=0;largest_relative_defect=None
    for i,a in enumerate(tr):
        stages=a['solverStages'];stages=[stages] if isinstance(stages,dict) else stages
        attempts=a['attempts'];attempts=[attempts] if isinstance(attempts,dict) else attempts
        flag_counts.update(s['exitFlag'] for s in stages)
        init=sum(p['initializationSeconds'] for p in attempts);form=sum(p['formulationSeconds'] for p in attempts)
        clfconstruction=sum(p['clfConstructionSeconds'] for p in attempts)
        pcbf=sum(s['seconds'] for s in stages if s['objective']=='pcbfSlack')
        clf=sum(s['seconds'] for s in stages if s['objective']=='clfSlack')
        scale=max(a['clfInitialValue'],1);tol=cfg['solver']['feasibilityTolerance']*scale
        actual_delta=None;required_defect=None;modeled_zero=a['clfSlack']<=tol
        if i+1<len(tr):
            actual_delta=tr[i+1]['clfInitialValue']-a['clfInitialValue']
            required_defect=actual_delta+a['clfRequiredDecrease']
            if modeled_zero:
                zero+=1;violations+=required_defect>tol;increases+=actual_delta>tol
                if a['clfRequiredDecrease']>1e-5:
                    v=required_defect/a['clfRequiredDecrease']
                    largest_relative_defect=v if largest_relative_defect is None else max(largest_relative_defect,v)
        row=dict(speed=speed,scenario=r['scenario'],time=a['time'],runtime=a['controllerSeconds'],
            px=a['state'][0],py=a['state'][1],yaw=a['state'][2],vx=a['state'][3],vy=a['state'][4],yawRate=a['state'][5],steering=a['input'][0],brakingRatio=a['input'][1],
            initialization=init,formulation=form,clfConstruction=clfconstruction,pcbfSolve=pcbf,clfSolve=clf,
            other=a['controllerSeconds']-init-form-pcbf-clf,solverCalls=a['solverCalls'],flowRestarted=a['flowRestarted'],
            primaryOptimum=a['primaryOptimum'],pcbfSlack=a['predictiveBarrierValue'],clfSlack=a['clfSlack'],
            clfValue=a['clfInitialValue'],modeledNextValue=a['clfNextValue'],requiredDecrease=a['clfRequiredDecrease'],
            actualValueChange=actual_delta,requiredDecreaseDefect=required_defect,modeledZero=modeled_zero,
            tailReached=a['clfTailReached'],nonpositiveFlag=any(s['exitFlag']<=0 for s in stages),
            positiveFinalFlags=a['optimizationConverged'])
        frames.append(row)
        if worst is None or row['runtime']>worst['runtime']:
            worst=dict(row,state=a['state'],input=a['input'],solverStages=stages,attempts=attempts)
    row=dict(speed=speed,scenario=r['scenario'],frames=r['executedFrames'],recovered=r['recovery']['recovered'],
        entryTime=r['recovery']['entryTimeSeconds'],confirmationTime=r['recovery']['confirmationTimeSeconds'],
        minimumGap=r['minimumReplayClearanceMeters'],failure=r['failure'],failedCallSeconds=r['failedFrameSeconds'],
        finalError=r['finalTransverseError'],baselineCollides=r['baselineCruise']['collisionDetected'],
        baselineInitiallySeparated=r['baselineCruise']['initialClearanceMeters']>0,
        medianRuntime=float(np.median(runtime)),p95Runtime=float(np.quantile(runtime,.95)),maxRuntime=max(runtime),
        over100ms=sum(v>.1 for v in runtime),zeroSlackPairs=zero,requiredDecreaseViolations=violations,
        actualIncreasesAtModeledZero=increases,largestRelativeRequiredDefect=largest_relative_defect,
        tailNotReached=sum(not a['clfTailReached'] for a in tr),
        flowRestarts=sum(a['flowRestarted'] for a in tr),conicCalls=sum(a['solverCalls'] for a in tr))
    rows.append(row)
    print(speed,r['scenario'],row['recovered'],row['confirmationTime'],row['minimumGap'],row['maxRuntime'],flush=True)
if not rows:raise SystemExit('No completed result files.')
times=np.array([r['runtime'] for r in frames]);summary=dict(cases=rows,caseCount=len(rows),frames=len(frames),
    recovered=sum(r['recovered'] for r in rows),collisions=sum(r['minimumGap']<=0 for r in rows),
    interrupted=sum(bool(r['failure']) for r in rows),minimumGap=min(r['minimumGap'] for r in rows),
    medianRuntime=float(np.median(times)),p95Runtime=float(np.quantile(times,.95)),maximumRuntime=float(max(times)),
    over100ms=int(sum(times>.1)),over50ms=int(sum(times>.05)),solverFlags=dict(flag_counts),
    conicCalls=sum(r['solverCalls'] for r in frames),flowRestarts=sum(r['flowRestarted'] for r in frames),
    nonpositiveFinalFlagFrames=sum(not r['positiveFinalFlags'] for r in frames),
    zeroSlackPairs=sum(r['zeroSlackPairs'] for r in rows),requiredDecreaseViolations=sum(r['requiredDecreaseViolations'] for r in rows),
    actualIncreasesAtModeledZero=sum(r['actualIncreasesAtModeledZero'] for r in rows),
    tailNotReached=sum(r['tailNotReached'] for r in rows),minimumModeledValue=min(r['modeledNextValue'] for r in frames),worstFrame=worst)
(raw/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
with (raw/'frame-results.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=frames[0].keys(),lineterminator='\n');w.writeheader();w.writerows(frames)
print({k:v for k,v in summary.items() if k not in ['cases','worstFrame']})
