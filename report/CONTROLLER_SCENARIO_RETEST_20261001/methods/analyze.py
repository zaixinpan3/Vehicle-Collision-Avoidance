"""Summarize a fresh controller campaign without changing the online algorithm.

Usage: python3 analyze.py RAW_DIRECTORY [--partial]
Requires the frozen source/scripts/auditJointPredictiveSafety.py for independent
rectangle geometry. Full per-hold ODE samples remain in the raw campaign JSON.
"""
from pathlib import Path
import argparse, csv, json, statistics, sys
from collections import Counter

p = argparse.ArgumentParser()
p.add_argument('raw', type=Path)
p.add_argument('--partial', action='store_true')
args = p.parse_args()
raw = args.raw
sys.path.insert(0, str(raw/'source/scripts'))
from auditJointPredictiveSafety import target_state, rectangle, distance
names = ['headOn', 'acceleratingHeadOn', 'brakingLead', 'crossing',
         'turningCrossing', 'curvedHeadOn', 'curvedCrossing']

def listed(v):
    return v if isinstance(v, list) else [v]

def stats(v):
    s = sorted(v)
    if not s:
        return None
    # Linear interpolation between order statistics, explicitly recorded.
    k = .95*(len(s)-1); i = int(k)
    p95 = s[i] if i+1 == len(s) else s[i]+(k-i)*(s[i+1]-s[i])
    return dict(count=len(v), median=statistics.median(v), p95=p95,
                maximum=max(v), over100ms=sum(x > .1 for x in v),
                over50ms=sum(x > .05 for x in v))

def write_csv(name, rows):
    with (raw/name).open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator='\n')
        w.writeheader()
        w.writerows({k: json.dumps(v) if isinstance(v, (list, dict)) else v
                    for k,v in r.items()} for r in rows)

cases, frames, worst, restarts, configs = [], [], None, [], []
flags = Counter()
for speed in (8, 15):
    for name in names:
        path = raw/'campaign'/f'speed{speed}'/(name+'.json')
        if args.partial and not path.exists():
            continue
        r = json.loads(path.read_text())['results']
        trace = listed(r['trace'])
        cfg = r['configuration']; car = cfg['vehicle']; q = r['targetInitialState']
        shape = (car['length']/2, car['width']/2, *car['rectangleOffset'])
        configs.append(dict(speed=speed, scenario=name, configuration=cfg,
                            targetInitialState=q, recovery=r['recovery']))
        times, free, gaps, indices = [], [], [], []
        for i, h in enumerate(trace):
            t = h['time']; duration = h['controllerSeconds']
            times.append(duration)
            target = target_state(q, t)
            gap = distance(rectangle(*h['state'][:3], shape), rectangle(*target[:3], q[7:11]))
            gaps.append(gap)
            stages = listed(h['solverStages']); attempts = listed(h['attempts'])
            init = sum(a['initializationSeconds'] for a in attempts)
            form = sum(a['formulationSeconds'] for a in attempts)
            elapsed = {key: sum(s['seconds'] for s in stages if s['objective']==key)
                       for key in ('pcbfSlack', 'clfSlack', 'anchorDeviation')}
            assert abs(sum(elapsed.values())-sum(s['seconds'] for s in stages)) < 1e-10
            flags.update(f"{s['objective']}:{s['exitFlag']}" for s in stages)
            row = dict(speed=speed, scenario=name, time=t, controllerSeconds=duration,
                initializationSeconds=init, formulationSeconds=form,
                **{key+'Seconds': value for key,value in elapsed.items()},
                otherSeconds=duration-init-form-sum(elapsed.values()),
                solverCalls=h['solverCalls'], flowRestarted=h['flowRestarted'],
                solverFlags=[s['exitFlag'] for s in stages],
                initialization=h['initialization'], clfFunction=h['clfFunction'],
                safetySlack=h['predictiveBarrierValue'], clfSlack=h['clfSlack'],
                clfValue=h['clfInitialValue'], clfRequiredDecrease=h['clfRequiredDecrease'],
                startGap=gap, transverseError=h['transverseError'])
            frames.append(row)
            if worst is None or duration > worst['controllerSeconds']:
                worst = dict(row, stages=stages, attempts=attempts, state=h['state'], input=h['input'])
            if h['flowRestarted']:
                restarts.append(dict(row, attempts=attempts))
            if h['clfFunction']=='recoveryCostToGo':
                free.append(duration); indices.append(i)
        zero = sum(trace[i]['clfSlack'] <= 1e-5*max(1,trace[i]['clfInitialValue']) for i in indices)
        pairs = [(i,i+1) for i in indices if i+1<len(trace) and trace[i+1]['clfFunction']=='recoveryCostToGo']
        decreases = sum(trace[j]['clfInitialValue'] <= trace[i]['clfInitialValue']
            - trace[i]['clfRequiredDecrease'] + 1e-6*max(1,trace[i]['clfInitialValue']) for i,j in pairs)
        rec = r['recovery']; baseline=r['baselineCruise']
        row = dict(speed=speed, scenario=name, holds=len(trace), duration=len(trace)*r['sampleTimeSeconds'],
            recovered=rec['recovered'], recoveryEntry=rec['entryTimeSeconds'],
            recoveryConfirmation=rec['confirmationTimeSeconds'], minimumGap=r['minimumReplayClearanceMeters'],
            failure=r['failure'], failureTime=r['failureTime'], failedCallSeconds=r['failedFrameSeconds'],
            finalError=r['finalTransverseError'], finalGap=distance(rectangle(*trace[-1]['nextState'][:3],shape),
                rectangle(*target_state(q,len(trace)*r['sampleTimeSeconds'])[:3],q[7:11])),
            frameStartGapMinimum=min(gaps), frameStartGapMaximum=max(gaps),
            recoveryClfFrames=len(indices), recoveryClfZeroSlack=zero,
            recoveryClfConsecutivePairs=len(pairs), recoveryClfDecreasePairs=decreases,
            firstRecoveryClfTime=trace[indices[0]]['time'] if indices else None,
            nonconvergedFrames=sum(not h['optimizationConverged'] for h in trace),
            nonpositiveStageFrames=sum(any(stage['exitFlag']<=0 for stage in listed(h['solverStages'])) for h in trace),
            flowRestarts=sum(h['flowRestarted'] for h in trace),
            domainReinitializations=sum(h.get('initializationFailure','').startswith('collisionAvoidanceController:') for h in trace),
            positiveSafetySlackFrames=sum(h['predictiveBarrierValue']>1e-5 for h in trace),
            maximumSafetySlack=max(h['predictiveBarrierValue'] for h in trace),
            baselineCollides=baseline['collisionDetected'], baselineInitialGap=baseline['initialClearanceMeters'],
            runtime=stats(times), recoveryRuntime=stats(free))
        assert row['holds']==r['executedFrames']
        assert max(times)==r['maximumFrameSeconds']
        cases.append(row)
        print(speed, name, 'recover',row['recoveryConfirmation'],'gap',round(row['minimumGap'],6),
              'max',round(max(times),6),'failure',row['failure'],flush=True)
summary = dict(caseCount=len(cases), totalHolds=len(frames),
    recovered=sum(c['recovered'] for c in cases), collisionCases=sum(c['minimumGap']<=0 for c in cases),
    interruptedCases=sum(bool(c['failure']) for c in cases),
    allBaselinesCollide=all(c['baselineCollides'] and c['baselineInitialGap']>0 for c in cases),
    runtime=stats([f['controllerSeconds'] for f in frames]),
    encounterRuntime=stats([f['controllerSeconds'] for f in frames if f['clfFunction']=='laneQuadratic']),
    recoveryRuntime=stats([f['controllerSeconds'] for f in frames if f['clfFunction']=='recoveryCostToGo']),
    solverExitFlags=dict(flags), flowRestarts=restarts, cases=cases, worstFrame=worst)
(raw/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
(raw/'configurations.json').write_text(json.dumps(configs,indent=2)+'\n')
write_csv('scenario-results.csv',cases)
write_csv('frame-results.csv',frames)
print({k:v for k,v in summary.items() if k not in ['cases','worstFrame','flowRestarts']})
