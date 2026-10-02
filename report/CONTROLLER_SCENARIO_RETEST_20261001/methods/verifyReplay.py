"""Independently recompute rectangle separation and sampled recovery dwell.

Run after campaign: python3 verifyReplay.py RAW_DIRECTORY
Uses only geometry functions from the frozen Python auditor. Its historical
solver-admission and fixed-buffer verdicts are not used: this experiment asks
for positive actual clearance, not a 5-cm intersample clearance guarantee.
"""
import argparse, json, sys
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('raw',type=Path);a=p.parse_args();raw=a.raw
sys.path.insert(0,str(raw/'source/scripts'))
from auditJointPredictiveSafety import target_state, rectangle, distance
rows=[]
for path in sorted((raw/'campaign').glob('speed*/*.json')):
    r=json.loads(path.read_text())['results'];q=r['targetInitialState'];car=r['configuration']['vehicle']
    shape=(car['length']/2,car['width']/2,*car['rectangleOffset'])
    minimum=float('inf');location=None;count=0;start=None;confirmed=None
    for h in r['trace']:
        for elapsed,x in zip(h['auditTimes'],h['auditStates']):
            time=h['time']+elapsed;target=target_state(q,time)
            gap=distance(rectangle(*x[:3],shape),rectangle(*target[:3],q[7:11]));count+=1
            if gap<minimum:
                minimum=gap;location=dict(time=time,holdTime=h['time'],holdElapsed=elapsed,state=x)
        time=h['time']+r['sampleTimeSeconds'];rec=r['recovery']
        inside=time>=rec['minimumTimeSeconds'] and all(abs(e)<=v for e,v in zip(h['transverseError'],rec['tolerances']))
        if not inside:start=None
        elif start is None:start=time
        if inside and time-start>=rec['dwellSeconds']-1e-10:
            confirmed=time;break
    difference=abs(minimum-r['minimumReplayClearanceMeters'])
    assert difference<1e-8,(path,difference)
    assert (confirmed is not None)==r['recovery']['recovered'],path
    if confirmed is not None:assert abs(confirmed-rec['confirmationTimeSeconds'])<1e-9
    assert count==31*r['executedFrames']
    row=dict(speed=r['configuration']['referenceSpeed'],scenario=r['scenario'],samples=count,
        independentMinimumGap=minimum,matlabMinimumGap=r['minimumReplayClearanceMeters'],difference=difference,
        minimumLocation=location,recoveryConfirmed=confirmed,positiveClearance=minimum>0)
    rows.append(row);print(row['speed'],row['scenario'],count,minimum,difference,flush=True)
assert len(rows)==14
(raw/'replay-verification.json').write_text(json.dumps(rows,indent=2)+'\n')
print('All 14 sampled geometry and dwell comparisons agree.')
