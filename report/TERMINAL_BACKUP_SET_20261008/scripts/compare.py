"""Paired comparison of terminal-set campaign variants (study script).
Usage: compare.py out-base out-A [out-D ...]; seed 0 = exact states, others noisy.
Writes campaign-runs.csv next to the first argument."""
import json, glob, os, sys, statistics as st, csv
from collections import Counter
def load(out):
    runs={}
    for f in glob.glob(os.path.join(out,'*-summary.json')):
        r=json.load(open(f)); key=os.path.basename(f)[:-13]
        fr=r.get('frames',[]); r['frames']=[fr] if isinstance(fr,dict) else (fr or [])
        runs[key]=r
    return runs
def outcome(r):
    if r.get('harnessError'): return 'harness'
    if r.get('collisionDetected'): return 'collision'
    if (r.get('recovery') or {}).get('recovered'): return 'recovered'
    f=r.get('failure') or ''
    if 'noOptimizationSolution' in f: return 'noSolution:'+f.split('(')[-1].rstrip(').')
    return 'other' if f else 'notRecovered'
outs=sys.argv[1:]
data={os.path.basename(o):load(o) for o in outs}
names=list(data)
for kind,sel in (('exact',lambda k:k.startswith('seed0-')),('noisy',lambda k:not k.startswith('seed0-'))):
    print(f'===== {kind} =====')
    for n in names:
        runs={k:v for k,v in data[n].items() if sel(k)}
        if not runs: continue
        c=Counter(outcome(r) for r in runs.values())
        rec=sum(1 for r in runs.values() if outcome(r)=='recovered')
        gaps=[r['minimumReplayClearanceMeters'] for r in runs.values() if r.get('minimumReplayClearanceMeters') is not None]
        roads=[r['minimumRoadMarginMeters'] for r in runs.values() if r.get('minimumRoadMarginMeters') is not None]
        frames=[f for r in runs.values() for f in r['frames']]
        lanes=[f.get('terminalLaneOffset') for f in frames if f.get('terminalLaneOffset') is not None]
        off=sum(1 for v in lanes if abs(v)>1e-9)
        usedRuns=sum(1 for r in runs.values() if any(f.get('terminalLaneOffset') not in (None,) and abs(f.get('terminalLaneOffset') or 0)>1e-9 for f in r['frames']))
        ctl=sorted(f['controllerSeconds'] for f in frames if f.get('controllerSeconds') is not None)
        print(f'[{n}] runs {len(runs)} recovered {rec}  {dict(c)}')
        if gaps: print(f'   min clearance {min(gaps):.3f} m; median per-run minimum {st.median(gaps):.3f} m' + (f'; min road margin {min(roads):.3f} m' if roads else ''))
        if lanes: print(f'   terminal lane != 0 in {off}/{len(lanes)} frames, {usedRuns} runs; lanes used {sorted(Counter(round(v,2) for v in lanes).items())}')
        if ctl: print(f'   controller s median {st.median(ctl):.3f} p95 {ctl[int(.95*len(ctl))]:.3f} max {ctl[-1]:.2f}')
    base=data[names[0]]
    for n in names[1:]:
        keys=[k for k in base if sel(k) and k in data[n]]
        better=[k for k in keys if outcome(base[k])!='recovered' and outcome(data[n][k])=='recovered']
        worse=[(k,outcome(data[n][k])) for k in keys if outcome(base[k])=='recovered' and outcome(data[n][k])!='recovered']
        print(f'  {n} vs {names[0]}: newly recovered {len(better)} {better}')
        print(f'  {n} vs {names[0]}: newly failed {len(worse)} {worse}')
path=os.path.join(os.path.dirname(os.path.abspath(outs[0])),'campaign-runs.csv')
keys=sorted(set().union(*[set(v) for v in data.values()]))
with open(path,'w',newline='') as fh:
    w=csv.writer(fh); w.writerow(['run']+[f'{n}:{c}' for n in names for c in ('outcome','holds','minClearance','minRoad','recoveryTime')])
    for k in keys:
        row=[k]
        for n in names:
            r=data[n].get(k,{})
            row+=[outcome(r) if r else '',r.get('executedFrames',''),r.get('minimumReplayClearanceMeters',''),r.get('minimumRoadMarginMeters',''),(r.get('recovery') or {}).get('confirmationTimeSeconds','')]
        w.writerow(row)
print('per-run table:',path)
