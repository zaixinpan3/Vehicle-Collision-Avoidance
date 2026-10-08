"""Paired comparison of noisy campaigns (study script, not project code).
Usage: compare.py out-baseline out-v1-set [out-v2-fullfan ...] > comparison.txt
Writes per-run CSV next to the first argument's parent as campaign-runs.csv."""
import json, glob, os, sys, statistics as st, csv
def load(out):
    runs={}
    for f in glob.glob(os.path.join(out,'*-summary.json')):
        r=json.load(open(f)); key=os.path.basename(f)[:-13]
        fr=r.get('frames',[])
        r['frames']=[fr] if isinstance(fr,dict) else (fr or [])
        runs[key]=r
    return runs
def outcome(r):
    if r.get('harnessError'): return 'harness'
    if r.get('collisionDetected'): return 'collision'
    if r.get('recovery',{}).get('recovered'): return 'recovered'
    f=r.get('failure') or ''
    if 'noOptimizationSolution' in f:
        reason=f.split('(')[-1].rstrip(').')
        return 'noSolution:'+reason
    return 'other' if f else 'notRecovered'
outs=sys.argv[1:]
data={os.path.basename(o):load(o) for o in outs}
names=list(data)
keys=sorted(set().union(*[set(v) for v in data.values()]))
print('runs per variant:',{n:len(data[n]) for n in names})
for n in names:
    from collections import Counter
    c=Counter(outcome(r) for r in data[n].values())
    rec=[r for r in data[n].values() if outcome(r)=='recovered']
    gaps=[r['minimumReplayClearanceMeters'] for r in data[n].values() if r.get('minimumReplayClearanceMeters') is not None]
    roads=[r['minimumRoadMarginMeters'] for r in data[n].values() if r.get('minimumRoadMarginMeters') is not None]
    rt=[r['recovery']['confirmationTimeSeconds'] for r in rec if r['recovery'].get('confirmationTimeSeconds') is not None]
    print(f'\n[{n}] outcomes {dict(c)}')
    if gaps: print(f'  min clearance {min(gaps):.3f} m; median per-run minimum {st.median(gaps):.3f} m')
    if roads: print(f'  min road margin {min(roads):.3f} m')
    if rt: print(f'  recovery confirmation time median {st.median(rt):.1f} s')
    obs=[x for r in data[n].values() for x in [f['observerSeconds'] for f in r.get('frames',[])]]
    ctl=[x for r in data[n].values() for x in [f['controllerSeconds'] for f in r.get('frames',[])]]
    if obs: print(f'  observer s/frame median {st.median(obs):.3f} p95 {sorted(obs)[int(.95*len(obs))]:.3f} max {max(obs):.2f}; controller median {st.median(ctl):.3f}')
    # set widths by time since first published target set
    bins=[(0,0.5),(0.5,1.0),(1.0,2.0),(2.0,4.0)]
    acc={b:{'course':[],'speed':[],'acc':[],'curv':[],'slack':[]} for b in bins}
    for r in data[n].values():
        fr=[f for f in r.get('frames',[]) if f.get('targetSet') and f['targetSet'].get('available')]
        if not fr: continue
        t0=fr[0]['time']
        for f in fr:
            dt=f['time']-t0
            for b in bins:
                if b[0]<=dt<b[1]:
                    s=f['targetSet'];acc[b]['course'].append(s['courseRadius']);acc[b]['speed'].append(s['speedHalfWidth'])
                    acc[b]['acc'].append(s['accelerationHalfWidth']);acc[b]['curv'].append(s['curvatureHalfWidth'])
                    if f.get('primaryOptimum') is not None: acc[b]['slack'].append(f['primaryOptimum'])
    if any(acc[b]['course'] for b in bins):
        print('  published set median half-widths by time since first set [course rad | speed m/s | A m/s2 | curvature 1/m | primary slack m]:')
        for b in bins:
            a=acc[b]
            if a['course']:
                print(f"    {b[0]:.1f}-{b[1]:.1f}s  {st.median(a['course']):.3f} | {st.median(a['speed']):.2f} | {st.median(a['acc']):.2f} | {st.median(a['curv']):.4f} | {st.median(a['slack']) if a['slack'] else float('nan'):.3f}  (n={len(a['course'])})")
print('\npaired transitions against', names[0])
base=data[names[0]]
for n in names[1:]:
    better=[k for k in keys if k in base and k in data[n] and outcome(base[k])!='recovered' and outcome(data[n][k])=='recovered']
    worse=[k for k in keys if k in base and k in data[n] and outcome(base[k])=='recovered' and outcome(data[n][k])!='recovered']
    print(f'  {n}: newly recovered {len(better)} {better}\n  {n}: newly failed {len(worse)} {[(k,outcome(data[n][k])) for k in worse]}')
path=os.path.join(os.path.dirname(os.path.abspath(outs[0])),'campaign-runs.csv')
with open(path,'w',newline='') as fh:
    w=csv.writer(fh); w.writerow(['run']+[f'{n}:{c}' for n in names for c in ('outcome','holds','minClearance','minRoad','recoveryTime')])
    for k in keys:
        row=[k]
        for n in names:
            r=data[n].get(k,{})
            row+= [outcome(r) if r else '', r.get('executedFrames',''), r.get('minimumReplayClearanceMeters',''), r.get('minimumRoadMarginMeters',''), (r.get('recovery') or {}).get('confirmationTimeSeconds','')]
        w.writerow(row)
print('\nper-run table:',path)
