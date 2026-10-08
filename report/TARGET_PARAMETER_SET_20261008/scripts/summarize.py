import json, glob, sys, os, statistics as st
out=sys.argv[1]
rows=[]
for f in sorted(glob.glob(os.path.join(out,'*-summary.json'))):
    r=json.load(open(f)); rows.append(r)
def outcome(r):
    if r.get('harnessError'): return 'harness'
    if r.get('collisionDetected'): return 'collision'
    if r.get('recovery',{}).get('recovered'): return 'recovered'
    fail=r.get('failure') or ''
    if 'noOptimizationSolution' in fail: return 'noSolution'
    if fail: return 'error'
    return 'notRecovered'
from collections import Counter
c=Counter(outcome(r) for r in rows)
print(out.split('/')[-1], 'runs',len(rows), dict(c))
gaps=[r['minimumReplayClearanceMeters'] for r in rows if 'minimumReplayClearanceMeters' in r and r['minimumReplayClearanceMeters'] is not None]
if gaps: print(' min clearance %.3f m, median of per-run minima %.3f m'%(min(gaps),st.median(gaps)))
road=[r['minimumRoadMarginMeters'] for r in rows if r.get('minimumRoadMarginMeters') is not None]
if road: print(' min road margin %.3f m'%min(road))
for r in rows:
    o=outcome(r)
    if o!='recovered':
        print('  ',r.get('seed'),r.get('speed'),r.get('scenario',r.get('name','')),o,(r.get('failure') or '')[:110], 'holds',r.get('executedFrames'))
wall=[r.get('wallSeconds',0) for r in rows]; print(' wall s median %.0f max %.0f'%(st.median(wall),max(wall)))
