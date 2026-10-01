#!/usr/bin/env python3
"""Compare the 5-cm campaign with the 6-mm campaign of commit 5f74977 (same driver, same fixtures)."""
import json, os, csv, math, statistics as st, sys
old = os.path.expanduser('~/.cache/collisionAvoidance/rti-failure-analysis-20261001/campaign')
new = os.path.expanduser('~/.cache/collisionAvoidance/margin-5cm-20261001/campaign')
names = ['headOn', 'acceleratingHeadOn', 'brakingLead', 'crossing', 'turningCrossing', 'curvedHeadOn', 'curvedCrossing']
def stages(t):
    s = t['solverStages']
    return s if isinstance(s, list) else [s]
def load(root, speed, n):
    f = f'{root}/speed{speed}/{n}.json'
    if not os.path.exists(f):
        return None
    r = json.load(open(f))['results']
    tr = r['trace'] if isinstance(r['trace'], list) else [r['trace']]
    prim = [t['primaryOptimum'] for t in tr if t['primaryOptimum'] is not None]
    return dict(margin=r['configuration']['collision']['safetyMarginMeters'], frames=r['executedFrames'], failure=r['failure'],
                failureTime=r['failureTime'], recovered=r['recovery']['recovered'], confirmation=r['recovery']['confirmationTimeSeconds'],
                minGap=r['minimumReplayClearanceMeters'], finalLateral=r['finalTransverseError'][0], finalSpeed=r['finalTransverseError'][2],
                relaxedFrames=sum(1 for p in prim if p > 1e-5), maxPcbf=max(prim) if prim else float('nan'),
                clfMinus7=sum(1 for t in tr if any(s['exitFlag'] == -7 for s in stages(t)[1:2])),
                restarts=sum(1 for t in tr if t.get('flowRestarted')),
                maxSeconds=max(t['controllerSeconds'] for t in tr), medianSeconds=st.median(t['controllerSeconds'] for t in tr))
rows = []
for speed in (8, 15):
    for n in names:
        a, b = load(old, speed, n), load(new, speed, n)
        for tag, r in (('6mm', a), ('50mm', b)):
            if r is None:
                continue
            rows.append(dict(speed=speed, scenario=n, run=tag, **r))
out = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/.cache/collisionAvoidance/margin-5cm-20261001/comparison.csv')
with open(out, 'w', newline='') as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)
for r in rows:
    print(f"{r['speed']:>2} {r['scenario']:<18} {r['run']:>4} frames={r['frames']:>4} rec={str(r['recovered']):5} conf={r['confirmation']} "
          f"gap={r['minGap']:.4f} ey={r['finalLateral']:9.2f} ev={r['finalSpeed']:6.2f} relaxed={r['relaxedFrames']:>3} maxPcbf={r['maxPcbf']:.3g} "
          f"-7={r['clfMinus7']:>3} restarts={r['restarts']} max={r['maxSeconds']:.2f}s fail='{r['failure'][:60]}'")
