#!/usr/bin/env python3
"""Summarize the 14-case RTI campaign: outcome, collision, solver behavior per case."""
import json, os, sys, csv, statistics as st, math

W = os.path.expanduser('~/.cache/collisionAvoidance/rti-failure-analysis-20261001')
names = ['headOn', 'acceleratingHeadOn', 'brakingLead', 'crossing', 'turningCrossing', 'curvedHeadOn', 'curvedCrossing']
rows = []
for speed in (8, 15):
    for n in names:
        f = f'{W}/campaign/speed{speed}/{n}.json'
        if not os.path.exists(f):
            continue
        r = json.load(open(f))['results']
        tr = r['trace']
        if isinstance(tr, dict):
            tr = [tr]
        margin = 0.006
        # Per-frame minimum sampled clearance is not stored; use the audit states with target.
        sec = [t['controllerSeconds'] for t in tr]
        prim = [t['primaryOptimum'] if t['primaryOptimum'] is not None else float('nan') for t in tr]
        flags = []
        for t in tr:
            s = t['solverStages']
            if isinstance(s, dict):
                s = [s]
            flags.append(tuple(x['exitFlag'] for x in s))
        positive = [i for i, p in enumerate(prim) if p is not None and not math.isnan(p) and p > 1e-6]
        nonconv = [i for i, f in enumerate(flags) if any(x <= 0 for x in f)]
        restarts = sum(1 for t in tr if t.get('flowRestarted'))
        clf = [t['clfSlack'] for t in tr]
        e = r['finalTransverseError']
        rows.append(dict(
            speed=speed, scenario=n, frames=r['executedFrames'], failure=r['failure'],
            recovered=r['recovery']['recovered'], confirmation=r['recovery']['confirmationTimeSeconds'],
            minClearance=r['minimumReplayClearanceMeters'], collided=r['minimumReplayClearanceMeters'] <= 0,
            belowMargin=r['minimumReplayClearanceMeters'] < margin,
            positivePcbfFrames=len(positive), firstPositivePcbf=(tr[positive[0]]['time'] if positive else None),
            maxPcbf=max([p for p in prim if not math.isnan(p)] or [float('nan')]),
            nonconvergedFrames=len(nonconv), firstNonconverged=(tr[nonconv[0]]['time'] if nonconv else None),
            flagSet=sorted(set(x for f in flags for x in f)), restarts=restarts,
            maxClfSlack=max(clf), finalLateral=e[0], finalHeading=e[1],
            maxFrameSeconds=max(sec), medianFrameSeconds=st.median(sec)))
out = sys.argv[1] if len(sys.argv) > 1 else f'{W}/campaign-summary.csv'
with open(out, 'w', newline='') as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)
for r in rows:
    print(f"{r['speed']:>2} {r['scenario']:<18} frames={r['frames']:>4} rec={str(r['recovered']):5} conf={r['confirmation']} "
          f"gap={r['minClearance']:.4f} pcbf>0 frames={r['positivePcbfFrames']:>4} first={r['firstPositivePcbf']} max={r['maxPcbf']:.3g} "
          f"nonconv={r['nonconvergedFrames']:>4} first={r['firstNonconverged']} flags={r['flagSet']} restarts={r['restarts']} "
          f"maxCLF={r['maxClfSlack']:.3g} ey={r['finalLateral']:.1f} epsi={r['finalHeading']:.2f} fail='{r['failure']}'")
