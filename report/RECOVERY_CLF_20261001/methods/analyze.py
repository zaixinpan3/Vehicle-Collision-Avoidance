"""Compare the recovery-CLF campaign with the 5-cm baseline campaign.

For every case: outcome, recovery time, minimum replay clearance and controller
time. For target-free frames of the new campaign (clfFunction ==
recoveryCostToGo): CLF slack, and the decrease of the recovery CLF between
consecutive frames measured at the ODE45 plant states (clfInitialValue of the
next frame), against the required decrease and the hold-model decrease l(x).
"""
import json, csv, math, sys
from pathlib import Path

NEW = Path('/home/zai/.cache/collisionAvoidance/recovery-clf-20261001/campaign')
OLD = Path('/home/zai/.cache/collisionAvoidance/margin-5cm-20261001/campaign')
OUT = Path('/home/zai/.cache/collisionAvoidance/recovery-clf-20261001')
SCENARIOS = ["headOn", "acceleratingHeadOn", "brakingLead", "crossing", "turningCrossing", "curvedHeadOn", "curvedCrossing"]
ETA = 0.5  # recovery.decreaseFraction


def load(path):
    d = json.loads(path.read_text())
    r = d['results']
    return r[0] if isinstance(r, list) else r


def outcome(r):
    rec = r['recovery']
    t = rec.get('confirmationTimeSeconds') if rec.get('recovered') else None
    trace = r['trace']
    restarts = sum(1 for e in trace if e.get('flowRestarted'))
    return dict(recovered=bool(rec.get('recovered')), recoveredAt=t, frames=r['executedFrames'],
                minGap=r['minimumReplayClearanceMeters'], maxSeconds=r['maximumFrameSeconds'],
                restarts=restarts, failure=r.get('failure') or '', finalError=r.get('finalTransverseError'))


rows = []
for speed in (8, 15):
    for name in SCENARIOS:
        new = load(NEW / f'speed{speed}' / f'{name}.json')
        old = load(OLD / f'speed{speed}' / f'{name}.json')
        o, n = outcome(old), outcome(new)
        trace = new['trace']
        rec = [i for i, e in enumerate(trace) if e.get('clfFunction') == 'recoveryCostToGo']
        slackOk = [trace[i]['clfSlack'] <= 1e-5 * max(1.0, trace[i]['clfInitialValue']) for i in rec]
        pairs = [(i, i + 1) for i in rec if i + 1 < len(trace) and trace[i + 1].get('clfFunction') == 'recoveryCostToGo']
        meets, falls, rel = 0, 0, []
        for i, j in pairs:
            v0, v1 = trace[i]['clfInitialValue'], trace[j]['clfInitialValue']
            req = trace[i]['clfRequiredDecrease']
            if v1 <= v0 - req + 1e-6 * max(1.0, v0):
                meets += 1
            if v1 < v0 or v0 == 0:
                falls += 1
            stage = req / ETA
            if stage > 1e-6:
                rel.append(abs(v1 - (v0 - stage)) / stage)
        firstFree = trace[rec[0]]['time'] if rec else None
        times = [e['controllerSeconds'] for e in trace]
        recTimes = [trace[i]['controllerSeconds'] for i in rec]
        rows.append(dict(
            speed=speed, scenario=name,
            baselineRecovered=o['recovered'], baselineRecoveredAt=o['recoveredAt'], baselineMinGap=o['minGap'],
            recovered=n['recovered'], recoveredAt=n['recoveredAt'], minGap=n['minGap'], frames=n['frames'],
            failure=n['failure'], restarts=n['restarts'], baselineRestarts=o['restarts'],
            firstTargetFreeFrame=firstFree, targetFreeFrames=len(rec),
            shareZeroSlack=(sum(slackOk) / len(slackOk)) if slackOk else None,
            consecutivePairs=len(pairs), shareMeetsRequiredDecrease=(meets / len(pairs)) if pairs else None,
            shareDecreasing=(falls / len(pairs)) if pairs else None,
            medianPlantVersusModelRelative=(sorted(rel)[len(rel) // 2]) if rel else None,
            maxPlantVersusModelRelative=max(rel) if rel else None,
            maxControllerSeconds=max(times), medianTargetFreeSeconds=(sorted(recTimes)[len(recTimes) // 2]) if recTimes else None,
            maxTargetFreeSeconds=max(recTimes) if recTimes else None,
            finalError=n['finalError']))

keys = list(rows[0].keys())
with (OUT / 'campaign-comparison.csv').open('w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=keys)
    w.writeheader()
    for r in rows:
        w.writerow({k: (json.dumps(v) if isinstance(v, list) else v) for k, v in r.items()})


def fmt(v, p=2):
    if v is None:
        return '---'
    if isinstance(v, bool):
        return 'yes' if v else 'no'
    if isinstance(v, float):
        return f'{v:.{p}f}'
    return str(v)


for r in rows:
    print(f"{r['speed']:>2} {r['scenario']:<18} base rec {fmt(r['baselineRecoveredAt'])} gap {fmt(r['baselineMinGap'],3)} | "
          f"new rec {fmt(r['recoveredAt'])} gap {fmt(r['minGap'],3)} frames {r['frames']} fail '{r['failure']}' restarts {r['restarts']}/{r['baselineRestarts']} | "
          f"free from {fmt(r['firstTargetFreeFrame'])} n={r['targetFreeFrames']} rho0 {fmt(r['shareZeroSlack'],3)} "
          f"meets {fmt(r['shareMeetsRequiredDecrease'],3)} falls {fmt(r['shareDecreasing'],3)} plant/model med {fmt(r['medianPlantVersusModelRelative'],4)} "
          f"max {fmt(r['maxPlantVersusModelRelative'],3)} | t med {fmt(r['medianTargetFreeSeconds'],3)} max {fmt(r['maxControllerSeconds'],3)}")
