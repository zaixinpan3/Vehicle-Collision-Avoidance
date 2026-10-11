"""Paired comparison of the stop-at-rest target contract against the signed-speed one.
Usage: python3 -I compare.py <out-signed> <out-stop> <output-directory>
Both arms run the braking-continuation terminal set. Seed 0 runs are exact
states, the others noisy. Writes comparison.txt, campaign-runs.csv and
failures.txt to the output directory and prints the comparison."""
import csv, glob, json, os, statistics as st, sys
from collections import Counter


def load(out):
    runs = {}
    for f in glob.glob(os.path.join(out, '*-summary.json')):
        r = json.load(open(f))
        fr = r.get('frames') or []
        r['frames'] = [fr] if isinstance(fr, dict) else fr
        runs[os.path.basename(f)[:-13]] = r
    return runs


def outcome(r):
    if not r: return 'missing'
    if r.get('harnessError'): return 'harness'
    if r.get('collisionDetected'): return 'collision'
    if (r.get('recovery') or {}).get('recovered'): return 'recovered'
    f = r.get('failure') or ''
    if 'noOptimizationSolution' in f: return 'noSolution:' + f.split('(')[-1].rstrip(').')
    return 'other:' + f[:60] if f else 'notRecovered'


def quantile(a, p):
    a = sorted(a)
    if not a: return float('nan')
    x = p * (len(a) - 1); i = int(x); f = x - i
    return a[i] if i + 1 >= len(a) else a[i] + (a[i + 1] - a[i]) * f


def scenario(key):
    return key.split('-', 2)[2]


def main():
    base, new, target = sys.argv[1], sys.argv[2], sys.argv[3]
    data = {'signed': load(base), 'stop': load(new)}
    lines = []
    out = lines.append
    for kind, sel in (('exact', lambda k: k.startswith('seed0-')), ('noisy', lambda k: not k.startswith('seed0-'))):
        out(f'===== {kind} =====')
        for name, runs in data.items():
            runs = {k: v for k, v in runs.items() if sel(k)}
            if not runs: continue
            c = Counter(outcome(r) for r in runs.values())
            rec = c.get('recovered', 0)
            gaps = [r['minimumReplayClearanceMeters'] for r in runs.values() if r.get('minimumReplayClearanceMeters') is not None]
            frames = [f for r in runs.values() for f in r['frames']]
            ctl = [f['controllerSeconds'] for f in frames if f.get('controllerSeconds') is not None]
            horizons = [f['horizonSteps'] for f in frames if f.get('horizonSteps') is not None]
            first = [r['frames'][0]['horizonSteps'] for r in runs.values() if r['frames']]
            out(f'[{name}] runs {len(runs)} recovered {rec}  {dict(sorted(c.items()))}')
            if gaps: out(f'   least replay clearance {min(gaps):.3f} m; median per-run least {st.median(gaps):.3f} m')
            if ctl: out(f'   controller s median {st.median(ctl):.3f} p95 {quantile(ctl, .95):.3f} max {max(ctl):.2f}')
            if horizons: out(f'   horizon holds median {st.median(horizons):.0f} p95 {quantile(horizons, .95):.0f} max {max(horizons)}; first frame median {st.median(first):.0f}')
            ends = Counter(f.get('terminalEnd') or '' for f in frames)
            out(f'   terminal ends {dict(sorted(ends.items()))}')
            if True:
                stops = Counter(f.get('extensionStopped') or '' for f in frames)
                braking = sum(1 for f in frames if (f.get('appendedBrakingSteps') or 0) > 0)
                cand = sum(1 for f in frames if f.get('candidateFeasible') is False)
                out(f'   frames {len(frames)}: braking hold appended {braking}, extension stopped {dict(sorted(stops.items()))}, shifted candidate infeasible {cand}')
            per = Counter((scenario(k), outcome(r) == 'recovered') for k, r in runs.items())
            names = sorted({scenario(k) for k in runs})
            out('   recovered per scenario: ' + ', '.join(f'{s} {per[(s, True)]}/{per[(s, True)] + per[(s, False)]}' for s in names))
        keys = [k for k in data['signed'] if sel(k) and k in data['stop']]
        better = [k for k in keys if outcome(data['signed'][k]) != 'recovered' and outcome(data['stop'][k]) == 'recovered']
        worse = [(k, outcome(data['stop'][k])) for k in keys if outcome(data['signed'][k]) == 'recovered' and outcome(data['stop'][k]) != 'recovered']
        out(f'  stop vs signed: newly recovered {len(better)} {better}')
        out(f'  stop vs signed: newly failed {len(worse)} {worse}')
    text = '\n'.join(lines)
    print(text)
    os.makedirs(target, exist_ok=True)
    open(os.path.join(target, 'comparison.txt'), 'w').write(text + '\n')
    keys = sorted(set(data['signed']) | set(data['stop']))
    with open(os.path.join(target, 'campaign-runs.csv'), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['run'] + [f'{n}:{c}' for n in data for c in ('outcome', 'frames', 'minClearance', 'maxLateralDeviation', 'recoveryTime')])
        for k in keys:
            row = [k]
            for n in data:
                r = data[n].get(k, {})
                row += [outcome(r), r.get('executedFrames', ''), r.get('minimumReplayClearanceMeters', ''),
                        r.get('maximumReplayLateralDeviationMeters', ''), (r.get('recovery') or {}).get('confirmationTimeSeconds', '')]
            w.writerow(row)
    flines = ['Unrecovered encounters: outcome, frames executed, and over the last ten',
              'recorded frames the terminal ends, the extension stops and the stop time',
              'of the endpoint\'s continuation (s).']
    for n in data:
        flines.append(f'== {n}')
        for k in sorted(data[n]):
            r = data[n][k]
            if outcome(r) == 'recovered': continue
            last = r['frames'][-10:]
            ends = dict(Counter(f.get('terminalEnd') or '' for f in last))
            stops = dict(Counter(f.get('extensionStopped') or '' for f in last))
            stop = [f.get('terminalStopSeconds') for f in last if f.get('terminalStopSeconds') is not None]
            flines.append(f"{k:38s} {outcome(r):30s} frames {r.get('executedFrames', '-'):>5} ends {ends} extension {stops} stop {stop[-1] if stop else '-'}")
    open(os.path.join(target, 'failures.txt'), 'w').write('\n'.join(flines) + '\n')


if __name__ == '__main__':
    main()
