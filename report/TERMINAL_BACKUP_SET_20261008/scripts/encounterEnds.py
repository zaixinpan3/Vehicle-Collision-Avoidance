"""How the accepted plans of a campaign output folder end their encounters.
Usage: python3 -I encounterEnds.py <outDir> [<outDir> ...]
Per folder: counts of terminalEnd over all frames (exact = seed0, noisy = the
rest), and for the frames ending by exit or by a cone/separation certificate
the time from the endpoint to the end (median, 99th percentile, maximum), and
the noisy braking leads' outcomes and hold counts."""
import glob, json, os, sys
def outcome(r):  # as compare.py
    if r.get('harnessError'): return 'harness'
    if r.get('collisionDetected'): return 'collision'
    if (r.get('recovery') or {}).get('recovered'): return 'recovered'
    f = r.get('failure') or ''
    if 'noOptimizationSolution' in f: return 'noSolution:' + f.split('(')[-1].rstrip(').')
    return 'other' if f else 'notRecovered'
def q(a, p):
    a = sorted(a); x = p * (len(a) - 1); i = int(x); f = x - i
    return a[i] if i + 1 >= len(a) else a[i] + (a[i + 1] - a[i]) * f
for out in sys.argv[1:]:
    ends = {'exact': [], 'noisy': []}; counts = {'exact': {}, 'noisy': {}}; leads = []
    for f in sorted(glob.glob(os.path.join(out, '*-summary.json'))):
        name = os.path.basename(f)[:-13]; g = 'exact' if name.startswith('seed0-') else 'noisy'
        r = json.load(open(f)); fr = r.get('frames') or []
        if isinstance(fr, dict): fr = [fr]
        if g == 'noisy' and 'brakingLead' in name:
            leads.append((name, outcome(r), r.get('executedFrames')))
        for x in fr:
            k = x.get('terminalEnd') or ''; counts[g][k] = counts[g].get(k, 0) + 1
            if k not in ('', 'noTarget') and x.get('terminalExitSeconds') is not None:
                ends[g].append(x['terminalExitSeconds'])
    print(f'[{out}]')
    for g in ('exact', 'noisy'):
        print(f'  {g}: {counts[g]}')
        v = ends[g]
        if v: print(f'  {g} end after endpoint (s): n {len(v)} median {q(v,.5):.3f} p99 {q(v,.99):.3f} max {max(v):.3f}')
    v = ends['exact'] + ends['noisy']
    if v: print(f'  all end after endpoint (s): n {len(v)} median {q(v,.5):.3f} p99 {q(v,.99):.3f} max {max(v):.3f}')
    for name, result, holds in leads: print(f'  {name:34s} {str(result):45s} holds {holds}')
