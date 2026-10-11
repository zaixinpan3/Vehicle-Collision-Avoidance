"""Conic-solve statistics of campaign arms from the full traces (study script).
Usage: python3 -I solverstats.py <label>=<out-dir> [...]
For each arm, exact (seed 0) and noisy runs separately, over all frames: the
conic solves per frame, the interior-point iterations per solve, the solves
accepted as AlmostSolved (exit flag 2), the solver's share of the
controller time, the frames whose issued step is the full step (alpha = 1)
and the frames that linearized again in the step-size rule. Prints the table."""
import glob, json, os, statistics as st, sys


def quantile(a, p):
    a = sorted(a)
    if not a: return float('nan')
    x = p * (len(a) - 1); i = int(x); f = x - i
    return a[i] if i + 1 >= len(a) else a[i] + (a[i + 1] - a[i]) * f


def stages(entry):
    s = entry.get('solverStages') or []
    return [s] if isinstance(s, dict) else s


def main():
    for kind, exact in (('exact', True), ('noisy', False)):
        print(f'===== {kind} =====')
        for spec in sys.argv[1:]:
            label, out = spec.split('=', 1)
            solves = iters = almost = 0; per = []; solverShare = []; full = 0; again = 0; frames = 0; stageSeconds = []
            for f in glob.glob(os.path.join(out, 'seed*-speed*.json')):
                if f.endswith('-summary.json') or os.path.basename(f).startswith('seed0-') != exact: continue
                r = json.load(open(f))['results']; r = r[0] if isinstance(r, list) else r
                for e in r['trace']:
                    frames += 1
                    numeric = [s for s in stages(e) if s.get('numericalSolve')]
                    solves += len(numeric)
                    for s in numeric:
                        info = s.get('solverInfo') or {}
                        if info.get('iterations') is not None: per.append(info['iterations'])
                        if s.get('exitFlag') == 2: almost += 1
                        stageSeconds.append(s.get('seconds') or 0)
                    t = e.get('controllerSeconds') or 0
                    if t > 0: solverShare.append(sum(s.get('seconds') or 0 for s in numeric) / t)
                    full += e.get('acceptedStep') == 1
                    again += (e.get('acceptanceIterations') or 0) > 0
            if not frames: continue
            print(f'[{label}] frames {frames}: solves/frame {solves / frames:.2f}; iterations per solve median {st.median(per):.0f} '
                  f'p95 {quantile(per, .95):.0f}; AlmostSolved {almost}; solve ms median {1000 * st.median(stageSeconds):.2f}; '
                  f'solver share of controller time median {st.median(solverShare):.2f}; full step {full / frames:.3f}; '
                  f'linearized again {again}')


if __name__ == '__main__':
    main()
