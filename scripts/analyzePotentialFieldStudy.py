"""Audit sampled geometry and summarize paired initialization experiments."""
import argparse
import hashlib
import json
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('repository', type=Path)
    parser.add_argument('experiment', type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.repository / 'scripts'))
    from auditJointPredictiveSafety import audit
    rows = []
    for version in ('baseline', 'current'):
        directory = args.experiment / (version + '-campaign')
        summary = json.loads((directory / 'summary.json').read_text())
        for result in summary:
            stem = f"speed{result['speed']}-{result['scenario']}"
            source = directory / (stem + '.json')
            record = json.loads(source.read_text())['results']
            eight = geometry_record(record, audit)
            extended = directory / (stem + '-recovery.json')
            if extended.exists():
                source = extended
                record = json.loads(source.read_text())['results']
            final = geometry_record(record, audit)
            if record['trace']:
                worst = max(record['trace'], key=lambda t: t['controllerSeconds'])
                attempts = worst['attempts']
                if isinstance(attempts, dict):
                    attempts = [attempts]
                final['worstFrame'] = dict(
                    time=worst['time'], seconds=worst['controllerSeconds'],
                    initializationSeconds=sum(a['initializationSeconds'] for a in attempts),
                    formulationSeconds=sum(a['formulationSeconds'] for a in attempts),
                    solverSeconds=sum(s['seconds'] for s in worst['solverStages']),
                    solverCalls=worst['solverCalls'])
            rows.append(dict(version=version, speed=result['speed'], scenario=result['scenario'],
                             eightSeconds=eight, final=final, source=str(source),
                             sourceSha256=hashlib.sha256(source.read_bytes()).hexdigest()))
    output = dict(scope='Independent sampled geometry and nominal-recovery audit; no nonlinear feasibility certification',
                  physicalCriterion='Strictly positive rectangle distance; the 0.10 m buffer is an affine-node requirement',
                  results=rows)
    (args.experiment / 'comparison.json').write_text(json.dumps(output, indent=2) + '\n')
    for version in ('baseline', 'current'):
        final = [r['final'] for r in rows if r['version'] == version]
        # Duration completion is observation bookkeeping, not a control goal.
        # Unconfirmed recovery after an interruption does not prove divergence.
        print(version, 'collision-free and recovered',
              sum(r['strictlyCollisionFree'] and r['sampledRecoveryConfirmed'] for r in final),
              'collision observed', sum(not r['strictlyCollisionFree'] for r in final),
              'interrupted', sum(bool(r['failure']) for r in final),
              'recovery unconfirmed', sum(not r['sampledRecoveryConfirmed'] for r in final))


def geometry_record(result, audit):
    checked = audit(result)
    # Historical aggregate admission criteria require online residuals that
    # this RTI controller deliberately does not compute. Retain the geometry
    # and recovery findings, without relabeling that aggregate as a pass.
    assert checked['agreesWithMatlabGeometry']
    trace = result['trace']
    later = [t['controllerSeconds'] for t in trace[1:]]
    return dict(completed=result['completed'], executedFrames=result['executedFrames'],
                failure=result['failure'], failureTime=result['failureTime'],
                failedFrameSeconds=result['failedFrameSeconds'],
                minimumClearanceMeters=checked['minimumClearanceMeters'],
                strictlyCollisionFree=checked['strictlyCollisionFree'],
                sampledRecoveryConfirmed=checked['sampledRecoveryConfirmed'],
                recovery=result['recovery'], finalTransverseError=result['finalTransverseError'],
                samples=checked['samples'], agreesWithMatlabGeometry=checked['agreesWithMatlabGeometry'],
                allPredictionsZeroSlack=checked['allPredictionsZeroSlack'],
                everyHoldConverged=checked['everyHoldConverged'],
                firstFrameSeconds=result['firstFrameSeconds'],
                maximumFrameSeconds=result['maximumFrameSeconds'],
                maximumRunningSeconds=max(later, default=None),
                runningDeadlineMisses100ms=sum(t > .1 for t in later))


if __name__ == '__main__':
    main()
