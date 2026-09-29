"""Independently check cruise collisions and actual avoidance outcomes."""

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from auditJointPredictiveSafety import audit, rectangle, target_state, distance


def check_case(entry):
    path = Path(entry['source'])
    result = json.loads(path.read_text())['results']
    baseline = result['baselineCruise']
    curve = baseline['referenceCurve']
    times = np.asarray(baseline['times'])
    station = baseline['initialStationMeters'] + entry['referenceSpeedMetersPerSecond'] * times
    heading = curve['heading'] + curve['curvature'] * station
    if curve['curvature']:
        x = curve['origin'][0] + (np.sin(heading) - math.sin(curve['heading'])) / curve['curvature']
        y = curve['origin'][1] + (math.cos(curve['heading']) - np.cos(heading)) / curve['curvature']
    else:
        x = curve['origin'][0] + math.cos(curve['heading']) * station
        y = curve['origin'][1] + math.sin(curve['heading']) * station
    expected = np.column_stack([x, y, heading])
    assert np.max(np.abs(expected - np.asarray(baseline['egoPoses']))) < 1e-10
    q = result['targetInitialState']
    target = np.array([target_state(q, float(t))[:3] for t in times])
    assert np.max(np.abs(target - np.asarray(baseline['targetPoses']))) < 1e-10
    vehicle = result['configuration']['vehicle']
    shape = [vehicle['length'] / 2, vehicle['width'] / 2, *vehicle['rectangleOffset']]
    penetration = []
    for ego, other in zip(expected, target):
        a, b = rectangle(*ego, shape), rectangle(*other, q[7:11])
        axes = [(math.cos(h), math.sin(h)) for h in (ego[2], ego[2]+math.pi/2, other[2], other[2]+math.pi/2)]
        gap = []
        for nx, ny in axes:
            pa, pb = [nx*px+ny*py for px, py in a], [nx*px+ny*py for px, py in b]
            gap.append(max(min(pa)-max(pb), min(pb)-max(pa)))
        penetration.append(max(0, -max(gap)))
    collision = np.asarray(penetration) > 1e-9
    first = float(times[np.flatnonzero(collision)[0]]) if np.any(collision) else None
    initial = distance(rectangle(*expected[0], shape), rectangle(*target[0], q[7:11]))
    assert np.any(collision) and initial > 0
    assert int(np.sum(collision)) == baseline['strictOverlapSamples']
    assert abs(first-baseline['firstCollisionSeconds']) < 1e-12
    checked = audit(result)
    avoided = bool(result['completed'] and checked['strictlyCollisionFree']
                   and checked['agreesWithMatlabGeometry'] and checked['everyHoldFeasible']
                   and checked['allPredictionsZeroSlack'] and checked['minimumRoadMarginMeters'] >= -1e-9)
    return dict(**entry, sourceSha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                independentBaselineCollision=True, baselineStrictOverlapSamples=int(np.sum(collision)),
                baselineInitialClearanceMeters=initial, baselineMaximumOverlapDepthMeters=max(penetration),
                collisionAvoided=avoided, failedFrameSeconds=result['failedFrameSeconds'],
                geometryAudit=checked)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('campaign', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    campaign = json.loads(args.campaign.read_text())
    entries = [check_case(entry) for entry in campaign['results']]
    successful = [e for e in entries if e['collisionAvoided']]
    report = dict(scope='Independent baseline admission and sampled avoidance audit; failures retained',
                  totalScenarios=len(entries), baselineCollisions=sum(e['independentBaselineCollision'] for e in entries),
                  avoided=len(successful), initializationFailures=sum(e['executedFrames'] == 0 for e in entries),
                  minimumCompletedClearanceMeters=min(e['minimumReplayClearanceMeters'] for e in successful),
                  maximumCompletedRunningSeconds=max(e['maximumRunningSeconds'] for e in successful),
                  results=entries)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'results'}))


if __name__ == '__main__':
    main()
