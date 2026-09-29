"""Independently audit geometry in runNonlinearPredictiveSafetyValidation exports.

The target follows constant tangential acceleration and constant sideslip.
Its independent initial state and parameters are read from each export.
This is an offline sampled replay audit; it does not prove continuous safety. Only the Python standard library is used.
"""

import argparse
import json
import math
from pathlib import Path


def rectangle(x, y, heading, shape=(2.4, .95, 0, 0)):
    c, s = math.cos(heading), math.sin(heading)
    length, width, offset_x, offset_y = shape
    x, y = x + c * offset_x - s * offset_y, y + s * offset_x + c * offset_y
    return [(x + c * a - s * b, y + s * a + c * b)
            for a, b in ((-length, -width), (length, -width), (length, width), (-length, width))]


def target_state(initial, time):
    """Closed-form single-track motion, including signed velocity after braking."""
    x, y, heading, velocity, acceleration, beta, rear_axle = initial[:7]
    arc = velocity * time + .5 * acceleration * time * time
    curvature = math.sin(beta) / rear_axle
    angle = curvature * arc / 2
    travel = arc * (math.sin(angle) / angle if angle else 1)
    course = heading + beta + angle
    return (x + travel * math.cos(course), y + travel * math.sin(course),
            heading + curvature * arc, velocity + acceleration * time)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def edge_distance(p, a, b):
    v = (b[0] - a[0], b[1] - a[1])
    w = (p[0] - a[0], p[1] - a[1])
    fraction = max(0, min(1, dot(v, w) / dot(v, v)))
    return math.hypot(w[0] - fraction * v[0], w[1] - fraction * v[1])


def distance(a, b):
    separated = False
    for polygon in (a, b):
        for i, p in enumerate(polygon):
            q = polygon[(i + 1) % 4]
            normal = (p[1] - q[1], q[0] - p[0])
            pa, pb = [dot(normal, v) for v in a], [dot(normal, v) for v in b]
            separated |= min(pa) > max(pb) or min(pb) > max(pa)
    if not separated:
        return 0.0
    return min(edge_distance(p, other[i], other[(i + 1) % 4])
               for vertices, other in ((a, b), (b, a))
               for p in vertices for i in range(4))


def clearance_passes(minimum, margin):
    """Require strict separation, plus any explicitly configured buffer."""
    return minimum > 0 and minimum >= margin - 1e-9


def feasible_hold(hold):
    """A retained witness is executable without a new optimization result."""
    return (hold.get('hardResidual') == 0
            and hold.get('source') in ('sequentialConvexification',
                                       'retainedContinuation', 'feasibleInitialization'))


def audit(result):
    name = result['scenario']
    if name not in ('oncoming', 'recovery', 'circular', 'turningTarget',
                    'acceleratingTarget', 'acceleratingTurn', 'brakingTarget'):
        raise ValueError(f'Unsupported fixture: {name}')
    initial = result['targetInitialState']
    vehicle = result['configuration']['vehicle']
    ego_shape = (vehicle['length'] / 2, vehicle['width'] / 2, *vehicle['rectangleOffset'])
    minimum = math.inf
    road_margin = math.inf
    samples = 0
    for hold in result['trace']:
        for relative_time, state in zip(hold['auditTimes'], hold['auditStates']):
            samples += 1
            body = rectangle(*state[:3], shape=ego_shape)
            if name == 'circular':
                # Positive 0.005 1/m curvature: center (0, 200), radius 200.
                lateral = [200 - math.hypot(x, y - 200) for x, y in body]
            else:
                lateral = [y for _, y in body]
            road_margin = min(road_margin, 4 - max(abs(y) for y in lateral))
            if initial:
                time = hold['time'] + relative_time
                target = rectangle(*target_state(initial, time)[:3], shape=initial[7:11])
                minimum = min(minimum, distance(body, target))
    complete = result['completed'] and result['executedFrames'] == result['requestedFrames']
    reported = result['minimumReplayClearanceMeters']
    clearance_match = samples > 0 and (reported is None or abs(minimum - reported) < 1e-9)
    zero_slack = bool(result['trace']) and all(hold.get('predictiveBarrierValue') == 0 for hold in result['trace'])
    optimized = bool(result['trace']) and all(hold['solverCalls'] > 0 for hold in result['trace'])
    converged = bool(result['trace']) and all(hold.get('scvxConverged', False) for hold in result['trace'])
    feasible = bool(result['trace']) and all(feasible_hold(hold) for hold in result['trace'])
    collision_free = samples > 0 and minimum > 0
    clearance_satisfied = samples > 0 and clearance_passes(minimum, result['requiredClearanceMeters'])
    passed = (feasible and zero_slack and complete and samples > 0 and clearance_match and road_margin >= -1e-9
              and clearance_satisfied)
    return dict(scenario=name, passed=passed, samples=samples,
                minimumClearanceMeters=minimum if math.isfinite(minimum) else None,
                minimumRoadMarginMeters=road_margin if math.isfinite(road_margin) else None,
                agreesWithMatlabGeometry=clearance_match, allPredictionsZeroSlack=zero_slack,
                everyHoldFeasible=feasible, everyHoldInvokedOptimizer=optimized,
                everyHoldConverged=converged,
                strictlyCollisionFree=collision_free, configuredClearanceSatisfied=clearance_satisfied,
                finalLateralErrorMeters=(result['finalTransverseError'][0] if result['finalTransverseError'] else None))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--files', nargs='+', default=['short-replays.json', 'oncoming.json'])
    args = parser.parse_args()
    entries = []
    for name in args.files:
        data = json.loads((args.directory / name).read_text())['results']
        entries.extend(audit(result) for result in (data if isinstance(data, list) else [data]))
    args.output.write_text(json.dumps(dict(scope='independent sampled geometry', results=entries), indent=2) + '\n')
    for entry in entries:
        print(json.dumps(entry))
    if not all(entry['passed'] for entry in entries):
        raise SystemExit('Replay geometry audit failed.')


if __name__ == '__main__':
    main()
