"""Independently audit geometry in runNonlinearPredictiveSafetyValidation exports.

The named fixtures use 4.8 m by 1.9 m rectangles, an 8 m wide corridor,
and an oncoming target initially at (24, 0), heading pi, speed 8 m/s.
This is an offline sampled replay audit; it does not prove continuous safety. Only the Python standard library is used.
"""

import argparse
import json
import math
from pathlib import Path


def rectangle(x, y, heading):
    c, s = math.cos(heading), math.sin(heading)
    return [(x + c * a - s * b, y + s * a + c * b)
            for a, b in ((-2.4, -.95), (2.4, -.95), (2.4, .95), (-2.4, .95))]


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


def audit(result):
    name = result['scenario']
    if name not in ('oncoming', 'storedPolicy', 'recovery', 'solverFailure', 'circular', 'turningTarget'):
        raise ValueError(f'Unsupported fixture: {name}')
    minimum = math.inf
    road_margin = math.inf
    samples = 0
    for hold in result['trace']:
        for relative_time, state in zip(hold['auditTimes'], hold['auditStates']):
            samples += 1
            body = rectangle(*state[:3])
            if name == 'circular':
                # Positive 0.005 1/m curvature: center (0, 200), radius 200.
                lateral = [200 - math.hypot(x, y - 200) for x, y in body]
            else:
                lateral = [y for _, y in body]
            road_margin = min(road_margin, 4 - max(abs(y) for y in lateral))
            if name in ('oncoming', 'storedPolicy', 'turningTarget'):
                time = hold['time'] + relative_time
                if name == 'turningTarget':
                    yaw = math.pi - .8 * time
                    target = rectangle(24 - 10 * math.sin(yaw), 10 + 10 * math.cos(yaw), yaw)
                else:
                    target = rectangle(24 - 8 * time, 0, math.pi)
                minimum = min(minimum, distance(body, target))
    complete = result['completed'] and result['executedFrames'] == result['requestedFrames']
    reported = result['minimumReplayClearanceMeters']
    clearance_match = reported is None or abs(minimum - reported) < 1e-9
    zero_slack = all(hold.get('predictiveBarrierValue', 0) <= 1e-5 for hold in result['trace'])
    passed = (zero_slack and complete and samples > 0 and clearance_match and road_margin >= -1e-9
              and minimum >= result['requiredClearanceMeters'] - 1e-9)
    return dict(scenario=name, passed=passed, samples=samples,
                minimumClearanceMeters=minimum if math.isfinite(minimum) else None,
                minimumRoadMarginMeters=road_margin,
                agreesWithMatlabGeometry=clearance_match, allPlansZeroSlack=zero_slack,
                finalLateralErrorMeters=result['finalTransverseError'][0])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    entries = []
    for name in ('short-replays.json', 'stored-policy.json', 'oncoming.json'):
        data = json.loads((args.directory / name).read_text())['results']
        entries.extend(audit(result) for result in (data if isinstance(data, list) else [data]))
    args.output.write_text(json.dumps(dict(scope='independent sampled geometry', results=entries), indent=2) + '\n')
    for entry in entries:
        print(json.dumps(entry))
    if not all(entry['passed'] for entry in entries):
        raise SystemExit('Replay geometry audit failed.')


if __name__ == '__main__':
    main()
