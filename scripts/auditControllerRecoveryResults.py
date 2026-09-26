"""Audit the fixed 15-model/7-vehicle recovery campaign from saved MAT files.

Requires NumPy and SciPy. Checks finite states, completion, final continuous
2 s recovery sampled at control times, and vehicle SAT margins independently
of the MATLAB geometry implementation. Inputs are the unmodified existing
campaign: model 30 s at 8 m/s, vehicle 18 s at 10 m/s, tolerances .5 m/s,
.2 m, .02 rad. No collision-free continuation is inferred after a failed hold.
Generated audit/trace exports stay in the supplied external output directory.
"""
from pathlib import Path
import argparse
import csv
import json
import numpy as np
from scipy.io import loadmat

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("output_directory", type=Path, help="Directory produced by runControllerRecoveryValidation")
out = parser.parse_args().output_directory
exact_rows = json.loads((out / 'exact-summary.json').read_text())
assert len(exact_rows) == 15, "The exact-model campaign is incomplete."
exact_audit = []
for row in exact_rows:
    r = loadmat(out / (row['name'] + '.mat'), simplify_cells=True)['result']
    t, x = np.atleast_1d(r['time']), np.atleast_2d(r['state'])
    assert bool(r['completed']) == row['completed']
    assert abs(t[-1] - row['executedSeconds']) < 1e-9
    assert np.isfinite(x).all()
    mask = t >= 28 - 1e-10
    angle = x[2] + np.arctan2(x[4], x[3])
    course = np.arctan2(np.sin(angle), np.cos(angle))
    recovered = bool(r['completed'] and np.any(mask) and np.max(np.abs(x[3, mask] - 8)) <= .5
                     and np.max(np.abs(x[1, mask])) <= .2 and np.max(np.abs(course[mask])) <= .02)
    assert recovered == row['recoveredCruise']
    exact_audit.append(dict(name=row['name'], matReloadPassed=True,
                           finiteStates=True, recoveryIndependentlyMatches=True))
(out / 'independent-exact-audit.json').write_text(json.dumps(exact_audit, indent=2) + '\n')
print('15 exact-model files independently audited; all checks passed.')
rows = json.loads((out / 'vehicle-summary.json').read_text())
if isinstance(rows, dict):
    rows = [rows]
assert len(rows) == 7, 'The vehicle campaign is incomplete.'
audit = []
for row in rows:
    r = loadmat(out / (row['name'] + '.mat'), simplify_cells=True)['result']
    t = np.atleast_1d(r['controlTime'])
    x = np.atleast_2d(r['controlState'])
    assert np.isfinite(x).all()
    assert abs(t[-1] - row['executedSeconds']) < 1e-9
    complete = bool(r['metrics']['controllerCompletedScenario'] and not r['failure']['occurred'])
    assert complete == row['completed']
    lateral = np.atleast_1d(r['controlTracking']['lateralError'])
    angle = np.atleast_1d(r['controlTracking']['headingError']) + np.arctan2(x[:, 4], x[:, 3])
    course = np.arctan2(np.sin(angle), np.cos(angle))
    mask = t >= 16 - 1e-10
    recovered = bool(complete and np.any(mask) and np.max(np.abs(x[mask, 3] - 10)) <= .5
                     and np.max(np.abs(lateral[mask])) <= .2 and np.max(np.abs(course[mask])) <= .02)
    assert recovered == row['recoveredCruise']
    first_visible = np.flatnonzero(np.atleast_1d(r['attempts']['targetVisible']))
    first_time = float(np.atleast_1d(r['attempts']['time'])[first_visible[0]]) if len(first_visible) else None
    minimum_gap = None
    if 'avoidance' in r and np.size(r['plantTrace']['time']) > 0:
        trace, target = r['plantTrace'], r['targetTruth']
        ego_position = np.column_stack([trace['positionX'], trace['positionY']])
        delta = np.atleast_2d(target['position']) - ego_position
        ego_yaw, target_yaw = np.atleast_1d(trace['yaw']), np.atleast_1d(target['yaw'])
        ego_axes = np.stack([np.column_stack([np.cos(ego_yaw), np.sin(ego_yaw)]),
                             np.column_stack([-np.sin(ego_yaw), np.cos(ego_yaw)])], axis=1)
        target_axes = np.stack([np.column_stack([np.cos(target_yaw), np.sin(target_yaw)]),
                                np.column_stack([-np.sin(target_yaw), np.cos(target_yaw)])], axis=1)
        axes = np.concatenate([ego_axes, target_axes], axis=1)
        vehicle = r['controllerConfiguration']['vehicle']
        ego_half = .5 * np.array([vehicle['length'], vehicle['width']])
        target_half = .5 * np.array([target['length'], target['width']])
        ego_support = np.abs(np.einsum('nki,nji->nkj', axes, ego_axes)) @ ego_half
        target_support = np.abs(np.einsum('nki,nji->nkj', axes, target_axes)) @ target_half
        gaps = np.max(np.abs(np.einsum('ni,nki->nk', delta, axes)) - ego_support - target_support, axis=1)
        minimum_gap = float(np.min(gaps))
        assert abs(minimum_gap - row['minimumBodyGap']) < 1e-9
        assert bool(minimum_gap > 0) == row['collisionFree']
    audit.append(dict(name=row['name'], completed=complete, recoveredCruise=recovered,
                      firstVisibleTime=first_time, finalSpeed=float(x[-1, 3]),
                      maximumFullRunSpeedError=float(np.max(np.abs(x[:, 3] - 10))),
                      maximumFullRunLateralError=float(np.max(np.abs(lateral))),
                      independentlyComputedMinimumSATMargin=minimum_gap, allChecksPassed=True))
    np.savetxt(out / (row['name'] + '-trace.csv'), np.column_stack([t, x[:, 3], lateral, course]),
               delimiter=',', header='time_s,speed_mps,lateral_error_m,course_error_rad', comments='')
(out / 'independent-vehicle-audit.json').write_text(json.dumps(audit, indent=2) + '\n')
with (out / 'independent-vehicle-audit.csv').open('w') as f:
    writer = csv.DictWriter(f, fieldnames=list(audit[0]))
    writer.writeheader()
    writer.writerows(audit)
print(f'{len(audit)} saved vehicle results independently audited; all checks passed.')
