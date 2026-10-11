"""Check the analytic formulas in JOINT_TERMINAL_SET_DESIGN_20261010.tex.

These independent model and arithmetic checks do not run the production
controller or establish a robust nonlinear vehicle safety certificate.
Target state order: inertial X/Y [m], body yaw [rad], signed tangential
speed [m/s], constant tangential acceleration [m/s^2], sideslip [rad].

Run from the repository root:
    python scripts/checkJointTerminalSetDerivations.py --output \
        report/JOINT_TERMINAL_SET_DESIGN_20261010/derivation-checks.json
"""

import argparse
import json
from pathlib import Path

import numpy as np


def targetFlow(state, time, rearAxleDistance):
    nextState = state.copy()
    arc = state[3] * time + 0.5 * state[4] * time**2
    curvature = np.sin(state[5]) / rearAxleDistance
    halfAngle = 0.5 * curvature * arc
    chord = arc * np.sinc(halfAngle / np.pi)
    course = state[2] + state[5] + halfAngle
    nextState[:2] += chord * np.array([np.cos(course), np.sin(course)])
    nextState[2] += curvature * arc
    nextState[3] += state[4] * time
    return nextState


def targetField(state, rearAxleDistance):
    speed = state[3]
    course = state[2] + state[5]
    return np.array([
        speed * np.cos(course), speed * np.sin(course),
        speed * np.sin(state[5]) / rearAxleDistance,
        state[4], 0.0, 0.0,
    ])


def rotation(angle):
    return np.array([[np.cos(angle), -np.sin(angle)],
                     [np.sin(angle), np.cos(angle)]])


def egoPosition(position, yaw, bodyVelocity, yawRate, time):
    angle = yawRate * time
    diagonal = time * np.sinc(angle / np.pi)
    offDiagonal = 0.0 if yawRate == 0.0 else 2 * np.sin(angle / 2)**2 / yawRate
    integral = np.array([[diagonal, -offDiagonal], [offDiagonal, diagonal]])
    return position + rotation(yaw) @ integral @ bodyVelocity


def runChecks(outputPath=None):
    seed = 20261010
    rng = np.random.default_rng(seed)
    semigroupError = 0.0
    relativeFieldError = 0.0
    rk4Error = 0.0
    semigroupCases = 64
    rk4Cases = 14
    relativeCases = 64
    brakingCases = 100
    for index in range(semigroupCases):
        beta = [0.0, 1e-12, -1e-12, 0.25, -0.25][index % 5]
        state = np.array([*rng.uniform(-30, 30, 2), rng.uniform(-np.pi, np.pi),
                          rng.uniform(-15, 25), rng.uniform(-6, 6), beta])
        rearAxleDistance = rng.uniform(1.2, 2.5)
        firstTime, secondTime = rng.uniform(0, 5, 2)
        direct = targetFlow(state, firstTime + secondTime, rearAxleDistance)
        composed = targetFlow(targetFlow(state, firstTime, rearAxleDistance),
                              secondTime, rearAxleDistance)
        semigroupError = max(semigroupError, float(np.max(np.abs(direct - composed))))

    for index in range(rk4Cases):
        state = np.array([*rng.uniform(-10, 10, 2), rng.uniform(-np.pi, np.pi),
                          rng.uniform(2, 20), rng.uniform(-5, 3),
                          [0.0, 1e-12, -0.25, 0.25][index % 4]])
        rearAxleDistance = rng.uniform(1.2, 2.5)
        duration = 4.0
        stepCount = 4096
        step = duration / stepCount
        integrated = state.copy()
        for _ in range(stepCount):
            first = targetField(integrated, rearAxleDistance)
            second = targetField(integrated + step * first / 2, rearAxleDistance)
            third = targetField(integrated + step * second / 2, rearAxleDistance)
            fourth = targetField(integrated + step * third, rearAxleDistance)
            integrated += step * (first + 2 * second + 2 * third + fourth) / 6
        exact = targetFlow(state, duration, rearAxleDistance)
        rk4Error = max(rk4Error, float(np.max(np.abs(exact - integrated))))

    skew = np.array([[0.0, -1.0], [1.0, 0.0]])
    for index in range(relativeCases):
        target = np.array([*rng.uniform(-20, 20, 2), rng.uniform(-np.pi, np.pi),
                           rng.uniform(-10, 20), rng.uniform(-5, 3), rng.uniform(-0.4, 0.4)])
        position = rng.uniform(-20, 20, 2)
        yaw = rng.uniform(-np.pi, np.pi)
        bodyVelocity = np.array([rng.uniform(3, 20), rng.uniform(-1, 1)])
        yawRate = rng.uniform(-0.6, 0.6)
        rearAxleDistance = rng.uniform(1.2, 2.5)
        relativePosition = rotation(-yaw) @ (target[:2] - position)
        relativeYaw = target[2] - yaw
        analytic = np.concatenate([
            target[3] * np.array([np.cos(relativeYaw + target[5]),
                                 np.sin(relativeYaw + target[5])])
            - bodyVelocity - yawRate * skew @ relativePosition,
            [target[3] * np.sin(target[5]) / rearAxleDistance - yawRate],
        ])
        differenceStep = 1e-5
        values = []
        for time in [-differenceStep, differenceStep]:
            targetNext = targetFlow(target, time, rearAxleDistance)
            egoYaw = yaw + yawRate * time
            egoNext = egoPosition(position, yaw, bodyVelocity, yawRate, time)
            values.append(np.concatenate([
                rotation(-egoYaw) @ (targetNext[:2] - egoNext),
                [targetNext[2] - egoYaw],
            ]))
        finiteDifference = (values[1] - values[0]) / (2 * differenceStep)
        relativeFieldError = max(relativeFieldError,
                                 float(np.max(np.abs(finiteDifference - analytic))))

    brakingMarginError = 0.0
    unsafeRejections = 0
    for _ in range(brakingCases):
        closingSpeed = rng.uniform(0.01, 15)
        deceleration = rng.uniform(0.5, 8)
        initialMargin = rng.uniform(0, 2)
        initialGap = closingSpeed**2 / (2 * deceleration) + initialMargin
        times = np.linspace(0, 2 * closingSpeed / deceleration, 101)
        brakingTime = np.minimum(times, closingSpeed / deceleration)
        gap = initialGap - closingSpeed * brakingTime + 0.5 * deceleration * brakingTime**2
        residualSpeed = np.maximum(closingSpeed - deceleration * times, 0)
        margin = gap - residualSpeed**2 / (2 * deceleration)
        brakingMarginError = max(brakingMarginError,
                                float(np.max(np.abs(margin - initialMargin))))
        unsafeGap = closingSpeed**2 / (2 * deceleration) - 0.01
        matchTime = closingSpeed / deceleration
        unsafeMinimum = unsafeGap - closingSpeed * matchTime + 0.5 * deceleration * matchTime**2
        unsafeRejections += int(unsafeMinimum < 0)

    # Repository contract: signed target speed with constant A and beta.
    # Ego travels at 8 m/s; target starts 45 m ahead at 18 m/s with A=-2.
    rangeExit = (10 - np.sqrt(80)) / 2
    rangeReturn = (10 + np.sqrt(80)) / 2
    referencePointIntersection = (10 + np.sqrt(280)) / 2
    results = {
        'seed': seed,
        'target_semigroup': {'cases': semigroupCases, 'max_abs_error': semigroupError},
        'target_flow_vs_independent_rk4': {
            'cases': rk4Cases, 'steps_per_case': 4096, 'duration_seconds': 4.0,
            'max_abs_error': rk4Error,
        },
        'relative_vector_field': {'cases': relativeCases, 'max_abs_error': relativeFieldError},
        'scalar_braking_margin': {
            'cases': brakingCases, 'max_abs_error': brakingMarginError,
            'unsafe_cases_rejected': unsafeRejections,
        },
        'signed_contract_range_return_counterexample': {
            'ego_speed_mps': 8, 'target_initial_speed_mps': 18,
            'target_acceleration_mps2': -2, 'initial_gap_m': 45, 'range_m': 50,
            'exit_seconds': float(rangeExit), 'return_seconds': float(rangeReturn),
            'reference_point_intersection_seconds': float(referencePointIntersection),
        },
        'scope': 'Numerical checks of analytic derivations only; no full vehicle simulation, production change, or robust safety certificate.',
    }
    assert semigroupError < 1e-9
    assert rk4Error < 1e-7
    assert relativeFieldError < 1e-7
    assert brakingMarginError < 1e-10
    assert unsafeRejections == brakingCases
    assert rangeExit < rangeReturn < referencePointIntersection
    if outputPath is not None:
        outputPath.parent.mkdir(parents=True, exist_ok=True)
        outputPath.write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='Optional JSON results path.')
    runChecks(parser.parse_args().output)
