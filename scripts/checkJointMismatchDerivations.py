"""Exact-arithmetic checks of the joint mismatch and tube-shift derivations.

Run: python scripts/checkJointMismatchDerivations.py
These finite algebraic specializations use fractions, with no fitted slopes
or floating-point tolerance. They do not certify the full NRMM/Fiala domain.
No report or result file is created.
"""

from fractions import Fraction as F
from itertools import product
from math import ceil
import json


def matrixVector(matrix, vector):
    return tuple(sum((a * b for a, b in zip(row, vector)), F(0))
                 for row in matrix)


def addVectors(*vectors):
    return tuple(sum(values, F(0)) for values in zip(*vectors))


def scaleVector(scale, vector):
    return tuple(scale * value for value in vector)


def support(direction, generators):
    return sum((abs(sum((a * b for a, b in zip(direction, column)), F(0)))
                for column in generators), F(0))


def jointLinearIdentity():
    a, b, gain, observerGain, parameterMap = F(9, 10), F(1, 2), F(-1), F(1, 2), F(1, 5)
    matrix = ((a + b * gain, -b * gain, parameterMap),
              (F(0), a - observerGain, parameterMap),
              (F(0), F(0), F(1)))
    plantColumn, sensorColumn = (F(1), F(1), F(0)), (F(0), -observerGain, F(0))
    # Equality on a basis proves equality of these two linear maps in all
    # seven arguments for the declared rational coefficients.
    for index in range(7):
        values = [F(int(j == index)) for j in range(7)]
        x, estimate, nominal, nominalInput, theta, defect, noise = values
        measured = x + noise
        applied = nominalInput + gain * (estimate - nominal)
        nextTrue = a * x + b * applied + parameterMap * theta + defect
        nextEstimate = a * estimate + b * applied + observerGain * (measured - estimate)
        nextNominal = a * nominal + b * nominalInput
        expected = (nextTrue - nextNominal, nextTrue - nextEstimate, theta)
        propagated = addVectors(matrixVector(matrix, (x - nominal, x - estimate, theta)),
                                scaleVector(defect, plantColumn), scaleVector(noise, sensorColumn))
        assert propagated == expected
    parameterRadius = F(1, 20)
    shared = (scaleVector(parameterRadius, (parameterMap, parameterMap, F(0))),)
    independent = ((parameterMap * parameterRadius, F(0), F(0)),
                   (F(0), parameterMap * parameterRadius, F(0)))
    observerDeviation = (F(1), F(-1), F(0))
    sharedSupport = support(observerDeviation, shared)
    independentSupport = support(observerDeviation, independent)
    assert sharedSupport == 0 and independentSupport == 2 * parameterMap * parameterRadius
    return matrix, plantColumn, sensorColumn, sharedSupport, independentSupport


def finiteJointSupports(count=24):
    matrix, plantColumn, sensorColumn, _, _ = jointLinearIdentity()
    initial = ((F(1, 25), F(1, 25), F(0)), (F(0), F(0), F(1, 20)))
    disturbances = (scaleVector(F(1, 1000), plantColumn),
                    scaleVector(F(1, 100), sensorColumn))
    generators = list(initial)
    inputDirection = (F(-1), F(1), F(0))
    assert support(inputDirection, generators) == 0
    for _ in range(count):
        generators = [matrixVector(matrix, column) for column in generators] + list(disturbances)
    errorBlock = tuple(row[:2] for row in matrix[:2])
    parameterResponse = (F(0), F(0))
    for _ in range(count):
        parameterResponse = addVectors(matrixVector(errorBlock, parameterResponse),
                                       (matrix[0][2], matrix[1][2]))
    assert generators[1][:2] == scaleVector(initial[1][2], parameterResponse)
    direction = (F(1), F(0), F(0))
    signs = [F(1 if sum((a * b for a, b in zip(direction, column)), F(0)) >= 0 else -1)
             for column in generators]
    attained = addVectors(*(scaleVector(signs[j], initial[j]) for j in range(2)))
    fixedTheta = attained[2]
    for index in range(count):
        attained = addVectors(matrixVector(matrix, attained),
                              *(scaleVector(signs[2 + 2 * index + j], disturbances[j]) for j in range(2)))
        assert attained[2] == fixedTheta
    radius = support(direction, generators)
    assert attained[0] == radius
    return radius, support(inputDirection, generators)


def persistentParameterSupport():
    transition, radius, count = F(-1, 2), F(1, 20), 2
    fixed = abs(sum((transition**j for j in range(count)), F(0))) * radius
    fresh = sum((abs(transition**j) for j in range(count)), F(0)) * radius
    assert fixed == F(1, 40) and fresh == F(3, 40)
    return fixed, fresh


def nonlinearReserve():
    stateRadius, estimateRadius, parameterRadius, noiseRadius = F(1, 10), F(1, 25), F(1, 20), F(1, 1000)
    linearRadius = stateRadius / 2 + estimateRadius / 4 + parameterRadius / 5 + noiseRadius
    reserve = stateRadius * parameterRadius / 10
    # The residual x*theta/10 has exact absolute range bounded by reserve.
    # The full expression is multi-affine, so extrema on this box are at
    # vertices (maximize successively in each scalar argument).
    values = []
    for signs in product((-1, 1), repeat=4):
        x, epsilon, theta, noise = [s * r for s, r in
                                   zip(signs, (stateRadius, estimateRadius, parameterRadius, noiseRadius))]
        remainder = x * theta / 10
        assert abs(remainder) <= reserve
        values.append(x / 2 - epsilon / 4 + theta / 5 + noise + remainder)
    exactPeak = max(abs(value) for value in values)
    assert exactPeak > linearRadius and exactPeak <= linearRadius + reserve
    assert reserve == F(1, 2000)
    return linearRadius, reserve, exactPeak


def nonlinearImage(interval, parameters=(F(-1, 20), F(1, 20))):
    values = [x / 2 + theta + x * theta / 10
              for x, theta in product(interval, parameters)]
    return min(values), max(values)


def subset(first, second):
    return first[0] >= second[0] and first[1] <= second[1]


def carryCertificates():
    oldRoot = (F(-1, 5), F(1, 5))
    oldFirst = nonlinearImage(oldRoot)
    oldSecond = nonlinearImage(oldFirst)
    measurement = (F(1, 10), F(1, 5))
    assert not subset(measurement, oldFirst)
    posterior = (max(oldFirst[0], measurement[0]), min(oldFirst[1], measurement[1]))
    assert posterior[0] <= posterior[1] and subset(posterior, oldFirst)
    shiftedImage = nonlinearImage(posterior)
    assert subset(shiftedImage, oldSecond)
    # A valid but coarse fresh outer enclosure need not fit the old tube.
    freshOuter = (F(-1), F(1))
    assert subset(shiftedImage, freshOuter) and not subset(freshOuter, oldSecond)
    carried = (max(freshOuter[0], oldSecond[0]), min(freshOuter[1], oldSecond[1]))
    assert subset(shiftedImage, carried) and subset(carried, oldSecond)
    return posterior, carried


def brakeAdvance(position, speed, rate, duration):
    movingDuration = min(duration, speed / rate)
    return position + speed * movingDuration - rate * movingDuration**2 / 2, max(F(0), speed - rate * duration)


def commonBrakingCertificate():
    positions, speeds, rates = (F(-1, 5), F(3, 5)), (F(79, 10), F(17, 2)), (F(19, 5), F(21, 5))
    period, targetMinimum, bodySpan, clearance = F(1, 5), F(177, 10), F(24, 5), F(1, 5)
    stopBound = speeds[1] / rates[0]
    holdCount = ceil(stopBound / period)
    maximumStop = positions[1] + speeds[1]**2 / (2 * rates[0])
    robustMargin = targetMinimum - maximumStop - bodySpan - clearance
    assert robustMargin > 0
    nextMaximumPosition, nextMaximumSpeed = brakeAdvance(positions[1], speeds[1], rates[0], period)
    assert nextMaximumPosition + nextMaximumSpeed**2 / (2 * rates[0]) == maximumStop
    for initialPosition, initialSpeed, rate in product(positions, speeds, rates):
        stopPosition = initialPosition + initialSpeed**2 / (2 * rate)
        assert stopPosition <= maximumStop
        position, speed = initialPosition, initialSpeed
        for _ in range(holdCount):
            middlePosition, _ = brakeAdvance(position, speed, rate, period / 2)
            position, speed = brakeAdvance(position, speed, rate, period)
            assert targetMinimum - middlePosition - bodySpan >= clearance
            assert targetMinimum - position - bodySpan >= clearance
            # The same remaining stopping pose survives every shifted hold.
            assert position + speed**2 / (2 * rate) == stopPosition
        assert speed == 0 and position == stopPosition
        assert brakeAdvance(position, speed, rate, period) == (position, F(0))
    return stopBound, holdCount, robustMargin


def runChecks():
    _, _, _, shared, independent = jointLinearIdentity()
    finiteRadius, inputRadius = finiteJointSupports()
    fixed, fresh = persistentParameterSupport()
    linearRadius, reserve, peak = nonlinearReserve()
    posterior, carried = carryCertificates()
    stopTime, stopHolds, stopMargin = commonBrakingCertificate()
    return {
        "scope": "exact rational algebra and nonlinear scalar specializations",
        "fullVehicleNonlinearCertificate": False,
        "sharedMismatchObserverDeviationSupport": str(shared),
        "decorrelatedMismatchObserverDeviationSupport": str(independent),
        "attainedFinitePhysicalSupport": str(finiteRadius),
        "finiteInputSupport": str(inputRadius),
        "fixedParameterSupport": str(fixed),
        "freshParameterSupport": str(fresh),
        "uncorrectedLinearRadius": str(linearRadius),
        "nonlinearReserve": str(reserve),
        "nonlinearExactPeak": str(peak),
        "conditionedRoot": [str(x) for x in posterior],
        "carriedSuccessor": [str(x) for x in carried],
        "commonStopTimeBoundSeconds": str(stopTime),
        "commonStopHoldBound": stopHolds,
        "robustStopMarginMeters": str(stopMargin),
    }


if __name__ == "__main__":
    print(json.dumps(runChecks(), indent=2))
