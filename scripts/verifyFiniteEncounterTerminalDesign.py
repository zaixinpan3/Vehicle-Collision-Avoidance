"""Exact arithmetic checks for the finite-encounter terminal design note.

These are scalar proof fixtures, not a vehicle simulation or a verification
of an implemented controller. Only Python standard-library modules are used.
"""

import argparse
from fractions import Fraction as F
import json
from pathlib import Path


def matmul(left, right):
    return [[sum(a * b for a, b in zip(row, column))
             for column in zip(*right)] for row in left]


def multiply(matrix, vector):
    return [sum(a * b for a, b in zip(row, vector)) for row in matrix]


def transition(time):
    return [[F(1), time, time**2 / 2], [F(0), F(1), time],
            [F(0), F(0), F(1)]]


def disturbance(time):
    jerk = F(383, 1000)
    return [jerk * time**3 / 6, jerk * time**2 / 2, jerk * time]


def verify():
    times = [F(0), F(1, 10), F(1, 3), F(8, 5)]
    for first in times:
        for second in times:
            assert matmul(transition(first), transition(second)) == transition(first + second)
            propagated = multiply(transition(first), disturbance(second))
            assert [a + b for a, b in zip(propagated, disturbance(first))] == disturbance(first + second)

    # State is integer separation. Reaching 5 ends this scalar encounter.
    # The post-exit physical motion is intentionally not represented here.
    states, controls, disturbances = range(1, 6), range(3), (-1, 0, 1)
    goal = 5
    terminalSets = [{goal}]
    policies = [{}]
    for remaining in range(1, 5):
        feasible, policy = {goal}, {}
        for state in states:
            if state == goal:
                continue
            choices = [control for control in controls if all(
                state + control + noise >= 1
                and min(goal, state + control + noise) in terminalSets[-1]
                for noise in disturbances)]
            if choices:
                feasible.add(state)
                policy[state] = choices
        assert feasible == set(range(max(1, goal - remaining), goal + 1))
        terminalSets.append(feasible)
        policies.append(policy)

    def enumeratePaths(state, remaining):
        if state == goal:
            return 1
        assert remaining > 0 and state in terminalSets[remaining]
        count = 0
        for control in policies[remaining][state]:
            for noise in disturbances:
                successor = state + control + noise
                assert successor >= 1
                count += enumeratePaths(min(goal, successor), remaining - 1)
        return count

    pathCount = sum(enumeratePaths(state, remaining)
                    for remaining in range(5) for state in terminalSets[remaining])

    # An incorrect quantifier swap would certify a nonexistent causal action.
    adaptiveAfterNoise = all(any(control + noise == 0 for control in (-1, 1))
                             for noise in (-1, 1))
    oneCausalControl = any(all(control + noise == 0 for noise in (-1, 1))
                          for control in (-1, 1))
    assert adaptiveAfterNoise and not oneCausalControl

    # Every two-step forecast reaches the goal, but resetting the completion
    # time while repeatedly applying the first zero input never reaches it.
    state = 0
    for _ in range(20):
        proposed = (0, 1)
        assert state + sum(proposed) >= 1
        state += proposed[0]
    assert state == 0

    # Safety over a short horizon alone does not provide a successor plan.
    # Fixed positive velocity, no control authority, wall at position 5/2.
    wall = F(5, 2)
    assert all(position < wall for position in (0, 1, 2))
    assert not all(position < wall for position in (1, 2, 3))

    # This trajectory obeys jerk -0.1 at all times. It exits range 11 by t=2,
    # then returns. Thus a correct finite exit is not a nonreturn certificate.
    def separation(time):
        return F(10) + time - time**3 / 60

    assert separation(F(2)) > 11
    assert separation(F(8)) < 11
    assert separation(F(10)) > 1 and separation(F(11)) < 1
    # On [0,2], derivative 1-t^2/20 >= 4/5, so it is continuously safe.
    assert 1 - F(2)**2 / 20 == F(4, 5)

    # An estimated center beyond range is not a robust exit test.
    center, radius, sensingRange = F(6), F(2), F(5)
    assert center > sensingRange and center - radius < sensingRange

    return {
        "scope": "Exact scalar proof fixtures; no vehicle or controller validation",
        "semigroupPairs": len(times)**2,
        "terminalSets": [sorted(values) for values in terminalSets],
        "allAdmissiblePolicyAndDisturbancePaths": pathCount,
        "wrongQuantifierWouldPass": adaptiveAfterNoise,
        "causalActionExistsInCounterexample": oneCausalControl,
        "resetDeadlineExampleSteps": 20,
        "resetDeadlineExampleFinalState": state,
        "shortHorizonIsNotRecursive": True,
        "exitDoesNotProveNonreturn": {
            "range": 11,
            "separationAt2": str(separation(F(2))),
            "separationAt8": str(separation(F(8))),
            "separationAt10": str(separation(F(10))),
            "separationAt11": str(separation(F(11))),
        },
        "nominalExitDoesNotProveRobustExit": True,
        "passed": True,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = json.dumps(verify(), indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(result, encoding="utf-8")
    print(result, end="")
