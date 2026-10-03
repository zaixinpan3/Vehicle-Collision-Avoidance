"""Behavior checks for the perception scenario's target speed reference."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from perceptionTargetTrajectory import TargetTrajectory, target_longitudinal_actuation


class PerceptionTargetTrajectoryTest(unittest.TestCase):
    def setUp(self):
        self.trajectory = TargetTrajectory(14.0, 1.0, 0.6, 0.8, 22.0)

    def test_target_speeds_up_when_ego_closes_the_gap(self):
        close = self.trajectory.target_speed(0.0, 11.0, 10.0)
        far = self.trajectory.target_speed(0.0, 11.0, 18.0)
        self.assertGreater(close, 11.0)
        self.assertLess(far, 11.0)

    def test_speed_matches_moving_reference_at_zero_gap_error(self):
        for time_s in (0.0, 1.0, 3.0, 7.0):
            gap, gap_rate = self.trajectory.reference(time_s)
            speed = self.trajectory.target_speed(time_s, 11.0, gap)
            self.assertAlmostEqual(speed - 11.0, gap_rate)
            dt = 1e-5
            numerical = (self.trajectory.reference(time_s + dt)[0] - gap) / dt
            self.assertAlmostEqual(numerical, gap_rate, places=5)

    def test_gap_converges_with_lagged_physical_speed_tracking(self):
        gap, target_speed = 18.0, 0.0
        minimum_gap, maximum_gap = gap, gap
        dt = 0.01
        for step in range(3000):
            time_s = step * dt
            ego_speed = min(11.0, 2.0 * time_s)
            speed_request = self.trajectory.target_speed(time_s, ego_speed, gap)
            acceleration = max(-4.0, min(3.0, (speed_request - target_speed) / 0.4))
            target_speed = max(0.0, target_speed + acceleration * dt)
            gap += (target_speed - ego_speed) * dt
            minimum_gap, maximum_gap = min(minimum_gap, gap), max(maximum_gap, gap)
        self.assertGreater(minimum_gap, 9.0)
        self.assertLess(maximum_gap, 24.0)
        self.assertLess(abs(gap - self.trajectory.reference(time_s)[0]), 0.3)

    def test_reference_never_requests_reverse_or_excessive_speed(self):
        self.assertEqual(self.trajectory.target_speed(0.0, 0.0, 100.0), 0.0)
        self.assertEqual(self.trajectory.target_speed(0.0, 30.0, 1.0), 22.0)

    def test_invalid_parameters_and_state_are_rejected(self):
        for values in ((14.0, 14.0, 0.6, 0.8, 22.0),
                       (14.0, 1.0, -0.6, 0.8, 22.0),
                       (14.0, 1.0, 0.6, 0.0, 22.0),
                       (math.nan, 1.0, 0.6, 0.8, 22.0)):
            with self.assertRaises(ValueError):
                TargetTrajectory(*values)
        for values in ((-1.0, 11.0, 14.0), (0.0, math.inf, 14.0),
                       (0.0, 11.0, math.nan)):
            with self.assertRaises(ValueError):
                self.trajectory.target_speed(*values)

    def test_target_actuation_releases_launch_limit_and_brakes_for_overspeed(self):
        self.assertEqual(target_longitudinal_actuation(11.0, 0.0), (1.0, 0.0))
        throttle, brake = target_longitudinal_actuation(11.0, 13.0)
        self.assertEqual(throttle, 0.0)
        self.assertGreater(brake, 0.0)
        self.assertEqual(target_longitudinal_actuation(0.0, 0.0), (0.0, 1.0))

    def test_scene_rejects_reference_outside_coverage_envelope(self):
        import json
        root = Path(__file__).resolve().parents[1]
        scene = json.loads((root / "config/carlaPerceptionScene.json").read_text())
        TargetTrajectory.from_scene(scene)
        scene["targetTrajectory"]["coverageCenterGapMinM"] = 14.0
        with self.assertRaises(ValueError):
            TargetTrajectory.from_scene(scene)


if __name__ == "__main__":
    unittest.main()
