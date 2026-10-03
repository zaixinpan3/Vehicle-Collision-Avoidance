"""Generate a lane-following target speed reference for perception scenarios.

This scenario policy consumes vehicle state to position the simulated target.
Its inputs and outputs are not perception measurements or observer priors.
The adapter supplies a separate lane-following steering command and physical
speed controller; this module never changes a vehicle pose directly.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TargetTrajectory:
    """A smooth center-gap reference with relative-speed feedback (SI units)."""

    center_gap_m: float
    gap_amplitude_m: float
    gap_frequency_rad_s: float
    gap_gain_per_s: float
    max_speed_mps: float

    def __post_init__(self):
        values = (self.center_gap_m, self.gap_amplitude_m,
                  self.gap_frequency_rad_s, self.gap_gain_per_s,
                  self.max_speed_mps)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Target trajectory parameters must be finite.")
        if (self.center_gap_m <= self.gap_amplitude_m
                or self.gap_amplitude_m < 0.0
                or self.gap_frequency_rad_s < 0.0
                or self.gap_gain_per_s <= 0.0 or self.max_speed_mps <= 0.0):
            raise ValueError("Target trajectory requires positive gaps, gain and speed.")

    @classmethod
    def from_scene(cls, scene):
        settings = scene["targetTrajectory"]
        if settings["lateralOffsetM"] != 0.0:
            raise ValueError("The visibility scenario follows the lane center.")
        trajectory = cls(
            settings["centerGapM"], settings["gapAmplitudeM"],
            settings["gapFrequencyRadS"], settings["gapGainPerS"],
            settings["maxSpeedMps"])
        lower = settings["coverageCenterGapMinM"]
        upper = settings["coverageCenterGapMaxM"]
        initial = settings["initialGapM"]
        if not all(math.isfinite(value) for value in (lower, upper, initial)):
            raise ValueError("Scenario gap limits must be finite.")
        if not (0.0 < lower <= trajectory.center_gap_m - trajectory.gap_amplitude_m
                <= trajectory.center_gap_m + trajectory.gap_amplitude_m <= upper
                and lower <= initial <= upper):
            raise ValueError("Spawn and reference gaps must lie inside coverage limits.")
        return trajectory

    def reference(self, time_s):
        """Return desired center gap and its derivative in m and m/s."""
        if not math.isfinite(time_s) or time_s < 0.0:
            raise ValueError("Scenario time must be finite and nonnegative.")
        phase = self.gap_frequency_rad_s * time_s
        gap = self.center_gap_m + self.gap_amplitude_m * math.sin(phase)
        gap_rate = (self.gap_amplitude_m * self.gap_frequency_rad_s
                    * math.cos(phase))
        return gap, gap_rate

    def target_speed(self, time_s, ego_forward_speed_mps, center_gap_m):
        """Request a speed; actual visibility still requires sensor validation.

        With exact velocity tracking and no saturation, the gap error obeys
        e_dot = -gap_gain_per_s * e. Physical acceleration limits and turns
        prevent this reference law from being a hard visibility guarantee.
        """
        if not all(math.isfinite(value) for value in
                   (ego_forward_speed_mps, center_gap_m)):
            raise ValueError("Scenario vehicle state must be finite.")
        gap, gap_rate = self.reference(time_s)
        speed = (ego_forward_speed_mps + gap_rate
                 + self.gap_gain_per_s * (gap - center_gap_m))
        return min(self.max_speed_mps, max(0.0, speed))


def target_longitudinal_actuation(speed_reference_mps, measured_speed_mps):
    """Return normalized throttle/brake for the scenario target's speed servo.

    Retain the capture follower's proportional gains and throttle feedforward,
    but allow full target throttle to recover spacing after launch. The ego
    follower is unchanged. This actuator law is a scenario driver, not an
    avoidance controller or part of the estimator.
    """
    if not all(math.isfinite(value) and value >= 0.0 for value in
               (speed_reference_mps, measured_speed_mps)):
        raise ValueError("Speed reference and measured speed must be finite and nonnegative.")
    if speed_reference_mps == 0.0:
        return 0.0, 1.0
    error = speed_reference_mps - measured_speed_mps
    throttle = min(1.0, max(0.0, 0.18 * error + 0.25))
    brake = min(0.8, max(0.0, -0.25 * error))
    if brake > 0.05:
        throttle = 0.0
    return throttle, brake
