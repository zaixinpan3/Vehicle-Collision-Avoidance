"""Width of the obstacle forecast family under the published parameter sets.

Evaluates the position radius of the constant-parameter enclosure
(manuscript eq. target_tube; controller/predictiveSafetyGeometry.m,
targetErrorTube) for an obstacle at 8 m/s on a straight course, with the
median published radii of the estimated-state campaign (course 0.20 rad,
speed 1.4 m/s, position 0.064 m; report MANUSCRIPT_ALGORITHM_SYNC_20261011)
and the declared bounds of the closed-loop target contract on the constants
(scripts/collisionThreatContract.m over the seven scenarios: |A| <= 1.1 m/s^2,
|beta| <= 0.055 rad, l_r = 1.6 m, so |kappa| <= sin(0.055)/1.6). No
simulation is run; the script only evaluates the closed-form enclosure.
"""
import math

V = 8.0            # obstacle speed, m/s (nominal A = 0, straight course)
b_p = 0.064        # published relative-position radius, m (median)
b_gamma = 0.20     # published course radius, rad (smallest median)
b_V = 1.4          # published speed radius, m/s (smallest median)
b_A = 1.1          # declared |A_C| bound of the closed-loop contract, m/s^2
b_kappa = math.sin(0.055) / 1.6   # declared |kappa_C| bound, 1/m
kappa_nominal = 0.0

print(f"b_kappa = {b_kappa:.4f} 1/m")
for tau in (1.0, 2.0, 3.0, 5.0):
    s = V * tau                                  # nominal arc length
    b_s = b_V * tau + 0.5 * b_A * tau**2         # arc-length error
    radius = b_p + b_s + abs(s) * b_gamma + 0.5 * s**2 * b_kappa
    course = min(math.pi, b_gamma + abs(s) * b_kappa + (abs(kappa_nominal) + b_kappa) * b_s)
    print(f"tau = {tau:.0f} s: arc {s:5.1f} m, b_s {b_s:5.2f} m, course term {abs(s)*b_gamma:5.2f} m, "
          f"curvature term {0.5*s**2*b_kappa:5.2f} m -> position radius {radius:5.1f} m, course radius {course:.2f} rad")
