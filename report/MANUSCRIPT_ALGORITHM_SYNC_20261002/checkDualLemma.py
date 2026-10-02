#!/usr/bin/env python3
"""Numerical sanity check of the fixed-multiplier lemma stated in the manuscript.

For random pairs of rectangles it checks, with independent geometry:
(i)   min_j g_j(x; lambda) <= d for random admissible multipliers;
(ii)  the closed-form multipliers built from the closest-feature normal give
      H' lambda = n and min_j g_j = d for disjoint rectangles;
(iii) for overlapping rectangles every admissible nonzero multiplier gives a
      negative value;
and the first-order yaw remainder bound |v_j(psi+dpsi) - linearization| <=
|o_j| dpsi^2 / 2. This is a test of the statement, not a proof.

Usage: python3 checkDualLemma.py [samples]
"""
import math
import random
import sys

random.seed(20261002)
samples = int(sys.argv[1]) if len(sys.argv) > 1 else 20000


def rotation(angle):
    c, s = math.cos(angle), math.sin(angle)
    return ((c, -s), (s, c))


def vertices(pose, half):
    r = rotation(pose[2])
    return [(pose[0] + r[0][0] * a + r[0][1] * b, pose[1] + r[1][0] * a + r[1][1] * b)
            for a, b in ((-half[0], -half[1]), (half[0], -half[1]), (half[0], half[1]), (-half[0], half[1]))]


def closest_on_segment(p, a, b):
    vx, vy = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / (vx * vx + vy * vy)))
    return (a[0] + t * vx, a[1] + t * vy)


def separated(a, b):
    for polygon in (a, b):
        for i in range(4):
            p, q = polygon[i], polygon[(i + 1) % 4]
            normal = (p[1] - q[1], q[0] - p[0])
            pa = [normal[0] * v[0] + normal[1] * v[1] for v in a]
            pb = [normal[0] * v[0] + normal[1] * v[1] for v in b]
            if min(pa) > max(pb) or min(pb) > max(pa):
                return True
    return False


def closest_pair(ego, obstacle):
    """Closest points (ego point, obstacle point) of two disjoint convex quadrilaterals."""
    best = (math.inf, None, None)
    for i in range(4):
        for k in range(4):
            q = closest_on_segment(ego[i], obstacle[k], obstacle[(k + 1) % 4])
            d = math.dist(ego[i], q)
            if d < best[0]:
                best = (d, ego[i], q)
            q = closest_on_segment(obstacle[i], ego[k], ego[(k + 1) % 4])
            d = math.dist(obstacle[i], q)
            if d < best[0]:
                best = (d, q, obstacle[i])
    return best


def rows(multiplier, ego, pose_c, half_c):
    """g_j = lambda' [H (v_j - p_C) - h] with H = [I; -I] R(psi_C)'."""
    r = rotation(pose_c[2])
    values = []
    for v in ego:
        dx, dy = v[0] - pose_c[0], v[1] - pose_c[1]
        local = (r[0][0] * dx + r[1][0] * dy, r[0][1] * dx + r[1][1] * dy)
        residual = (local[0] - half_c[0], local[1] - half_c[1], -local[0] - half_c[0], -local[1] - half_c[1])
        values.append(sum(m * e for m, e in zip(multiplier, residual)))
    return min(values)


def admissible():
    multiplier = [random.random() for _ in range(4)]
    norm = math.hypot(multiplier[0] - multiplier[2], multiplier[1] - multiplier[3])
    scale = random.random() / max(norm, 1e-12)
    return [m * scale for m in multiplier]


half_e, half_c = (2.4, 0.95), (2.4, 0.95)
worst = {"weak": -math.inf, "strong": 0.0, "normal": 0.0, "overlap": -math.inf, "yaw": -math.inf}
disjoint = overlapping = 0
for _ in range(samples):
    pose_e = (random.uniform(-8, 8), random.uniform(-8, 8), random.uniform(-math.pi, math.pi))
    pose_c = (0.0, 0.0, random.uniform(-math.pi, math.pi))
    ego, obstacle = vertices(pose_e, half_e), vertices(pose_c, half_c)
    if separated(ego, obstacle):
        disjoint += 1
        d, z_e, z_c = closest_pair(ego, obstacle)
        worst["weak"] = max(worst["weak"], rows(admissible(), ego, pose_c, half_c) - d)
        n = ((z_e[0] - z_c[0]) / d, (z_e[1] - z_c[1]) / d)
        r = rotation(pose_c[2])
        w = (r[0][0] * n[0] + r[1][0] * n[1], r[0][1] * n[0] + r[1][1] * n[1])
        star = [max(w[0], 0), max(w[1], 0), max(-w[0], 0), max(-w[1], 0)]
        back = (r[0][0] * (star[0] - star[2]) + r[0][1] * (star[1] - star[3]),
                r[1][0] * (star[0] - star[2]) + r[1][1] * (star[1] - star[3]))
        worst["normal"] = max(worst["normal"], math.dist(back, n))
        worst["strong"] = max(worst["strong"], abs(rows(star, ego, pose_c, half_c) - d))
    else:
        overlapping += 1
        multiplier = admissible()
        if max(multiplier) > 1e-6:
            worst["overlap"] = max(worst["overlap"], rows(multiplier, ego, pose_c, half_c))
    # first-order vertex rotation remainder
    offset = (random.choice((-1, 1)) * half_e[0], random.choice((-1, 1)) * half_e[1])
    psi, dpsi = random.uniform(-math.pi, math.pi), random.uniform(-0.25, 0.25)
    r0, r1 = rotation(psi), rotation(psi + dpsi)
    exact = (r1[0][0] * offset[0] + r1[0][1] * offset[1], r1[1][0] * offset[0] + r1[1][1] * offset[1])
    base = (r0[0][0] * offset[0] + r0[0][1] * offset[1], r0[1][0] * offset[0] + r0[1][1] * offset[1])
    linear = (base[0] - base[1] * dpsi, base[1] + base[0] * dpsi)
    worst["yaw"] = max(worst["yaw"], math.dist(exact, linear) - 0.5 * math.hypot(*offset) * dpsi ** 2)

print(f"disjoint {disjoint}, overlapping {overlapping}")
print(f"(i)   max of min_j g_j - d over random multipliers: {worst['weak']:.3e} (must be <= 0)")
print(f"(ii)  max |H' lambda* - n|: {worst['normal']:.3e}; max |min_j g_j - d|: {worst['strong']:.3e}")
print(f"(iii) max value under overlap for nonzero multipliers: {worst['overlap']:.3e} (must be < 0)")
print(f"yaw remainder minus bound: {worst['yaw']:.3e} (must be <= 0)")
ok = (worst["weak"] <= 1e-12 and worst["normal"] < 1e-9 and worst["strong"] < 1e-9
      and worst["overlap"] < 0 and worst["yaw"] <= 1e-12)
print("PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
