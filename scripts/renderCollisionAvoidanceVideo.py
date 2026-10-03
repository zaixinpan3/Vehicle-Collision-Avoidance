"""Render fourteen recorded controller fixtures as a sequential annotated MP4.

Uses the exported ode45 states, actual held controls, and analytic target flow.
The full-scene overview shows recorded paths, not an online prediction. English
annotations, equal-scale vehicle geometry, and closest-approach pauses preserve
experiment meaning. Generated media belongs outside the versioned source tree.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from auditJointPredictiveSafety import distance, rectangle, target_state, recovery_completed

WIDTH, HEIGHT = 1600, 900
BACKGROUND = '#0A121F'
PANEL = '#101E2F'
BORDER = '#25384D'
TEXT = '#EDF4FB'
MUTED = '#94A9BE'
EGO = '#54DED5'
TARGET = '#FF927D'
GOLD = '#FFD28B'
MAP_BOX = (32, 253, 1132, 664)
SIDE_BOX = (1152, 253, 1568, 664)
NAMES = ['headOn', 'acceleratingHeadOn', 'brakingLead', 'crossing',
         'turningCrossing', 'curvedHeadOn', 'curvedCrossing']
TITLES = {'recovery': 'Lane recovery', 'circular': 'Curved-road tracking',
          'brakingTarget': 'Braking target beside the road',
          'turningTarget': 'Turning target', 'acceleratingTurn': 'Accelerating turn',
          'oncoming': 'Constant-speed head-on encounter',
          'acceleratingTarget': 'Accelerating head-on encounter',
          'headOn': 'Constant-speed head-on encounter',
          'acceleratingHeadOn': 'Accelerating head-on encounter',
          'brakingLead': 'Braking lead vehicle', 'crossing': 'Crossing vehicle',
          'turningCrossing': 'Accelerating turning crossing',
          'curvedHeadOn': 'Head-on encounter on a curved road',
          'curvedCrossing': 'Crossing encounter on a curved road'}

FONT = Path('/home/zai/.local/share/fonts/Inter/Inter.ttc')
FALLBACK_FONT = Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
FONTS = {}


def font(size):
    if size not in FONTS:
        FONTS[size] = ImageFont.truetype(str(FONT if FONT.exists() else FALLBACK_FONT), size)
    return FONTS[size]


def label(draw, point, text, size=18, color=TEXT, anchor=None):
    draw.text(point, text, font=font(size), fill=color, anchor=anchor)


def card(draw, box, fill=PANEL, radius=15):
    draw.rounded_rectangle(box, radius, fill=fill, outline=BORDER, width=1)


def format_gap(value):
    if value is None:
        return 'No target'
    if value < .1:
        return f'{value * 1000:.3f} mm'
    return f'{value:.3f} m'


def nearest_pair(a, b):
    """Nearest boundary pair for two strictly separated rectangles."""
    best = (math.inf, None, None)
    for vertices, other, reverse in [(a, b, False), (b, a, True)]:
        for vertex in vertices:
            p = np.array(vertex)
            for index, start in enumerate(other):
                start = np.array(start)
                v = np.array(other[(index + 1) % 4]) - start
                fraction = np.clip(np.dot(p - start, v) / np.dot(v, v), 0, 1)
                q = start + fraction * v
                gap = float(np.linalg.norm(p - q))
                if gap < best[0]:
                    best = (gap, q if reverse else p, p if reverse else q)
    return best


def prepare_scene(path, speed, index):
    """Read the actual ODE replay, without imposing an old solver contract."""
    source = json.loads(path.read_text())['results']
    assert source['completed'] and not source['failure'] and source['trace']
    trace = source['trace']
    times, states = [], []
    for hold in trace:
        for dt, state in zip(hold['auditTimes'], hold['auditStates']):
            time = hold['time'] + dt
            if times and abs(time - times[-1]) < 1e-10:
                continue
            times.append(time)
            states.append(state)
    times, states = np.asarray(times), np.asarray(states)
    assert np.all(np.diff(times) > 0)
    states[:, 2] = np.unwrap(states[:, 2])
    vehicle = source['configuration']['vehicle']
    shape = [vehicle['length'] / 2, vehicle['width'] / 2, *vehicle['rectangleOffset']]
    target = source['targetInitialState']
    targets = np.asarray([target_state(target, float(t)) for t in times])
    gaps = np.asarray([distance(rectangle(*x[:3], shape), rectangle(*q[:3], target[7:11]))
                       for x, q in zip(states, targets)])
    assert gaps.min() > 0
    assert abs(float(gaps.min()) - source['minimumReplayClearanceMeters']) < 1e-9
    assert recovery_completed(source), 'The recorded final nominal dwell is incomplete'
    closest = int(np.argmin(gaps))
    curve = source['baselineCruise']['referenceCurve']
    curvature = float(curve['curvature'])
    heading = float(curve['heading'])
    origin = np.asarray(curve['origin'])
    tangent, normal = np.array([math.cos(heading), math.sin(heading)]), np.array([-math.sin(heading), math.cos(heading)])
    if curvature:
        center = origin + normal / curvature
        radial = states[:, :2] - center
        phase = np.unwrap(np.arctan2(curvature * (radial @ tangent), -curvature * (radial @ normal)))
        station = phase / curvature
        lateral = (1 - np.linalg.norm(radial, axis=1) * abs(curvature)) / curvature
    else:
        station = (states[:, :2] - origin) @ tangent
        lateral = (states[:, :2] - origin) @ normal
    inputs = np.asarray([h['input'] for h in trace])
    node_times = np.asarray([h['time'] for h in trace])
    baseline = source['baselineCruise']
    baseline_times = np.asarray(baseline['times'])
    baseline_poses = np.asarray(baseline['egoPoses'])
    baseline_poses[:, 2] = np.unwrap(baseline_poses[:, 2])
    errors = np.asarray([h['transverseError'] for h in trace])
    normalized_error = np.max(np.abs(errors) / np.asarray(source['recovery']['tolerances']), axis=1)
    # Keep long outgoing target rays from shrinking the ego overview to a dot.
    near = np.linalg.norm(targets[:, :2] - states[:, :2], axis=1) < 70
    bounds = np.concatenate([states[:, :2], targets[near, :2]])
    lower, upper = bounds.min(axis=0) - [7, 7], bounds.max(axis=0) + [7, 7]
    scene = dict(source=source, path=path, index=index, speed=speed, times=times,
                 states=states, target=target, targets=targets, gaps=gaps,
                 closest_time=float(times[closest]), shape=shape, lateral=lateral,
                 inputs=inputs, node_times=node_times, bounds=(lower, upper),
                 duration=float(times[-1]), name=source['scenario'], curved=bool(curvature),
                 curve=curve, station=station, baseline=baseline, baseline_only=False,
                 baseline_times=baseline_times, baseline_poses=baseline_poses,
                 error_times=node_times + source['sampleTimeSeconds'], normalized_error=normalized_error,
                 minimum_display=source['minimumReplayClearanceMeters'])
    scene['charts'] = make_charts(scene)
    scene['overview'] = overview_image(scene)
    return scene


def sample(scene, time):
    x = np.array([np.interp(time, scene['times'], scene['states'][:, j]) for j in range(6)])
    q = np.array(target_state(scene['target'], time)) if scene['target'] else None
    i = int(np.clip(np.searchsorted(scene['node_times'], time, side='right') - 1,
                    0, len(scene['inputs']) - 1))
    body = rectangle(*x[:3], scene['shape'])
    other = rectangle(*q[:3], scene['target'][7:11]) if q is not None else None
    gap = distance(body, other) if other is not None else None
    return x, q, scene['inputs'][i], body, other, gap


def road_points(scene, start=None, end=None):
    start = float(scene['station'].min()) - 90 if start is None else start
    end = float(scene['station'].max()) + 90 if end is None else end
    station = np.linspace(start, end, max(300, int(abs(end - start) * 2)))
    curve = scene['curve']
    heading, curvature = float(curve['heading']), float(curve['curvature'])
    tangent = np.array([math.cos(heading), math.sin(heading)])
    normal = np.array([-math.sin(heading), math.cos(heading)])
    if curvature:
        center = (np.asarray(curve['origin']) + np.sin(curvature * station)[:, None] / curvature * tangent
                  + (1 - np.cos(curvature * station))[:, None] / curvature * normal)
    else:
        center = np.asarray(curve['origin']) + station[:, None] * tangent
    return center


def draw_road(draw, scene, transform, station):
    center = road_points(scene, station - 70, station + 85)
    points = [transform(p) for p in center]
    draw.line(points, fill='#24374A', width=9)
    for i in range(0, len(points) - 3, 9):
        draw.line(points[i:i + 5], fill='#B4C6D7', width=2)


def vehicle(draw, pose, shape, transform, color, steering=0, detail=True):
    body = rectangle(*pose[:3], shape)
    draw.polygon([transform(p) for p in body], fill=color)
    if not detail:
        return
    draw.line([transform(p) for p in body + [body[0]]], fill='#EAF8FF', width=1)
    c, s = math.cos(pose[2]), math.sin(pose[2])

    def local(x, y):
        return transform((pose[0] + c * x - s * y, pose[1] + s * x + c * y))

    cabin = [(-.9, -.67), (1.2, -.67), (1.2, .67), (-.9, .67)]
    draw.polygon([local(*p) for p in cabin], fill='#173449')
    draw.line([local(.9, -.65), local(.9, .65)], fill='#B8E5EE', width=2)
    for y in [-.80, .80]:
        for axle in [-1.60, 1.40]:
            angle = steering if axle > 0 else 0
            dx, dy = .32 * math.cos(angle), .32 * math.sin(angle)
            draw.line([local(axle - dx, y - dy), local(axle + dx, y + dy)],
                      fill='#06111C', width=5)
    for y in [-.63, .63]:
        draw.line([local(2.28, y - .11), local(2.28, y + .11)], fill='#FFF1CE', width=3)


def make_charts(scene):
    charts = []
    entries = [('PATH OFFSET', scene['lateral'], 'm', EGO, False, 0.),
               ('EGO SPEED', np.hypot(scene['states'][:, 3], scene['states'][:, 4]),
                'm/s', EGO, False, scene['speed']),
               ('BODY CLEARANCE', scene['gaps'], 'm / log scale', GOLD, True, None)]
    for title, values, units, color, logarithmic, reference in entries:
        image = Image.new('RGB', (496, 154), PANEL)
        draw = ImageDraw.Draw(image)
        label(draw, (16, 10), title, 13, MUTED)
        label(draw, (479, 10), units, 13, MUTED, 'ra')
        vals = np.log10(np.maximum(values, 1e-6)) if logarithmic else values
        if logarithmic:
            low, high = min(-2., math.floor(vals.min())), math.ceil(vals.max())
        else:
            low, high = min(float(vals.min()), reference), max(float(vals.max()), reference)
            pad = max(.2, (high - low) * .12)
            low, high = low - pad, high + pad
        left, right, top, bottom = 53, 477, 38, 117
        def ordinate(value):
            return bottom - (value - low) / (high - low) * (bottom - top)
        for fraction in [0, .5, 1]:
            value = low + fraction * (high - low)
            y = ordinate(value)
            draw.line([(left, y), (right, y)], fill=BORDER)
            tick = f'{10 ** value:.1g}' if logarithmic else f'{value:.2g}'
            label(draw, (45, y - 7), tick, 12, MUTED, 'ra')
        if reference is not None:
            draw.line([(left, ordinate(reference)), (right, ordinate(reference))], fill='#748C9B')
        points = [(left + t / scene['duration'] * (right - left), ordinate(v))
                  for t, v in zip(scene['times'][::4], vals[::4])]
        draw.line(points, fill=color, width=2)
        label(draw, (left, 127), '0 s', 12, MUTED)
        label(draw, (right, 127), f"{scene['duration']:.1f} s", 12, MUTED, 'ra')
        charts.append(image)
    return charts


def camera(scene, time, width, height):
    station = float(np.interp(time, scene['times'], scene['station']))
    lateral = float(np.interp(time, scene['times'], scene['lateral']))
    curve = scene['curve']
    k, heading = curve['curvature'], curve['heading']
    direction = heading + k * station
    tangent, normal = np.array([math.cos(direction), math.sin(direction)]), np.array([-math.sin(direction), math.cos(direction)])
    if k:
        base = np.asarray(curve['origin']) + np.array([
            (math.sin(direction) - math.sin(heading)) / k,
            (math.cos(heading) - math.cos(direction)) / k])
    else:
        base = np.asarray(curve['origin']) + station * tangent
    center = base + 5 * tangent + .5 * lateral * normal
    span = max(54., (abs(lateral) + 12) * width / height)
    scale = width / span
    def transform(point):
        delta = np.asarray(point) - center
        return (width / 2 + np.dot(delta, tangent) * scale,
                height / 2 - np.dot(delta, normal) * scale)
    return transform, station, scale


def map_view(scene, time, closest=False):
    w, h = MAP_BOX[2] - MAP_BOX[0], MAP_BOX[3] - MAP_BOX[1]
    image = Image.new('RGB', (w, h), '#142336')
    draw = ImageDraw.Draw(image)
    x, q, control, body, other, gap = sample(scene, time)
    transform, station, scale = camera(scene, time, w, h)
    for px in np.arange(0, w, 5 * scale):
        draw.line([(px, 0), (px, h)], fill='#1B2C3D')
    for py in np.arange(h / 2 % (5 * scale), h, 5 * scale):
        draw.line([(0, py), (w, py)], fill='#1B2C3D')
    draw_road(draw, scene, transform, station)
    baseline_pose = np.asarray([np.interp(time, scene['baseline_times'], scene['baseline_poses'][:, j]) for j in range(3)])
    polygon = rectangle(*baseline_pose, scene['shape'])
    baseline_gap = distance(polygon, other)
    danger = baseline_gap <= 0
    color = '#F16D7C' if danger else '#687D94'
    for start, end in zip(polygon, polygon[1:] + [polygon[0]]):
        for fraction in np.arange(0, 1, .25):
            a = np.asarray(start) + fraction * (np.asarray(end) - start)
            b = np.asarray(start) + min(1, fraction + .14) * (np.asarray(end) - start)
            draw.line([transform(a), transform(b)], fill=color, width=2)
    if danger:
        draw.rounded_rectangle((18, 40, 335, 71), 8, fill='#382533')
        label(draw, (30, 49), 'GRAY GHOST: COLLISION WITHOUT AVOIDANCE', 11, '#F696A3')
    end = np.searchsorted(scene['times'], time, side='right')
    start = np.searchsorted(scene['times'], max(0, time - 3.5))
    if end - start > 1:
        draw.line([transform(p) for p in scene['states'][start:end:8, :2]], fill=EGO, width=3)
        draw.line([transform(p) for p in scene['targets'][start:end:8, :2]], fill=TARGET, width=3)
    vehicle(draw, x, scene['shape'], transform, EGO, control[0])
    ego_point = transform(x[:2])
    label(draw, (ego_point[0], ego_point[1] - 46), 'EGO', 15, EGO, 'ma')
    vehicle(draw, q, scene['target'][7:11], transform, TARGET)
    target_point = transform(q[:2])
    if 35 < target_point[0] < w - 35 and 40 < target_point[1] < h - 42:
        label(draw, (target_point[0], target_point[1] + 33), 'TARGET', 15, TARGET, 'ma')
    else:
        vector = np.asarray(target_point) - np.asarray([w / 2, h / 2])
        fraction = min((w / 2 - 80) / max(abs(vector[0]), 1e-6), (h / 2 - 65) / max(abs(vector[1]), 1e-6))
        pointer = np.asarray([w / 2, h / 2]) + fraction * vector
        angle = math.atan2(vector[1], vector[0])
        vertices = [pointer + np.array([math.cos(angle + a), math.sin(angle + a)]) * r
                    for a, r in [(0, 12), (2.4, 9), (-2.4, 9)]]
        draw.polygon([tuple(p) for p in vertices], fill=TARGET)
        label(draw, (pointer[0], pointer[1] + 16), f'Target / {gap:.0f} m gap', 12, TARGET, 'ma')
    if 0 < gap < 1:
        _, p, v = nearest_pair(body, other)
        draw.line([transform(p), transform(v)], fill=GOLD, width=1)
    label(draw, (18, 14), 'FOLLOW VIEW / EQUAL AXIS SCALE', 13, MUTED)
    label(draw, (w - 18, 14), 'Dashed centerline: given path', 13, MUTED, 'ra')
    if closest:
        draw.rounded_rectangle((w - 258, 41, w - 15, 71), 8, fill='#473724')
        label(draw, (w - 135, 49), 'CLOSEST RECORDED APPROACH', 12, GOLD, 'ma')
    draw.line([(22, h - 27), (22 + 5 * scale, h - 27)], fill=TEXT, width=2)
    label(draw, (22 + 2.5 * scale, h - 48), '5 m', 12, TEXT, 'ma')
    return image


def overview_image(scene):
    w = SIDE_BOX[2] - SIDE_BOX[0]
    image = Image.new('RGB', (w, 169), PANEL)
    draw = ImageDraw.Draw(image)
    lo, hi = scene['bounds']
    scale = min((w - 40) / (hi[0] - lo[0]), 120 / (hi[1] - lo[1]))
    center = (lo + hi) / 2
    def transform(p):
        return (w / 2 + (p[0] - center[0]) * scale, 100 - (p[1] - center[1]) * scale)
    draw.line([transform(p) for p in road_points(scene)], fill='#425365', width=1)
    draw.line([transform(p) for p in scene['targets'][::10, :2]], fill='#735249', width=2)
    draw.line([transform(p) for p in scene['states'][::10, :2]], fill='#2B797F', width=2)
    draw.rectangle((0, 0, w, 28), fill=PANEL)
    label(draw, (16, 10), 'FULL RECORDED PATHS', 13, MUTED)
    return image, center, scale


def nominal_status(scene, time):
    if time >= scene['duration'] - 1e-8:
        return 'NOMINAL CRUISE RESTORED', EGO
    error = np.interp(time, scene['error_times'], scene['normalized_error'])
    if error <= 1:
        return 'TRACKING NOMINAL CRUISE', EGO
    gap = float(np.interp(time, scene['times'], scene['gaps']))
    if gap < 15:
        return 'AVOIDING THE ENCOUNTER', GOLD
    return 'RETURNING TO GIVEN PATH', TEXT


def side_view(scene, time):
    w, h = SIDE_BOX[2] - SIDE_BOX[0], SIDE_BOX[3] - SIDE_BOX[1]
    image = Image.new('RGB', (w, h), PANEL)
    overview, center, scale = scene['overview']
    image.paste(overview, (0, 0))
    draw = ImageDraw.Draw(image)
    x, q, _, body, other, gap = sample(scene, time)
    for pose, color in [(x, EGO), (q, TARGET)]:
        p = (w / 2 + (pose[0] - center[0]) * scale, 100 - (pose[1] - center[1]) * scale)
        if 10 < p[0] < w - 10 and 31 < p[1] < 160:
            draw.ellipse((p[0] - 4, p[1] - 4, p[0] + 4, p[1] + 4), fill=color)
    draw.line([(16, 175), (w - 16, 175)], fill=BORDER)
    if gap < .8:
        label(draw, (16, 188), 'GAP DETAIL / EQUAL AXIS SCALE', 12, MUTED)
        _, a, b = nearest_pair(body, other)
        center = (a + b) / 2
        span = max(.34, gap * 2.5 + .15)
        scale = (w - 32) / span
        detail = Image.new('RGB', (w - 32, 135), '#152337')
        dd = ImageDraw.Draw(detail)
        def close(p):
            return (detail.width / 2 + (p[0] - center[0]) * scale,
                    detail.height / 2 - (p[1] - center[1]) * scale)
        for polygon, color in [(body, EGO), (other, TARGET)]:
            dd.polygon([close(p) for p in polygon], fill=color)
        dd.line([close(a), close(b)], fill=GOLD, width=1)
        image.paste(detail, (16, 213))
        label(draw, (16, 358), format_gap(gap), 26, GOLD)
        label(draw, (w - 16, 370), 'body-to-body distance', 12, MUTED, 'ra')
    else:
        status, color = nominal_status(scene, time)
        label(draw, (16, 191), status, 16, color)
        offset = np.interp(time, scene['times'], scene['lateral'])
        label(draw, (16, 237), 'PATH OFFSET', 12, MUTED)
        label(draw, (w - 16, 231), f'{offset:+.2f} m', 24, EGO, 'ra')
        label(draw, (16, 280), 'SPEED / CRUISE', 12, MUTED)
        label(draw, (w - 16, 278), f"{math.hypot(x[3], x[4]):.2f} / {scene['speed']} m/s", 20, TEXT, 'ra')
        label(draw, (16, 328), 'RECORDED MINIMUM GAP', 12, MUTED)
        label(draw, (16, 352), format_gap(scene['minimum_display']), 26, GOLD)
    return image


def render_frame(scene, time, rate=1, closest=False, intro=False):
    image = Image.new('RGB', (WIDTH, HEIGHT), BACKGROUND)
    draw = ImageDraw.Draw(image)
    label(draw, (32, 18), 'FLOW-INITIALIZED RTI / CLOSED-LOOP REPLAY', 14, MUTED)
    label(draw, (32, 46), TITLES[scene['name']], 33)
    label(draw, (32, 99), f"Cruise {scene['speed']} m/s / {scene['speed'] * 3.6:.1f} km/h   |   Avoidance through nominal recovery", 18, MUTED)
    label(draw, (1566, 44), f"{scene['index']:02d} / 14", 31, TEXT, 'ra')
    for x, color, text in [(1355, EGO, 'EGO'), (1466, TARGET, 'TARGET')]:
        draw.ellipse((x, 101, x + 8, 109), fill=color)
        label(draw, (x + 17, 95), text, 14, MUTED)
    for index in range(14):
        left = 32 + index * 110
        color = EGO if index == scene['index'] - 1 else '#38606A' if index < scene['index'] - 1 else BORDER
        draw.rounded_rectangle((left, 136, left + 99, 141), 2, fill=color)
    x, _, _, _, _, gap = sample(scene, time)
    i = int(np.clip(np.searchsorted(scene['node_times'], time, side='right') - 1, 0, len(scene['node_times']) - 1))
    milliseconds = scene['source']['trace'][i]['controllerSeconds'] * 1000
    values = [('EGO SPEED', f'{math.hypot(x[3], x[4]):.2f} m/s', EGO),
              ('BODY CLEARANCE', format_gap(gap), GOLD),
              ('CONTROLLER CALL' + (' / FIRST' if i == 0 else ''), f'{milliseconds:.1f} ms', TEXT),
              ('SIMULATION TIME', f"{time:.2f} / {scene['duration']:.2f} s", TEXT)]
    for index, (title, value, color) in enumerate(values):
        left = 32 + index * 388
        card(draw, (left, 159, left + 368, 234))
        label(draw, (left + 16, 170), title, 12, MUTED)
        label(draw, (left + 16, 191), value, 26, color)
    image.paste(map_view(scene, time, closest), MAP_BOX[:2])
    image.paste(side_view(scene, time), SIDE_BOX[:2])
    draw = ImageDraw.Draw(image)
    for box in [MAP_BOX, SIDE_BOX]:
        draw.rounded_rectangle(box, 12, outline=BORDER, width=1)
    if intro:
        card(draw, (56, 555, 844, 638), fill='#152A3D')
        label(draw, (76, 566), f"SCENE {scene['index']:02d} / {scene['speed']} m/s", 23)
        label(draw, (76, 602), 'Cruise without avoidance collides. Watch the actual ego avoid and return.', 16, MUTED)
    for index, chart in enumerate(scene['charts']):
        left = 32 + 520 * index
        image.paste(chart, (left, 686))
        draw = ImageDraw.Draw(image)
        cursor = left + 53 + time / scene['duration'] * 424
        draw.line([(cursor, 724), (cursor, 803)], fill='#D2E1EC', width=1)
        draw.rounded_rectangle((left, 686, left + 496, 840), 12, outline=BORDER, width=1)
    playback = 'Closest approach / paused' if closest else f'{rate:g}x replay'
    label(draw, (32, 860), 'Recorded ODE motion / no road boundary constraints / gray ghost: cruise without avoidance', 14, MUTED)
    label(draw, (1568, 860), playback, 14, GOLD, 'ra')
    return image


def playback_rate(scene, time):
    # Slow every encounter, including a second meeting on a continuing circle.
    gap = np.interp(time, scene['times'], scene['gaps'])
    if abs(time - scene['closest_time']) < .55 or gap < 2:
        return .5
    if time < 3.5 or gap < 20:
        return 1.
    error = np.interp(time, scene['error_times'], scene['normalized_error'])
    if scene['duration'] > 25 and error <= 1 and gap > 40:
        return 8.
    return 2.


def scene_timeline(scene, fps):
    frames = [(0., 1., False, True)] * round(.8 * fps)
    time, paused = 0., False
    while time < scene['duration'] - 1e-10:
        rate = playback_rate(scene, time)
        frames.append((float(time), rate, False, False))
        next_time = min(scene['duration'], time + rate / fps)
        if not paused and time <= scene['closest_time'] < next_time:
            frames += [(scene['closest_time'], .5, True, False)] * round(.9 * fps)
            paused = True
        time = next_time
    frames += [(scene['duration'], 1., False, False)] * round(1.3 * fps)
    return frames


def cover(scenes, final=False):
    image = Image.new('RGB', (WIDTH, HEIGHT), BACKGROUND)
    draw = ImageDraw.Draw(image)
    label(draw, (44, 28), 'COLLISION AVOIDANCE / RECORDED NOMINAL EXPERIMENTS', 16, MUTED)
    label(draw, (44, 77), 'Fourteen encounters. Back to cruise.', 48)
    label(draw, (44, 148), '8 and 15 m/s / actual body geometry / slow motion at closest approach', 22, MUTED)
    for i, scene in enumerate([scenes[0], scenes[3], scenes[8], scenes[12]]):
        left, top = 44 + (i % 2) * 772, 219 + (i // 2) * 256
        frame = map_view(scene, scene['closest_time'])
        # Crop to the destination aspect ratio, never stretch vehicle geometry.
        crop_height = round(frame.width * 226 / 740)
        frame = frame.crop((0, (frame.height - crop_height) // 2,
                            frame.width, (frame.height + crop_height) // 2))
        image.paste(frame.resize((740, 226), Image.Resampling.LANCZOS), (left, top))
        draw = ImageDraw.Draw(image)
        draw.rounded_rectangle((left + 12, top + 13, left + 702, top + 46), 7, fill='#101E2F')
        label(draw, (left + 24, top + 20), f"{scene['speed']} m/s / {TITLES[scene['name']]}", 16)
    if final:
        minimum = min(s['minimum_display'] for s in scenes)
        maximum = max(h['controllerSeconds'] for s in scenes for h in s['source']['trace'][1:])
        headline = '14 / 14 avoided sampled collision and recovered nominal cruise'
        detail = f'Minimum recorded gap {minimum * 1000:.1f} mm / maximum running controller call {maximum * 1000:.1f} ms'
    else:
        headline = 'Cyan: ego / Coral: target / Gray outline: cruise without avoidance'
        detail = 'Each chapter includes the return to the given path and the desired cruise speed.'
    label(draw, (44, 750), headline, 25, EGO if final else TEXT)
    label(draw, (44, 800), detail, 19, MUTED)
    label(draw, (44, 853), 'Exact observations / no injected sensing or computation delay / playback speed is labeled', 14, MUTED)
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-directory', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--preview-only', action='store_true')
    parser.add_argument('--campaign', type=Path, help='Use baseline-certified scenario order from a campaign JSON')
    args = parser.parse_args()
    assert args.fps > 0
    args.output.parent.mkdir(parents=True, exist_ok=True)
    scenes = []
    cases = (json.loads(args.campaign.read_text())['results'] if args.campaign else
             [dict(referenceSpeedMetersPerSecond=speed, scenario=name) for speed in [8, 15] for name in NAMES])
    assert len(cases) == 14
    for case in cases:
        speed, name = case['referenceSpeedMetersPerSecond'], case['scenario']
        path = args.data_directory / f'speed{speed}-{name}.json'
        if not path.exists():
            path = args.data_directory / f'speed{speed}' / f'{name}.json'
        scene = prepare_scene(path, speed, len(scenes) + 1)
        if args.campaign:
            assert scene['baseline']['collisionDetected'] and scene['baseline']['initialClearanceMeters'] > 0
        scenes.append(scene)
        print(f"Prepared {scene['index']:02d}: {speed} m/s {name}; closest {scene['closest_time']:.6f} s", flush=True)
    manifest = dict(width=WIDTH, height=HEIGHT, fps=args.fps,
                    observations='Exact observations; no injected sensing or computation latency',
                    trajectory='Recorded ode45 hold replay; interpolation at video frames',
                    overview='Full recorded paths, not online predictions',
                    chapters=[])
    for scene in scenes:
        manifest['chapters'].append(dict(index=scene['index'], speedMetersPerSecond=scene['speed'],
            scenario=scene['name'], source=str(scene['path']),
            sourceSha256=hashlib.sha256(scene['path'].read_bytes()).hexdigest(),
            replaySamples=len(scene['times']),
            finalTransverseError=scene['source']['finalTransverseError'],
            simulationSeconds=scene['duration'], closestSimulationSeconds=scene['closest_time'],
            minimumBodyClearanceMeters=scene['source']['minimumReplayClearanceMeters'],
            independentGeometryAgrees=not scene['baseline_only'],
            completed=scene['source']['completed'], recovered=scene['source']['recovery']['recovered'],
            recoveryConfirmedSeconds=scene['source']['recovery']['confirmationTimeSeconds'],
            rolloutType='executedAvoidance',
            initializationFailure=scene['source']['failure'],
            baselineCollisionConfirmed=bool(scene['baseline'] and scene['baseline']['collisionDetected'])))
    for index in range(1, 15):
        scene = scenes[index - 1]
        render_frame(scene, scene['closest_time'], closest=bool(scene['target'])).save(
            args.output.parent / f'preview-scene{index:02d}.png')
    cover(scenes).save(args.output.parent / 'poster.png')
    if args.preview_only:
        (args.output.parent / 'preview-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        return
    temporary = args.output.with_name(args.output.stem + '-unindexed.mp4')
    command = ['ffmpeg', '-y', '-hide_banner', '-loglevel', 'warning', '-f', 'rawvideo',
               '-pix_fmt', 'rgb24', '-s', f'{WIDTH}x{HEIGHT}', '-r', str(args.fps),
               '-i', '-', '-an', '-c:v', 'libx264', '-preset', 'fast', '-crf', '19',
               '-threads', '4', '-pix_fmt', 'yuv420p', str(temporary)]
    frame_count = 0
    with (args.output.parent / 'ffmpeg-render.log').open('wb') as log:
        encoder = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=log)

        def write(frame):
            nonlocal frame_count
            encoder.stdin.write(frame.tobytes())
            frame_count += 1

        try:
            opening = cover(scenes)
            for _ in range(round(2.2 * args.fps)):
                write(opening)
            for scene, record in zip(scenes, manifest['chapters']):
                record['startVideoSeconds'] = frame_count / args.fps
                timeline = scene_timeline(scene, args.fps)
                last_key, last_frame = None, None
                for time, rate, closest, intro in timeline:
                    key = (time, rate, closest, intro)
                    if key != last_key:
                        last_frame = render_frame(scene, time, rate, closest, intro)
                        last_key = key
                    write(last_frame)
                for i in range(round(.2 * args.fps)):
                    write(Image.blend(last_frame, Image.new('RGB', last_frame.size, BACKGROUND),
                                      (i + 1) / round(.2 * args.fps)))
                record['endVideoSeconds'] = frame_count / args.fps
                record['videoFrames'] = len(timeline) + round(.2 * args.fps)
                print(f"Rendered scene {scene['index']:02d}/14; video {frame_count / args.fps:.2f} s", flush=True)
            ending = cover(scenes, final=True)
            for _ in range(round(3 * args.fps)):
                write(ending)
        finally:
            encoder.stdin.close()
        assert encoder.wait() == 0, 'ffmpeg encoding failed; see ffmpeg-render.log'
    metadata = [';FFMETADATA1', 'title=Collision avoidance: fourteen recorded scenarios',
                'comment=Exact observations. True geometry. Slow replay near closest approaches.']
    for entry in manifest['chapters']:
        metadata += ['[CHAPTER]', 'TIMEBASE=1/1000',
                     f"START={round(entry['startVideoSeconds'] * 1000)}",
                     f"END={round(entry['endVideoSeconds'] * 1000)}",
                     f"title={entry['index']:02d} - {entry['speedMetersPerSecond']} m/s - {TITLES[entry['scenario']]}" ]
    metadata_path = args.output.parent / 'chapters.ffmetadata'
    metadata_path.write_text('\n'.join(metadata) + '\n')
    subprocess.run(['ffmpeg', '-y', '-hide_banner', '-loglevel', 'error', '-i', str(temporary),
                    '-i', str(metadata_path), '-map_metadata', '1', '-map_chapters', '1',
                    '-codec', 'copy', '-movflags', '+faststart', str(args.output)], check=True)
    temporary.unlink()
    probe = json.loads(subprocess.check_output(['ffprobe', '-v', 'error', '-show_streams',
                         '-show_format', '-show_chapters', '-of', 'json', str(args.output)], text=True))
    stream = next(s for s in probe['streams'] if s['codec_type'] == 'video')
    assert stream['width'] == WIDTH and stream['height'] == HEIGHT
    assert int(stream['nb_frames']) == frame_count and len(probe['chapters']) == 14
    assert abs(float(probe['format']['duration']) - frame_count / args.fps) < .04
    manifest.update(video=str(args.output), totalFrames=frame_count, durationSeconds=frame_count / args.fps,
                    videoSha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
                    rendererSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    verification='Fourteen chapters; frame count, resolution, duration and geometry verified')
    (args.output.parent / 'render-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    (args.output.parent / 'ffprobe.json').write_text(json.dumps(probe, indent=2) + '\n')
    print(json.dumps({k:manifest[k] for k in ['video', 'totalFrames', 'durationSeconds', 'videoSha256']}), flush=True)


if __name__ == '__main__':
    main()
