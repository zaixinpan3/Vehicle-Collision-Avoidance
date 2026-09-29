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

from auditJointPredictiveSafety import distance, rectangle, target_state

WIDTH, HEIGHT = 1600, 900
BACKGROUND = '#0A121F'
PANEL = '#101E2F'
BORDER = '#25384D'
TEXT = '#EDF4FB'
MUTED = '#94A9BE'
EGO = '#54DED5'
TARGET = '#FF927D'
GOLD = '#FFD28B'
MAP_BOX = (32, 274, 1132, 664)
SIDE_BOX = (1152, 274, 1568, 664)
NAMES = ['recovery', 'circular', 'brakingTarget', 'turningTarget',
         'acceleratingTurn', 'oncoming', 'acceleratingTarget']
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
DESCRIPTIONS = {
    'recovery': ['A 10 mm initial lane offset.', 'Watch the lateral-error curve.'],
    'circular': ['A 200 m radius road.', 'Watch the heading and steering.'],
    'brakingTarget': ['The target brakes outside the corridor.',
                      'Its recorded model later reverses.'],
    'turningTarget': ['The target follows a curved path.',
                      'Watch when the ego can stay straight.'],
    'acceleratingTurn': ['The target accelerates while turning.',
                         'Both speed and heading change.'],
    'oncoming': ['The target approaches at 8 m/s.',
                 'Watch the ego move aside and return.'],
    'acceleratingTarget': ['The target accelerates at 1 m/s².',
                           'Watch the tightest passing distance.'],
    'headOn': ['Both vehicles approach on the same path.', 'Cruise without avoidance would collide.'],
    'acceleratingHeadOn': ['The oncoming target accelerates.', 'Cruise without avoidance would collide.'],
    'brakingLead': ['The lead vehicle brakes in the ego lane.', 'Cruise without avoidance would collide.'],
    'crossing': ['The target crosses the ego path.', 'Cruise without avoidance would collide.'],
    'turningCrossing': ['The target turns and accelerates into the path.', 'Cruise without avoidance would collide.'],
    'curvedHeadOn': ['Oncoming traffic follows the same curve.', 'Cruise without avoidance would collide.'],
    'curvedCrossing': ['The target crosses the curved ego path.', 'Cruise without avoidance would collide.'],
}
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
    source = json.loads(path.read_text())['results']
    baseline_only = not source['completed']
    if baseline_only:
        assert source.get('baselineCruise', {}).get('collisionDetected') and not source['trace']
    assert source['configuration']['collision']['safetyMarginMeters'] == .006
    trace = source['trace']
    times, states = [], []
    for hold in trace:
        assert hold['hardResidual'] == 0 and hold['predictiveBarrierValue'] == 0
        for dt, state in zip(hold['auditTimes'], hold['auditStates']):
            time = hold['time'] + dt
            if times and abs(time - times[-1]) < 1e-10:
                continue
            times.append(time)
            states.append(state)
    if baseline_only:
        times = source['baselineCruise']['times']
        poses = np.asarray(source['baselineCruise']['egoPoses'])
        states = np.column_stack([poses, np.full(len(times), speed), np.zeros((len(times), 2))])
    times = np.array(times)
    states = np.array(states)
    assert np.all(np.diff(times) > 0)
    states[:, 2] = np.unwrap(states[:, 2])
    vehicle = source['configuration']['vehicle']
    shape = [vehicle['length'] / 2, vehicle['width'] / 2, *vehicle['rectangleOffset']]
    target = source['targetInitialState']
    targets, gaps = None, None
    if target:
        targets = np.array([target_state(target, float(t)) for t in times])
        gaps = np.array([distance(rectangle(*x[:3], shape), rectangle(*q[:3], target[7:11]))
                         for x, q in zip(states, targets)])
        if not baseline_only:
            assert np.min(gaps) > 0
            assert abs(float(np.min(gaps)) - source['minimumReplayClearanceMeters']) < 1e-9
        closest = (int(np.searchsorted(times, source['baselineCruise']['firstCollisionSeconds']))
                   if baseline_only else int(np.argmin(gaps)))
    else:
        closest = len(times) - 1
    lateral = states[:, 1]
    curved = source['scenario'] == 'circular' or source.get('baselineCruise', {}).get('referenceCurve', {}).get('curvature') == .005
    if curved:
        lateral = 200 - np.hypot(states[:, 0], states[:, 1] - 200)
    inputs = np.array([h['input'] for h in trace])
    node_times = np.array([h['time'] for h in trace])
    if baseline_only:
        inputs = np.array([[np.nan, np.nan]])
        node_times = np.array([0.])
    bounds = states[:, :2]
    if targets is not None:
        bounds = np.concatenate([bounds, targets[:, :2]])
    lower, upper = bounds.min(axis=0) - [6, 5], bounds.max(axis=0) + [6, 5]
    scene = dict(source=source, path=path, index=index, speed=speed, times=times,
                 states=states, target=target, targets=targets, gaps=gaps,
                 closest_time=float(times[closest]), shape=shape, lateral=lateral,
                 inputs=inputs, node_times=node_times, bounds=(lower, upper),
                 duration=float(times[-1]), name=source['scenario'], curved=curved,
                 baseline=source.get('baselineCruise'), baseline_only=baseline_only,
                 minimum_display=float(np.min(gaps)) if baseline_only else source['minimumReplayClearanceMeters'])
    scene['charts'] = make_charts(scene)
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


def road_points(scene, start=-80, end=200):
    s = np.linspace(start, end, 600)
    if scene['curved']:
        k = .005
        center = np.column_stack([np.sin(k * s) / k, (1 - np.cos(k * s)) / k])
        normal = np.column_stack([-np.sin(k * s), np.cos(k * s)])
    else:
        center = np.column_stack([s, np.zeros_like(s)])
        normal = np.column_stack([np.zeros_like(s), np.ones_like(s)])
    return center, center + 4 * normal, center - 4 * normal


def draw_road(draw, scene, transform):
    center, upper, lower = road_points(scene)
    draw.polygon([transform(p) for p in np.concatenate([upper, lower[::-1]])], fill='#233345')
    for boundary in [upper, lower]:
        draw.line([transform(p) for p in boundary], fill='#BDCEDA', width=2)
    for i in range(0, len(center) - 5, 14):
        draw.line([transform(p) for p in center[i:i + 6]], fill='#8493A4', width=2)


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
    if scene['name'] in ['recovery', 'circular']:
        lateral, units = scene['lateral'] * 1000, 'mm'
    else:
        lateral, units = scene['lateral'], 'm'
    steering_times = np.column_stack([scene['node_times'],
        scene['node_times'] + scene['source']['sampleTimeSeconds']]).ravel()
    steering_values = np.repeat(np.degrees(scene['inputs'][:, 0]), 2)
    entries = [('LATERAL POSITION', scene['times'], lateral, units, EGO, False),
               ('STEERING', steering_times, steering_values, 'deg', EGO, False)]
    if scene['gaps'] is not None:
        entries.append(('BODY CLEARANCE', scene['times'], scene['gaps'], 'm · log scale', GOLD, True))
    else:
        entries.append(('EGO SPEED', scene['times'], scene['states'][:, 3], 'm/s', EGO, False))
    for title, times, values, units, color, logarithmic in entries:
        image = Image.new('RGB', (496, 154), PANEL)
        draw = ImageDraw.Draw(image)
        label(draw, (16, 10), title, 13, MUTED)
        label(draw, (479, 10), units, 13, MUTED, 'ra')
        if scene['baseline_only'] and title == 'STEERING':
            label(draw, (24, 53), 'NO EXECUTED CONTROL', 23, TARGET)
            label(draw, (24, 92), 'Initialization did not produce a feasible plan.', 15, MUTED)
            charts.append(image)
            continue
        if scene['baseline_only']:
            draw.rectangle((0, 0, 340, 30), fill=PANEL)
            label(draw, (16, 10), 'BASELINE ' + title, 13, MUTED)
        if logarithmic:
            vals = np.log10(np.maximum(values, 1e-6))
            low, high = min(-3, vals.min() - .15), math.ceil(vals.max())
        else:
            vals = values
            low, high = float(vals.min()), float(vals.max())
            pad = max(.1 if units == 'mm' else .005, (high - low) * .12)
            low, high = low - pad, high + pad
        low, high = float(low), float(high)
        left, right, top, bottom = 53, 477, 38, 117
        for fraction in [0, .5, 1]:
            y = bottom - fraction * (bottom - top)
            draw.line([(left, y), (right, y)], fill=BORDER)
            value = low + fraction * (high - low)
            tick = f'{10 ** value:.1g}' if logarithmic else f'{value:.2g}'
            label(draw, (45, y - 7), tick, 12, MUTED, 'ra')
        points = [(left + t / scene['duration'] * (right - left),
                   bottom - (v - low) / (high - low) * (bottom - top))
                  for t, v in zip(times, vals)]
        draw.line(points, fill=color, width=2)
        if logarithmic and not scene['baseline_only']:
            y = bottom - (math.log10(.006) - low) / (high - low) * (bottom - top)
            draw.line([(left, y), (right, y)], fill='#AB7855', width=1)
            label(draw, (right - 3, y - 15), '6 mm at constraint samples', 10, GOLD, 'ra')
        label(draw, (left, 127), '0 s', 12, MUTED)
        label(draw, (right, 127), f"{scene['duration']:.0f} s", 12, MUTED, 'ra')
        charts.append(image)
    return charts


def map_view(scene, time, closest=False):
    w, h = MAP_BOX[2] - MAP_BOX[0], MAP_BOX[3] - MAP_BOX[1]
    image = Image.new('RGB', (w, h), '#142336')
    draw = ImageDraw.Draw(image)
    x, q, control, body, other, gap = sample(scene, time)
    center = np.array([x[0] + 10, 0])
    if scene['curved']:
        s = 200 * math.atan2(x[0], 200 - x[1]) + 10
        center = np.array([200 * math.sin(s / 200), 200 * (1 - math.cos(s / 200))])
    scale = w / 46

    def transform(p):
        return (w / 2 + (p[0] - center[0]) * scale,
                h / 2 - (p[1] - center[1]) * scale)

    for gx in np.arange(math.floor((center[0] - 23) / 5) * 5, center[0] + 25, 5):
        px = transform((gx, center[1]))[0]
        draw.line([(px, 0), (px, h)], fill='#1B2C3D')
        if px < w - 110:
            label(draw, (px + 4, h - 21), f'{gx:.0f}', 12, MUTED)
    draw_road(draw, scene, transform)
    baseline = scene['baseline']
    if baseline and baseline['collisionDetected'] and not scene['baseline_only']:
        pose = [np.interp(time, baseline['times'], np.asarray(baseline['egoPoses'])[:, j]) for j in range(3)]
        polygon = rectangle(*pose, scene['shape'])
        danger = baseline['firstCollisionSeconds'] <= time <= baseline['lastCollisionSeconds']
        color = '#F16D7C' if danger else '#8391A3'
        for start, end in zip(polygon, polygon[1:] + [polygon[0]]):
            for fraction in np.arange(0, 1, .25):
                a = np.asarray(start) + fraction * (np.asarray(end) - start)
                b = np.asarray(start) + min(1, fraction + .14) * (np.asarray(end) - start)
                draw.line([transform(a), transform(b)], fill=color, width=2)
        point = transform(pose[:2])
        label(draw, (point[0], point[1] + 59), 'BASELINE COLLISION' if danger else 'CRUISE BASELINE', 12, color, 'ma')
    end = np.searchsorted(scene['times'], time, side='right')
    start = np.searchsorted(scene['times'], max(0, time - 2.5))
    if end - start > 1:
        draw.line([transform(p) for p in scene['states'][start:end:10, :2]],
                  fill=MUTED if scene['baseline_only'] else EGO, width=3)
        if scene['targets'] is not None:
            draw.line([transform(p) for p in scene['targets'][start:end:10, :2]], fill=TARGET, width=3)
    vehicle(draw, x, scene['shape'], transform, MUTED if scene['baseline_only'] else EGO,
            0 if scene['baseline_only'] else control[0])
    ego_point = transform(x[:2])
    label(draw, (ego_point[0], ego_point[1] - 47), 'CRUISE BASELINE' if scene['baseline_only'] else 'EGO',
          15, MUTED if scene['baseline_only'] else EGO, 'ma')
    if q is not None:
        vehicle(draw, q, scene['target'][7:11], transform, TARGET)
        target_point = transform(q[:2])
        if 0 < target_point[0] < w and 0 < target_point[1] < h:
            label(draw, (target_point[0], target_point[1] + 37), 'TARGET', 15, TARGET, 'ma')
        else:
            side = 'left' if target_point[0] < 0 else 'right' if target_point[0] > w else 'above'
            label(draw, (w - 18, 42), f'Target off-screen: {side}', 14, TARGET, 'ra')
        if 0 < gap < 1:
            _, p, v = nearest_pair(body, other)
            draw.line([transform(p), transform(v)], fill=GOLD, width=1)
    label(draw, (18, 14), 'FOLLOW CAMERA · TRUE BODY SCALE', 13, MUTED)
    if scene['baseline_only']:
        label(draw, (18, 39), 'COUNTERFACTUAL ONLY · INITIALIZATION FAILED', 16, TARGET)
    label(draw, (w - 17, h - 21), 'world X / m', 12, MUTED, 'ra')
    if closest:
        draw.rounded_rectangle((w - 258, 10, w - 15, 39), 8, fill='#473724')
        label(draw, (w - 135, 16), 'BASELINE COLLISION' if scene['baseline_only'] else 'CLOSEST RECORDED APPROACH', 12, GOLD, 'ma')
    # A real-world scale bar makes the body dimensions explicit.
    draw.line([(22, h - 47), (22 + 5 * scale, h - 47)], fill=TEXT, width=2)
    label(draw, (22 + 2.5 * scale, h - 67), '5 m', 12, TEXT, 'ma')
    return image


def side_view(scene, time):
    w, h = SIDE_BOX[2] - SIDE_BOX[0], SIDE_BOX[3] - SIDE_BOX[1]
    image = Image.new('RGB', (w, h), PANEL)
    draw = ImageDraw.Draw(image)
    label(draw, (16, 12), 'CRUISE BASELINE OVERVIEW' if scene['baseline_only'] else 'FULL REPLAY OVERVIEW', 13, MUTED)
    lo, hi = scene['bounds']
    scale = min((w - 40) / (hi[0] - lo[0]), 131 / (hi[1] - lo[1]))
    center = (lo + hi) / 2

    def transform(p):
        return (w / 2 + (p[0] - center[0]) * scale, 99 - (p[1] - center[1]) * scale)

    centerline, upper, lower = road_points(scene)
    # Overview uses a clipping layer to keep the road out of the detail card.
    overview = Image.new('RGB', (w, 158), PANEL)
    od = ImageDraw.Draw(overview)
    od.polygon([transform(p) for p in np.concatenate([upper, lower[::-1]])], fill='#1F3041')
    od.line([transform(p) for p in scene['states'][::12, :2]], fill=MUTED if scene['baseline_only'] else '#388D91', width=2)
    if scene['targets'] is not None:
        od.line([transform(p) for p in scene['targets'][::12, :2]], fill='#9F665E', width=2)
    x, q, _, body, other, gap = sample(scene, time)
    for pose, color in [(x, MUTED if scene['baseline_only'] else EGO), (q, TARGET)]:
        if pose is not None:
            p = transform(pose[:2]);od.ellipse((p[0] - 4, p[1] - 4, p[0] + 4, p[1] + 4), fill=color)
    image.paste(overview.crop((0, 30, w, 158)), (0, 30))
    draw = ImageDraw.Draw(image)
    draw.line([(16, 164), (w - 16, 164)], fill=BORDER)
    if gap is not None and gap < .6:
        label(draw, (16, 177), 'BASELINE OVERLAP · EQUAL AXIS SCALE' if scene['baseline_only'] and gap == 0
              else 'GAP DETAIL · EQUAL AXIS SCALE', 12, MUTED)
        _, a, b = nearest_pair(body, other)
        center = (a + b) / 2
        span = max(.34, gap * 2.5 + .15)
        scale = (w - 32) / span
        detail = Image.new('RGB', (w - 32, 139), '#152337')
        dd = ImageDraw.Draw(detail)

        def close(p):
            return (detail.width / 2 + (p[0] - center[0]) * scale,
                    detail.height / 2 - (p[1] - center[1]) * scale)

        for polygon, color in [(body, MUTED if scene['baseline_only'] else EGO), (other, TARGET)]:
            dd.polygon([close(p) for p in polygon], fill=color)
        if gap > 0:
            dd.line([close(a), close(b)], fill=GOLD, width=1)
        image.paste(detail, (16, 202))
        label(draw, (16, 351), format_gap(gap), 24, GOLD)
        label(draw, (w - 16, 360), 'baseline body distance' if scene['baseline_only'] else 'actual body distance', 12, MUTED, 'ra')
    else:
        label(draw, (16, 180), 'WHAT TO WATCH', 12, MUTED)
        for index, text in enumerate(DESCRIPTIONS[scene['name']]):
            label(draw, (16, 210 + 25 * index), text, 16, TEXT)
        label(draw, (16, 287), 'BASELINE MINIMUM DISTANCE' if scene['baseline_only'] else 'MINIMUM RECORDED DISTANCE', 12, MUTED)
        label(draw, (16, 311), format_gap(scene['minimum_display']), 28,
              GOLD if scene['target'] else TEXT)
        note = (f"Baseline collision starts at {scene['baseline']['firstCollisionSeconds']:.3f} s"
                if scene['baseline'] and scene['baseline']['collisionDetected']
                else 'Recorded paths are shown in the overview.')
        label(draw, (16, 354), note, 13, MUTED)
    return image


def render_frame(scene, time, rate=1, closest=False, intro=False):
    image = Image.new('RGB', (WIDTH, HEIGHT), BACKGROUND)
    draw = ImageDraw.Draw(image)
    label(draw, (32, 20), 'COLLISION AVOIDANCE / COUNTERFACTUAL CRUISE' if scene['baseline_only']
          else 'COLLISION AVOIDANCE / RECORDED EXPERIMENTS', 14, MUTED)
    label(draw, (32, 48), TITLES[scene['name']], 34)
    subtitle = (f"Reference {scene['speed']} m/s  ·  {scene['speed'] * 3.6:.1f} km/h  ·  "
                f"{'Baseline minimum' if scene['baseline_only'] else 'Closest'}: {format_gap(scene['minimum_display'])}")
    if scene['baseline'] and scene['baseline']['collisionDetected']:
        subtitle += f"  ·  Baseline collision at {scene['baseline']['firstCollisionSeconds']:.3f} s"
    label(draw, (32, 101), subtitle, 19, MUTED)
    label(draw, (1566, 48), f"{scene['index']:02d} / 14", 30, TEXT, 'ra')
    label(draw, (1392 if scene['baseline_only'] else 1440, 94), 'BASELINE' if scene['baseline_only'] else 'EGO', 15, MUTED)
    label(draw, (1507, 94), 'TARGET', 15, MUTED)
    first_dot = 1373 if scene['baseline_only'] else 1421
    draw.ellipse((first_dot, 100, first_dot+8, 108), fill=MUTED if scene['baseline_only'] else EGO)
    draw.ellipse((1488, 100, 1496, 108), fill=TARGET)
    for index in range(14):
        left = 32 + index * 110
        color = EGO if index == scene['index'] - 1 else '#38606A' if index < scene['index'] - 1 else BORDER
        draw.rounded_rectangle((left, 145, left + 99, 150), 2, fill=color)
    x, q, control, _, _, gap = sample(scene, time)
    values = [('BASELINE SPEED' if scene['baseline_only'] else 'EGO SPEED', f'{math.hypot(x[3], x[4]):.2f} m/s', EGO),
              ('STEERING', 'No command' if scene['baseline_only'] else f'{math.degrees(control[0]):+.2f} deg', EGO),
              ('BASELINE BODY GAP' if scene['baseline_only'] else 'CURRENT BODY GAP', format_gap(gap), GOLD if q is not None else MUTED),
              ('SIMULATION TIME', f"{time:.2f} / {scene['duration']:.0f} s", TEXT)]
    for index, (title, value, color) in enumerate(values):
        left = 32 + index * 388
        card(draw, (left, 174, left + 368, 252))
        label(draw, (left + 16, 185), title, 12, MUTED)
        label(draw, (left + 16, 207), value, 26, color)
    image.paste(map_view(scene, time, closest), MAP_BOX[:2])
    image.paste(side_view(scene, time), SIDE_BOX[:2])
    draw = ImageDraw.Draw(image)
    for box in [MAP_BOX, SIDE_BOX]:
        draw.rounded_rectangle(box, 12, outline=BORDER, width=1)
    if intro:
        card(draw, (56, 548, 866, 643), fill='#152A3D')
        label(draw, (76, 560), f"SCENE {scene['index']:02d}  /  {TITLES[scene['name']]}", 23)
        label(draw, (76, 600), '  '.join(DESCRIPTIONS[scene['name']]), 16, MUTED)
    for index, chart in enumerate(scene['charts']):
        left = 32 + 520 * index
        image.paste(chart, (left, 686))
        draw = ImageDraw.Draw(image)
        cursor = left + 53 + time / scene['duration'] * 424
        if not (scene['baseline_only'] and index == 1):
            draw.line([(cursor, 724), (cursor, 803)], fill='#D2E1EC', width=1)
        draw.rounded_rectangle((left, 686, left + 496, 840), 12, outline=BORDER, width=1)
    playback = 'Paused at closest approach' if closest else '0.5x replay' if rate == .5 else '1x replay'
    if closest and scene['baseline_only']:
        playback = 'Paused at baseline collision'
    footer = ('Dashed ghost: given-path constant-speed cruise without avoidance  ·  6 mm at constraint samples'
              if scene['baseline'] and scene['baseline']['collisionDetected']
              else 'True body geometry  ·  recorded ode45 motion  ·  6 mm buffer at constraint samples')
    if scene['baseline_only']:
        reason = scene['source']['failure'].split(':', 2)[1]
        footer = f'No avoidance trajectory available: {reason}. Showing cruise counterfactual only.'
    label(draw, (32, 860), footer, 14, MUTED)
    label(draw, (1568, 860), playback, 14, GOLD, 'ra')
    return image


def scene_timeline(scene, fps):
    frames = [(0., 1., False, True)] * fps
    if scene['target']:
        t = scene['closest_time']
        boundaries = sorted(set([0., max(0., t - .65), t, min(scene['duration'], t + .65), scene['duration']]))
        for start, end in zip(boundaries[:-1], boundaries[1:]):
            midpoint = (start + end) / 2
            rate = .5 if abs(midpoint - t) <= .65 + 1e-10 else 1.
            for time in np.arange(start, end - 1e-10, rate / fps):
                frames.append((float(time), rate, False, False))
            if abs(end - t) < 1e-10:
                frames += [(t, .5, True, False)] * round(.9 * fps)
    else:
        frames += [(float(t), .5, False, False) for t in np.arange(0, scene['duration'] - 1e-10, .5 / fps)]
    frames += [(scene['duration'], 1., False, False)] * round(.7 * fps)
    return frames


def cover(scenes, final=False):
    image = Image.new('RGB', (WIDTH, HEIGHT), BACKGROUND)
    draw = ImageDraw.Draw(image)
    label(draw, (44, 28), 'COLLISION AVOIDANCE / NOMINAL CLOSED-LOOP REPLAY', 16, MUTED)
    threats = all(s['baseline'] and s['baseline']['collisionDetected'] for s in scenes)
    label(draw, (44, 77), 'Fourteen real collision threats.' if threats else 'Fourteen scenarios. One controller.', 48)
    label(draw, (44, 145), '8 and 15 m/s  ·  true vehicle dimensions  ·  slowed closest approaches', 22, MUTED)
    selected = [scenes[0], scenes[1], scenes[3], scenes[5]] if threats else [scenes[5], scenes[6], scenes[12], scenes[8]]
    for i, scene in enumerate(selected):
        left, top = 44 + (i % 2) * 772, 219 + (i // 2) * 256
        frame = map_view(scene, scene['closest_time'])
        frame = frame.resize((740, 226), Image.Resampling.LANCZOS)
        image.paste(frame, (left, top))
        draw = ImageDraw.Draw(image)
        draw.rounded_rectangle((left + 12, top + 13, left + 564, top + 47), 7, fill='#101E2F')
        label(draw, (left + 24, top + 20), f"{scene['speed']} m/s · {TITLES[scene['name']]}", 16)
    if final:
        complete = sum(not s['baseline_only'] for s in scenes)
        headline = (f'{complete}/14 avoidance replays completed · {14-complete} initialization failures'
                    if complete < 14 else '14 completed · no collision detected in the recorded replay')
        minimum = min(s['source']['minimumReplayClearanceMeters'] for s in scenes
                      if s['source']['minimumReplayClearanceMeters'] is not None)
        maximum = max(h['controllerSeconds'] for s in scenes for h in s['source']['trace'][1:])
        detail = (f'Minimum body distance {minimum * 1000:.3f} mm  ·  '
                  f'max running control call {maximum * 1000:.3f} ms')
    else:
        headline = 'Cyan: ego vehicle     Coral: target vehicle'
        detail = ('Gray: cruise baseline. Failed initializations show counterfactual motion only.' if threats
                  else 'Closest-approach pauses and gap details show the actual recorded geometry.')
    label(draw, (44, 750), headline, 25, EGO if final else TEXT)
    label(draw, (44, 800), detail, 18, MUTED)
    label(draw, (44, 853), 'Exact observations · no injected sensing or computation delay · recorded 6 mm configuration', 14, MUTED)
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
        scene = prepare_scene(args.data_directory / f'speed{speed}' / f'{name}.json', speed, len(scenes) + 1)
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
            simulationSeconds=scene['duration'], closestSimulationSeconds=scene['closest_time'],
            minimumBodyClearanceMeters=scene['source']['minimumReplayClearanceMeters'],
            independentGeometryAgrees=not scene['baseline_only'],
            completed=scene['source']['completed'], rolloutType='counterfactualBaselineOnly' if scene['baseline_only'] else 'executedAvoidance',
            initializationFailure=scene['source']['failure'],
            baselineCollisionConfirmed=bool(scene['baseline'] and scene['baseline']['collisionDetected'])))
    for index in [1, 2, 6, 7, 13, 14]:
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
               '-threads', '2', '-pix_fmt', 'yuv420p', str(temporary)]
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
