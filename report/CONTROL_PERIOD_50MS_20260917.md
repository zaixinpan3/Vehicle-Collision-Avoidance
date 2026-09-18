# Can the control period shrink to the solve time? A 50 ms trial

September 17, 2026. Outcome: **not with the current admission cost**. On the
S-bend trials the cruise frames fit a 50 ms period with a wide margin, but
both encounter admission frames roughly double and miss the period, because
the certified horizon is fixed in seconds and its step count doubles when
the period halves.

## Why the period cannot follow the median solve time

The hold-node controller solves a cruise frame in about 7 ms at a 0.1 s
period. The period, however, must cover the worst frame, and the worst frame
is the encounter admission: restoration search, reformulation and a hard
solve over a horizon that must reach the finite exit (about 4.8 s for the
stationary target, 3.2 s for the oncoming one). Halving the period doubles
the stages of that horizon, so the number of rows, plan coordinates and
lifted variables of every admission program doubles while the time available
halves.

## Trial

`runSmoothReferenceControllerValidation(SampleTime=0.05, SampleCount=600,
Paths="sBend", RunStrictTiming=true)`: S-bend profile, 8 m/s, 600 holds
(30 s), cruise horizon kept at 1.6 s (32 stages), one thread, diagnostic 5 s
budget and strict 50 ms trials. The 0.1 s numbers are the hold-node results
of `NODE_CERTIFICATE_ADOPTION_20260917.md`.

| Scene | Period 0.1 s max frame (ms) | Period 0.05 s max frame (ms) | Period 0.05 s median (ms) | Strict trial at 0.05 s |
| --- | ---: | ---: | ---: | --- |
| cruise | 12.8 | 18.9 | 11.4 | 600/600, max 16.3 |
| stationary target | 67.1 | 125.4 (formulation 57.7, solve 63.4, 4 native) | 12.0 | 0/600, search stopped at 56 ms |
| oncoming target | 37.4 | 79.6 (formulation 38.0, solve 34.5, 3 native) | 12.2 | 52/600, stopped at admission (65 ms) |

Both diagnostic encounter trials complete 600/600 holds with node clearance
0.131 m (stationary) and 0.155 m (oncoming) above the margin and inter-node
dips of 0.1 mm and 2.0 mm; the strict trials stop at the first admission.
Cruise frames are slower than at 0.1 s because the cruise horizon has twice
the stages.

## Assessment

- 0.1 s remains the shortest period the current admission cost supports on
  this machine: the stationary admission (67 ms) fits it, the 50 ms period
  needs admission below 50 ms while its programs are twice as large.
- Shortening the period is worthwhile for the node certificate, since the
  inter-node gap shrinks with the period (0.4 m of travel per hold instead of
  0.8 m), but it requires admission cost to fall faster than linearly in the
  step count. The remaining admission cost is dominated by the restoration
  solve and the two geometry builds; a shorter certified horizon or a
  different restoration formulation would be algorithmic changes.
- The estimator runs at 80 Hz, so 50 ms would remain aligned with its
  12.5 ms sample period.

## Reproduction

```matlab
addpath('scripts');
runSmoothReferenceControllerValidation( ...
    OutputDirectory='/home/zai/.cache/collisionAvoidance/control-period-50ms-20260917/final', ...
    SampleCount=600, SampleTime=0.05, Paths="sBend", RunStrictTiming=true);
```

Compact summary:
[CONTROL_PERIOD_50MS_20260917.json](CONTROL_PERIOD_50MS_20260917.json).
