# Native observer and predictive controller

The Linux executable reads JSON Lines on standard input and flushes one JSON reply per request. Keep one process running across holds. The controller contains its configuration, NRMM observer, vehicle model, constraint and derivative evaluation, CasADi/IPOPT search, terminal membership check and plan-shift logic. The adjacent shared libraries are part of the executable package.

## Run on the PC

From the delivered PC archive or its installed directory:

```sh
./bin/collision-avoidance < example.jsonl
```

The example feeds a recorded initial sensor block and the next block from a stopped-lead experiment. The program computes both commands. For a closed-loop simulation on the development PC, run the following from the repository root in MATLAB. The existing independent plant advances using each command returned by the executable.

```matlab
addpath('scripts')
app = string(fullfile(pwd,'../build/collision-avoidance-pc/bin/collision-avoidance'));
runJointOutputFeedbackScenario(ControllerExecutable=app, Speeds=15, Frames=400, ...
    Scenarios=["stoppedLead","trafficBrakingLead","trafficHeadOn","trafficCrossing"], ...
    RandomSeed=20261107);
```

## Stream contract

Send `{"type":"warmup"}` before the recorded experiment to run a discarded private IPOPT problem. Send `{"type":"profile"}` to read the build configuration. The supplied PC build has a 15 m/s cruise reference, 50 ms holds, four 12.5 ms sensor intervals per hold, a 50 m detection range and a 0.1 m planning margin.

* `init` takes a fixed path in `reference` and one current reading in `block.samples`. Supply the reference origin in inertial meters, heading in radians, curvature in inverse meters and length in meters. Zero curvature gives a straight path. Supply the known initial ego information set in `block.initialHypotheses`. Each member has `ego=[x,y,heading,vxBody,vyBody,yawRate]`, four positive tire scales (front/rear stiffness, front/rear friction), and seven sensor biases (position x/y, acceleration x/y, yaw rate, radar x/y). Targets are acquired through radar.
* `step` takes the next complete `block` with its boundary `time` in seconds, the previously applied `heldInput`, and all sensor readings in acquisition order. Each reading contains `time`, inertial `position` in meters, body-frame inertial `acceleration` in m/s², `yawRate` in rad/s and `radarAvailable`. Include body-frame `relativePosition` in meters on detection. A missed return may carry null position components. Optional `relativeHeading` in radians and `radarPositionErrorMeters` describe that reading. A missing heading is JSON null. The preceding command applies to the boundary accelerometer reading.
* A `command` reply contains `actuatorInput=[steeringRadians,longitudinalRatio]`. Positive steering is left. Positive longitudinal ratio drives and negative ratio brakes. Apply the returned pair over the next hold. Diagnostics include target activation, accepted source, IPOPT calls, horizon length and PCBF deficit.
* An `error` reply contains a message and ends the current session. An explicit fresh `init` with a valid initial contract is required to start again.

## Rebuild

The development host needs MATLAB Coder, the project's CasADi setup, CMake, a C++17 compiler, Eigen3 and nlohmann/json. The running controller only loads native libraries. The configuration is fixed when the numerical code is generated. Regenerate the complete profile when changing the model, observer bounds, cruise speed or mesh.

```sh
matlab -batch "addpath('scripts','controller','config'); c=collisionAvoidanceControllerConfig; c.target.observerDomain=collisionThreatContract(strings(1,0), [8 15 20],ControllerConfiguration=c); exportArm64Sources('../build/pcbf-pc-work/src',c,'Intel->x86-64 (Linux 64)')"
scripts/native/buildController.sh ../build/pcbf-pc-work/src "$CASADI_ROOT" ../build/collision-avoidance-pc
```

The zero-deficit proposal uses the compact predictor tangents. When that proposal cannot meet the constraints, the primary PCBF solve differentiates the compiled held-input integration directly. Both paths check the same nonlinear trajectory, terminal conditions and inherited deficit before issuing a command.

The MATLAB campaign uses the packaged configuration for the independent plant. Its timings include request encoding, round trip and response decoding. The executable reports its own per-block processing time in `elapsedMs`. Simulation runs establish empirical outcomes for the tested streams. The terminal handoff uses the controller's assumed-safe continuation.
