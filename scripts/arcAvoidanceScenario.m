function results = arcAvoidanceScenario(duration, quiet, lightweight, cfgOverride, scene)
% Closed-loop arc-cruise / avoid / recover scenario with timing.
%
% The one entry point of the simulation: addpath the repository's
% scripts/ folder and call it. It puts controller/ and config/ on the
% path itself, so nothing else has to be set up first. The plant is
% the 14-DOF vehicle at the bottom of this file, deliberately unlike
% the controller's declared bicycle; the lead vehicle cruises along
% the same arc at a lower speed and is revealed inside the perception
% range.

    if nargin < 1 || isempty(duration)
        duration = 22.0;
    end
    if nargin < 2 || isempty(quiet)
        quiet = false;
    end
    if nargin < 3
        lightweight = false;
    end
    % Located relative to this script, never by absolute path: the
    % repository moves as one folder (controller/, scripts/, config/),
    % and a hardcoded home directory is the one thing that
    % cannot survive that.
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(projectRoot, 'controller'));
    addpath(fullfile(projectRoot, 'config'));

    if nargin < 4
        cfgOverride = [];
    end
    % Scene knobs, so the SAME closed loop can be re-run with the
    % curvature, the lead, or the perception range changed.
    if nargin < 5 || isempty(scene)
        scene = struct();
    end
    if ~isfield(scene, "radius"), scene.radius = 400.0; end
    if ~isfield(scene, "noTarget"), scene.noTarget = false; end
    if ~isfield(scene, "leadStation0"), scene.leadStation0 = 120.0; end
    if ~isfield(scene, "leadSpeed"), scene.leadSpeed = 5.0; end
    if ~isfield(scene, "perceptionRange")
        scene.perceptionRange = 50.0;
    end
    % A target whose heading differs from the path's: the case an
    % axis-aligned configuration box is loose on and the true
    % configuration polygon is exact on.
    if ~isfield(scene, "leadYawOffset"), scene.leadYawOffset = 0.0; end
    cfg = collisionAvoidanceControllerConfig(cfgOverride);
    sampleTime = cfg.controller.sampleTime;
    stepCount = round(duration/sampleTime);

    radius = scene.radius;
    road = struct('centerline', localArcPoints(radius, 900.0, 1.0));
    lane = localBuildLane(road.centerline);

    perceptionRange = scene.perceptionRange;
    leadSpeed = scene.leadSpeed;
    leadStation0 = scene.leadStation0;

    plantParameters = plant14dof("parameters");
    plantState = plant14dof("initialState", plantParameters, ...
        [0.0; 0.0; 0.0], cfg.referenceSpeed);
    collisionAvoidanceController('resetNominalTrajectory');

    results = struct();
    results.time = zeros(stepCount, 1);
    results.position = zeros(stepCount, 2);
    results.speed = zeros(stepCount, 1);
    results.lateralError = zeros(stepCount, 1);
    results.headingError = zeros(stepCount, 1);
    results.steering = zeros(stepCount, 1);
    results.acceleration = zeros(stepCount, 1);
    results.brakingRatio = zeros(stepCount, 1);
    results.solveTime = zeros(stepCount, 1);
    results.noSolution = false(stepCount, 1);
    results.failure = false(stepCount, 1);
    results.targetVisible = false(stepCount, 1);
    results.targetGap = inf(stepCount, 1);
    results.clearance = inf(stepCount, 1);
    results.leadPosition = zeros(stepCount, 2);
    results.bias = zeros(stepCount, 1);
    % Controller diagnostics (from the third output). The separation
    % rows are the hard support-function rows of the conic problem: `collisionMargin` is the measured state's margin (node
    % 0), `collisionPlanMargin` the least margin the committed plan
    % holds over every covered node, `imposedFrom` is the first imposed
    % node, and `closestRegion` identifies
    % which part of the target box the separating direction faces at
    % the plan's closest node (1 behind, 2 ahead, 3 left, 4 right, 5-8
    % corners). The certificate source identifies optimization or
    % execution of the carried continuation.
    results.collisionMargin = inf(stepCount, 1);
    results.collisionPlanMargin = inf(stepCount, 1);
    results.collisionActive = false(stepCount, 1);
    results.imposedFrom = zeros(stepCount, 1);
    results.closestRegion = zeros(stepCount, 1);
    results.closestNode = zeros(stepCount, 1);
    results.marginPredictedNext = inf(stepCount, 1);
    results.dualRegion = zeros(stepCount, 2);
    results.certificateSource = strings(stepCount, 1);
    results.carriedWitnessFeasible = false(stepCount, 1);
    % The terminal set (PCBF_CLF_ARCHITECTURE.md, "The terminal set"): the
    % least separation margin over the committed plan's braking tail,
    % the terminal heading error against its certified bound, the
    % committed tail's rest station, and whether the hard
    % target-conditioned terminal certificate held against the
    % controller's finite predicted target horizon.
    results.collisionTailMargin = inf(stepCount, 1);
    results.terminalHeadingError = zeros(stepCount, 1);
    results.terminalHeadingBound = zeros(stepCount, 1);
    results.restStation = zeros(stepCount, 1);
    results.terminalPredictionCertified = true(stepCount, 1);
    results.terminalPredictionMargin = inf(stepCount, 1);
    results.measuredClearance = inf(stepCount, 1);
    results.clfRelaxation = zeros(stepCount, 1);
    results.clfDerivativeResidual = zeros(stepCount, 1);
    results.clfInitialValue = zeros(stepCount, 1);
    results.objective = zeros(stepCount, 1);
    results.inputCost = zeros(stepCount, 1);
    results.exitFlag = zeros(stepCount, 1);
    results.iterations = zeros(stepCount, 1);
    results.solverCalls = zeros(stepCount, 1);
    results.nominalSource = strings(stepCount, 1);
    results.hardRowViolation = zeros(stepCount, 1);
    % The controller's inputs and plan per step, so any sample can be
    % rebuilt offline.
    results.egoInput = cell(stepCount, 1);
    results.targetInput = cell(stepCount, 1);
    results.plan = cell(stepCount, 1);
    lastCommand = [0.0; 0.0];
    % Estimator-side longitudinal disturbance observer: the declared
    % model says vxdot = a + vy*r, and the plant adds drag, rolling
    % resistance, and wheel-slip lag on top. The observer estimates
    % that mismatch from the realized acceleration and publishes it as
    % the controller's declared longitudinal model bias, which is what
    % makes the CLF's decrease demand offset-free.
    accelerationBias = 0.0;
    observerGain = 0.25;
    previousSpeed = cfg.referenceSpeed;
    for stepIdx = 1:stepCount
        time = (stepIdx-1)*sampleTime;
        readout = plant14dof("readout", plantParameters, plantState);
        leadStation = leadStation0+leadSpeed*time;
        leadTheta = leadStation/radius;
        leadPosition = [radius*sin(leadTheta); ...
            radius-radius*cos(leadTheta)];
        gap = norm(leadPosition-readout.position);

        targets = struct('targetPositionInertial', {});
        if gap <= perceptionRange && ~scene.noTarget
            targets = struct( ...
                'targetId', "lead", ...
                'targetPositionInertial', leadPosition, ...
                'targetVelocityInertial', ...
                    leadSpeed*[cos(leadTheta); sin(leadTheta)], ...
                'targetAccelerationInertial', ...
                    (leadSpeed^2/radius) ...
                        * [-sin(leadTheta); cos(leadTheta)], ...
                'targetYawInertial', leadTheta+scene.leadYawOffset, ...
                'targetLength', 4.8, 'targetWidth', 1.9);
        end

        if stepIdx > 1
            realizedAcceleration = ...
                (readout.longitudinalVelocity-previousSpeed) ...
                / sampleTime;
            modelledAcceleration = modifiedFialaTire.accelerationGain(cfg)*lastCommand(2) ...
                - ltvBicycleModel.roadLoad(readout.longitudinalVelocity, cfg)/cfg.vehicle.m ...
                + readout.lateralVelocity*readout.yawRate;
            accelerationBias = accelerationBias ...
                + observerGain*(realizedAcceleration ...
                    - modelledAcceleration-accelerationBias);
            accelerationBias = max(min(accelerationBias, 2.0), -2.0);
        end
        previousSpeed = readout.longitudinalVelocity;

        ego = struct( ...
            'positionX', readout.position(1), ...
            'positionY', readout.position(2), ...
            'yawAngle', readout.yaw, ...
            'longitudinalVelocity', readout.longitudinalVelocity, ...
            'lateralVelocity', readout.lateralVelocity, ...
            'yawRate', readout.yawRate, ...
            'longitudinalAcceleration', 0.0, ...
            'longitudinalAccelerationBias', accelerationBias, ...
            'heldActuatorInput', lastCommand);

        results.egoInput{stepIdx} = ego;
        results.targetInput{stepIdx} = targets;
        solveStart = tic;
        failed = false;
        try
            if lightweight
                % Request only the command for this timing measurement.
                command = collisionAvoidanceController( ...
                    ego, targets, road, cfg);
            else
                [command, plan, problem] = ...
                    collisionAvoidanceController( ...
                        ego, targets, road, cfg);
                results.plan{stepIdx} = plan;
                md = problem.metadata;
                results.collisionMargin(stepIdx) = md.collisionMargin;
                results.collisionPlanMargin(stepIdx) = ...
                    md.collisionPlanMargin;
                results.collisionActive(stepIdx) = md.collisionActive;
                results.imposedFrom(stepIdx) = md.collisionImposedFrom;
                results.closestRegion(stepIdx) = md.collisionClosestRegion;
                results.closestNode(stepIdx) = md.collisionClosestNode;
                if numel(md.collisionMarginProfile) >= 2
                    results.marginPredictedNext(stepIdx) = ...
                        md.collisionMarginProfile(2);
                    results.dualRegion(stepIdx, :) = ...
                        md.dualRegionProfile(1:2);
                end
                results.certificateSource(stepIdx) = md.certificateSource;
                results.carriedWitnessFeasible(stepIdx) = md.carriedWitnessFeasible;
                results.collisionTailMargin(stepIdx) = ...
                    md.collisionTailMargin;
                results.terminalHeadingError(stepIdx) = ...
                    md.terminalHeadingError;
                results.terminalHeadingBound(stepIdx) = ...
                    md.terminalHeadingBound;
                results.restStation(stepIdx) = md.restStation;
                results.terminalPredictionCertified(stepIdx) = ...
                    md.terminalPredictionCertified;
                results.terminalPredictionMargin(stepIdx) = ...
                    md.terminalPredictionMargin;
                results.measuredClearance(stepIdx) = ...
                    md.measuredClearance;
                results.clfRelaxation(stepIdx) = md.clfRelaxation;
                results.clfDerivativeResidual(stepIdx) = ...
                    md.clfDerivativeResidual;
                results.clfInitialValue(stepIdx) = md.clfInitialValue;
                results.objective(stepIdx) = md.objectiveValue;
                results.inputCost(stepIdx) = md.inputDeviationCost;
                results.exitFlag(stepIdx) = md.solverExitFlag;
                results.iterations(stepIdx) = md.solverIterations;
                results.solverCalls(stepIdx) = md.solverCallCount;
                results.nominalSource(stepIdx) = md.nominalSource;
                results.hardRowViolation(stepIdx) = ...
                    md.hardRowViolation;
            end
            % The commanded input is applied directly. The model bounds
            % steering angle and signed braking ratio but
            % does not constrain changes between samples.
            lastCommand = command.actuatorInput;
        catch err
            failed = true;
            % A "no solution" report is the controller's declared
            % answer (the program is infeasible), counted separately
            % from numerical failures. In both cases the harness holds
            % the previous steering and sets braking ratio to zero. What
            % the vehicle does after the controller has reported is
            % outside its contract, and holding a nonzero acceleration
            % indefinitely was measured to drive the plant out of the
            % model's speed domain.
            results.noSolution(stepIdx) = string(err.identifier) ...
                == "collisionAvoidanceController:noSolution";
            lastCommand(2) = 0.0;
            if ~quiet
                fprintf('step %d %s: %s\n', stepIdx, ...
                    localFailureLabel(err), err.message);
            end
        end
        results.solveTime(stepIdx) = toc(solveStart);
        results.failure(stepIdx) = failed;

        projection = laneGeometry.project(readout.position, lane);
        results.time(stepIdx) = time;
        results.position(stepIdx, :) = readout.position.';
        results.speed(stepIdx) = readout.longitudinalVelocity;
        results.lateralError(stepIdx) = projection.lateralPosition;
        results.headingError(stepIdx) = ...
            atan2(sin(readout.yaw-projection.heading), ...
                cos(readout.yaw-projection.heading));
        results.bias(stepIdx) = accelerationBias;
        results.steering(stepIdx) = lastCommand(1);
        results.acceleration(stepIdx) = modifiedFialaTire.accelerationGain(cfg)*lastCommand(2);
        results.brakingRatio(stepIdx) = lastCommand(2);
        results.targetVisible(stepIdx) = gap <= perceptionRange ...
            && ~scene.noTarget;
        results.targetGap(stepIdx) = gap;
        results.leadPosition(stepIdx, :) = leadPosition.';
        results.clearance(stepIdx) = localRectangleClearance( ...
            readout.position, readout.yaw, leadPosition, ...
            leadTheta+scene.leadYawOffset);

        plantState = plant14dof("step", plantParameters, ...
            plantState, lastCommand, sampleTime, 100);
    end

    if ~quiet
        localReport(results, sampleTime, cfg);
    end
end

function localReport(results, sampleTime, cfg)
    time = results.time;
    fprintf('\n===== closed-loop summary =====\n');
    fprintf(['steps: %d, sample %.3f s, no-solution reports %d, ' ...
        'other failures %d\n'], numel(time), sampleTime, ...
        sum(results.noSolution), ...
        sum(results.failure & ~results.noSolution));
    cruiseIdx = ~results.targetVisible;
    fprintf(['free cruise: |lateral error| max %.3f m, speed ' ...
        '%.2f-%.2f m/s\n'], ...
        max(abs(results.lateralError(cruiseIdx))), ...
        min(results.speed(cruiseIdx)), ...
        max(results.speed(cruiseIdx)));
    settled = time > 5.0 & ~results.targetVisible;
    if any(settled)
        fprintf(['settled cruise speed error: mean %+.3f m/s, ' ...
            'max |.| %.3f m/s (bias estimate %+.3f m/s^2)\n'], ...
            mean(results.speed(settled))-cfg.referenceSpeed, ...
            max(abs(results.speed(settled)-cfg.referenceSpeed)), ...
            results.bias(find(settled, 1, 'last')));
    end
    if any(results.targetVisible)
        fprintf(['avoidance: max |lateral| %.3f m, min clearance ' ...
            '%.3f m, speed min %.2f m/s\n'], max(abs(results.lateralError)), ...
            min(results.clearance), min(results.speed));
        fprintf('min gap to lead vehicle: %.2f m\n', ...
            min(results.targetGap));
    else
        fprintf('no target in this scene: max |lateral| %.3f m\n', ...
            max(abs(results.lateralError)));
    end
    tailIdx = time > time(end)-3.0;
    fprintf(['recovery (last 3 s): |lateral| %.3f m, speed ' ...
        '%.2f m/s, |steer| %.4f rad, CLF relaxation max %.3g\n'], ...
        max(abs(results.lateralError(tailIdx))), ...
        mean(results.speed(tailIdx)), ...
        max(abs(results.steering(tailIdx))), ...
        max(results.clfRelaxation(tailIdx)));
    issued = ~results.failure;
    visible = results.targetVisible & issued;
    if any(visible)
        active = results.collisionActive & visible;
        fprintf(['separation rows (target visible): measured margin ' ...
            'min %.3f m, plan margin min %.4f m, imposed from node ' ...
            '%d-%d; ' ...
            'rows active on %d samples'], ...
            min(results.collisionMargin(visible)), ...
            min(results.collisionPlanMargin(visible)), ...
            min(results.imposedFrom(visible)), ...
            max(results.imposedFrom(visible)), sum(active));
        if any(active)
            fprintf(' (%.2f s .. %.2f s)', time(find(active, 1)), ...
                time(find(active, 1, 'last')));
        end
        fprintf('\n');
        fprintf('certificate: checked optimization %d / carried continuation %d\n', ...
            sum(results.certificateSource(visible) == "checkedOptimization"), ...
            sum(results.certificateSource(visible) == "shiftedStoredPlan"));
        % Which way the stage-1 separating direction faces at the
        % plan's closest node: the homotopy class the plan is in.
        % Reporting only - nothing in the controller branches on it.
        regionNames = ["behind", "ahead", "left", "right", ...
            "rear-left", "rear-right", "front-left", "front-right"];
        parts = strings(1, 0);
        for regionIdx = 1:numel(regionNames)
            count = sum(results.closestRegion(visible) == regionIdx);
            if count > 0
                parts(end+1) = sprintf('%s %d', ...
                    regionNames(regionIdx), count); %#ok<AGROW>
            end
        end
        fprintf('separating direction at the closest node: %s\n', ...
            strjoin(parts, ' / '));
        % The terminal set: what the committed tails held, and whether
        % the finite target-prediction certificate held on every
        % issued sample.
        tailMargin = results.collisionTailMargin(visible);
        fprintf(['terminal set: tail margin min %.4f m over %d ' ...
            'samples with tail rows, terminal prediction held on %d of %d ' ...
            'issued samples (margin min %.3f m), terminal heading ' ...
            '|e| max %.4f rad of bound %.4f rad\n'], ...
            min(tailMargin), sum(isfinite(tailMargin)), ...
            sum(results.terminalPredictionCertified(visible)), sum(visible), ...
            min(results.terminalPredictionMargin(visible)), ...
            max(abs(results.terminalHeadingError(visible))), ...
            min(results.terminalHeadingBound(visible)));
    end
    % The realized per-step margin disturbance: the measured margin of
    % sample k against the value sample k-1 predicted for it. Only pairs
    % whose separating direction at that node is the same compare like
    % with like (each direction is a different margin).
    predicted = [inf; results.marginPredictedNext(1:end-1)];
    samePair = results.dualRegion(:, 1) ...
        == [0; results.dualRegion(1:end-1, 2)];
    both = isfinite(predicted) & isfinite(results.collisionMargin) ...
        & issued & samePair & visible;
    if any(both)
        disturbance = results.collisionMargin(both)-predicted(both);
        fprintf(['realized per-step margin disturbance: worst %.4f m ' ...
            '(loss), p99 %.4f m, over %d same-direction samples\n'], ...
            -min(disturbance), -prctile(disturbance, 1), sum(both));
    end
    relaxed = results.clfRelaxation > 1.0e-6 & issued;
    fprintf(['CLF relaxation: max %.4g (V per second), ' ...
        'nonzero on %d samples'], max(results.clfRelaxation), ...
        sum(relaxed));
    if any(relaxed)
        fprintf(' (%.2f s .. %.2f s)', time(find(relaxed, 1)), ...
            time(find(relaxed, 1, 'last')));
    end
    fprintf('; continuous derivative residual max %.3g\n', ...
        max(results.clfDerivativeResidual(issued)));
    fprintf(['hard-row violation of the committed plan: max %.3g; ' ...
        'input-deviation cost max %.4g\n'], ...
        max(results.hardRowViolation(issued)), ...
        max(results.inputCost(issued)));
    fprintf(['solver: exit flag ~= 1 on %d samples, ' ...
        'iterations median %.0f / max %d, calls max %d\n'], ...
        sum(results.exitFlag(issued) ~= 1), ...
        median(results.iterations(issued)), ...
        max(results.iterations(issued)), max(results.solverCalls));
    solve = results.solveTime;
    fprintf(['solve time: mean %.3f s, median %.3f s, p95 %.3f s, ' ...
        'max %.3f s\n'], mean(solve), median(solve), ...
        prctile(solve, 95), max(solve));
    fprintf('REAL-TIME FACTOR (mean solve / sample) = %.1f x\n', ...
        mean(solve)/sampleTime);
    fprintf('samples meeting the %.0f ms deadline: %d of %d\n', ...
        sampleTime*1000.0, sum(solve <= sampleTime), numel(solve));
end

function label = localFailureLabel(err)
    if string(err.identifier) == "collisionAvoidanceController:noSolution"
        label = "reported no solution";
    else
        label = "failed";
    end
end

function clearance = localRectangleClearance( ...
        egoPosition, egoYaw, targetPosition, targetYaw)
    clearance = avoidanceSafetyGeometry.rectangleDistance( ...
        egoPosition, egoYaw, targetPosition, targetYaw, ...
        [2.4; 0.95; 2.4; 0.95]);
end

function points = localArcPoints(radius, arcLength, spacing)
    theta = (0.0:spacing:arcLength).' / radius;
    points = [radius*sin(theta), radius-radius*cos(theta)];
end

function lane = localBuildLane(points)
    segment = diff(points, 1, 1);
    segmentLength = vecnorm(segment, 2, 2);
    lane = struct();
    lane.segmentStart = points(1:end - 1, :);
    lane.segment = segment;
    lane.segmentLength = segmentLength;
    lane.tangent = segment ./ segmentLength;
    lane.segmentStation = [0.0; cumsum(segmentLength(1:end-1))];
    heading = unwrap(atan2(lane.tangent(:, 2), lane.tangent(:, 1)));
    curvature = zeros(size(segment, 1), 1);
    centerSpacing = 0.5*(segmentLength(1:end - 1) ...
        + segmentLength(2:end));
    interfaceCurvature = diff(heading)./centerSpacing;
    curvature(1) = interfaceCurvature(1);
    curvature(end) = interfaceCurvature(end);
    curvature(2:end - 1) = 0.5*(interfaceCurvature(1:end - 1) ...
        + interfaceCurvature(2:end));
    lane.segmentCurvature = curvature;
end

% ====================================================================
% 14 DOF simulation plant of the harness (was plant14dof.m).
% ====================================================================

function out = plant14dof(action, varargin)
% plant14dof A 14-degree-of-freedom vehicle plant for closed-loop tests.
%
% Degrees of freedom: 6 sprung-body (X, Y, Z, roll, pitch, yaw), 4
% unsprung vertical travels, and 4 wheel spins. Tires use a linear
% slip model saturated on the friction ellipse of the instantaneous
% vertical load, so load transfer feeds back into grip. Deliberately
% unlike the controller's declared held-input Fiala bicycle: it has
% wheel-spin and vertical dynamics the controller never models.
%
%   parameters = plant14dof("parameters")
%   state      = plant14dof("initialState", parameters, pose, speed)
%   state      = plant14dof("step", parameters, state, input, dt, sub)
%   readout    = plant14dof("readout", parameters, state)

    switch string(action)
        case "parameters"
            out = localParameters();
        case "initialState"
            out = localInitialState(varargin{:});
        case "step"
            out = localStep(varargin{:});
        case "readout"
            out = localReadout(varargin{:});
        otherwise
            error("plant14dof:invalidAction", "Unknown action.");
    end
end

function p = localParameters()
    p = struct();
    p.totalMass = 1650.0;
    p.unsprungMass = 45.0;
    p.sprungMass = p.totalMass-4.0*p.unsprungMass;
    p.Ix = 550.0;
    p.Iy = 2200.0;
    p.Iz = 1700.0;
    p.lf = 1.4;
    p.lr = 1.65;
    p.halfTrack = 0.78;
    p.cgHeight = 0.55;
    p.rollCentreHeight = 0.15;
    p.gravity = 9.81;
    p.springRate = 42000.0;
    p.damperRate = 4200.0;
    p.tireVerticalRate = 260000.0;
    p.wheelInertia = 1.1;
    p.effectiveRadius = 0.32;
    p.corneringStiffness = [78000.0; 78000.0];
    p.longitudinalStiffness = 60000.0;
    p.frictionCoefficient = 0.9;
    p.rollingResistance = 0.012;
    p.dragArea = 0.72;
    p.airDensity = 1.2;
    % Corner geometry: [FL; FR; RL; RR] in body axes.
    p.cornerX = [p.lf; p.lf; -p.lr; -p.lr];
    p.cornerY = [p.halfTrack; -p.halfTrack; ...
        p.halfTrack; -p.halfTrack];
    p.staticLoad = localStaticLoad(p);
end

function load = localStaticLoad(p)
    frontLoad = p.totalMass*p.gravity*p.lr/(p.lf+p.lr)/2.0;
    rearLoad = p.totalMass*p.gravity*p.lf/(p.lf+p.lr)/2.0;
    load = [frontLoad; frontLoad; rearLoad; rearLoad];
end

function state = localInitialState(p, pose, speed)
% state = [X; Y; Z; roll; pitch; yaw; vx; vy; vz; p; q; r;
%          zU(4); zUdot(4); omega(4)]
    staticDeflection = p.staticLoad/p.springRate;
    state = zeros(24, 1);
    state(1:2) = pose(1:2);
    state(6) = pose(3);
    state(7) = speed;
    state(13:16) = -staticDeflection;
    state(21:24) = speed/p.effectiveRadius;
end

function state = localStep(p, state, input, dt, substeps)
    stepSize = dt/substeps;
    for k = 1:substeps
        k1 = localDerivative(p, state, input);
        k2 = localDerivative(p, state+0.5*stepSize*k1, input);
        k3 = localDerivative(p, state+0.5*stepSize*k2, input);
        k4 = localDerivative(p, state+stepSize*k3, input);
        state = state ...
            + (stepSize/6.0)*(k1+2.0*k2+2.0*k3+k4);
    end
end

function derivative = localDerivative(p, state, input)
    yaw = state(6);
    vx = state(7);
    vy = state(8);
    vz = state(9);
    rollRate = state(10);
    pitchRate = state(11);
    yawRate = state(12);
    roll = state(4);
    pitch = state(5);
    zBody = state(3);
    unsprungTravel = state(13:16);
    unsprungRate = state(17:20);
    spin = state(21:24);

    steering = input(1);
    brakingRatio = input(2);

    % Suspension: body corner vertical position and rate.
    cornerZ = zBody+p.cornerX*(-sin(pitch)) ...
        + p.cornerY*sin(roll);
    cornerRate = vz-p.cornerX*pitchRate*cos(pitch) ...
        + p.cornerY*rollRate*cos(roll);
    deflection = cornerZ-unsprungTravel;
    deflectionRate = cornerRate-unsprungRate;
    suspensionForce = -p.springRate*deflection ...
        - p.damperRate*deflectionRate;

    % Tire vertical force from the unsprung travel against the road.
    verticalLoad = max(0.0, p.staticLoad ...
        - p.tireVerticalRate*unsprungTravel);

    % Wheel planar velocities in the body frame.
    wheelVx = vx-yawRate*p.cornerY;
    wheelVy = vy+yawRate*p.cornerX;
    steerAngle = [steering; steering; 0.0; 0.0];
    contactVx = wheelVx.*cos(steerAngle)+wheelVy.*sin(steerAngle);
    contactVy = -wheelVx.*sin(steerAngle)+wheelVy.*cos(steerAngle);

    referenceSpeed = max(abs(contactVx), 1.0);
    slipAngle = -atan(contactVy./referenceSpeed);
    slipRatio = (spin*p.effectiveRadius-contactVx)./referenceSpeed;
    slipRatio = max(min(slipRatio, 1.0), -1.0);

    corneringStiffness = [p.corneringStiffness(1); ...
        p.corneringStiffness(1); p.corneringStiffness(2); ...
        p.corneringStiffness(2)];
    lateralLinear = corneringStiffness.*slipAngle;
    longitudinalLinear = p.longitudinalStiffness*slipRatio;
    capacity = p.frictionCoefficient*verticalLoad;
    demand = hypot(longitudinalLinear, lateralLinear);
    scale = ones(4, 1);
    saturated = demand > capacity & demand > 0.0;
    scale(saturated) = capacity(saturated)./demand(saturated);
    contactFx = longitudinalLinear.*scale;
    contactFy = lateralLinear.*scale;

    % The paper's common beta requests Fx_i=beta*mu_i*Fz_i at static loads.
    % Wheel dynamics and their actual contact-force losses remain in the plant.
    wheelTorque = brakingRatio*p.frictionCoefficient*p.staticLoad*p.effectiveRadius;
    rollingTorque = p.rollingResistance*verticalLoad ...
        * p.effectiveRadius .* tanh(spin);
    spinDerivative = (wheelTorque-contactFx*p.effectiveRadius ...
        - rollingTorque)/p.wheelInertia;

    % Body-frame tire forces.
    tireFx = contactFx.*cos(steerAngle)-contactFy.*sin(steerAngle);
    tireFy = contactFx.*sin(steerAngle)+contactFy.*cos(steerAngle);

    dragForce = 0.5*p.airDensity*p.dragArea*vx*abs(vx);
    totalFx = sum(tireFx)-dragForce;
    totalFy = sum(tireFy);
    totalFz = sum(suspensionForce)-p.sprungMass*p.gravity;

    vxDot = totalFx/p.totalMass+yawRate*vy;
    vyDot = totalFy/p.totalMass-yawRate*vx;
    vzDot = totalFz/p.sprungMass;

    rollMoment = sum(p.cornerY.*suspensionForce) ...
        + totalFy*(p.cgHeight-p.rollCentreHeight);
    pitchMoment = -sum(p.cornerX.*suspensionForce) ...
        - totalFx*p.cgHeight;
    yawMoment = sum(p.cornerX.*tireFy)-sum(p.cornerY.*tireFx);

    unsprungAcceleration = ( ...
        -suspensionForce ...
        - p.tireVerticalRate*unsprungTravel ...
        - p.unsprungMass*p.gravity)/p.unsprungMass;

    derivative = zeros(24, 1);
    derivative(1) = vx*cos(yaw)-vy*sin(yaw);
    derivative(2) = vx*sin(yaw)+vy*cos(yaw);
    derivative(3) = vz;
    derivative(4) = rollRate;
    derivative(5) = pitchRate;
    derivative(6) = yawRate;
    derivative(7) = vxDot;
    derivative(8) = vyDot;
    derivative(9) = vzDot;
    derivative(10) = rollMoment/p.Ix;
    derivative(11) = pitchMoment/p.Iy;
    derivative(12) = yawMoment/p.Iz;
    derivative(13:16) = unsprungRate;
    derivative(17:20) = unsprungAcceleration;
    derivative(21:24) = spinDerivative;
end

function readout = localReadout(~, state)
    readout = struct( ...
        "position", state(1:2), ...
        "yaw", state(6), ...
        "longitudinalVelocity", state(7), ...
        "lateralVelocity", state(8), ...
        "yawRate", state(12), ...
        "roll", state(4), ...
        "pitch", state(5), ...
        "verticalVelocity", state(9), ...
        "wheelSpin", state(21:24));
end
