function prediction = ltvBicyclePrediction(model, storedSchedule)
% ltvBicyclePrediction Scheduled Frenet LTV dynamic-bicycle condensation, with the braking tail.
%
% Builds the linear time-varying prediction the two-stage program is
% written over, in PATH COORDINATES: state x = [s; d; ePsi; vx; vy;
% r] (station and left-positive lateral offset along the lane
% centerline, heading error to the path tangent, body velocities and
% yaw rate). The HEAD (nodes 0..N) is the Frenet dynamic bicycle of
% ltvBicycleStageMatrices, linearized about the SCHEDULE - the frozen
% speed vBar = max(vx, scheduleSpeedFloor) and the stations
% s_k = s_0 + k vBar Ts with the centerline curvature at each of them.
% The TAIL (nodes N+1..N+N_b) is the kinematic braking tail of the
% terminal set (kinematicBrakingTail, TWO_STAGE_SAFETY.md "The
% terminal set"): from the terminal state x_N the station and speed
% advance under N_b further accelerations a_N..a_{N+N_b-1}, which are
% decision variables,
%
%   v_{k+1} = v_k + Ts a_k,
%   s_{k+1} = s_k + Ts v_k + 1/2 Ts^2 a_k + Ts kappa_k vhat_k d_N,
%
% exact for piecewise-constant acceleration up to the last term, the
% first-order Frenet coupling (1 - kappa d)^-1 with the speed frozen at
% the tail SCHEDULE vhat - the time-optimal braking profile from vBar,
% the tail's analogue of the head's frozen vBar. The lateral states
% (d, ePsi, vy, r) are carried frozen at their terminal values: the
% tail's lane-keeping law holds the terminal lateral offset within the
% band terminalLateralCertificate certifies, and the rows charge that
% band, so no lateral dynamics are predicted there. The schedule
% depends only on the measured state and the route; it is regenerated
% from scratch at every sample and no previous solution enters the
% model.
%
% The condensed affine map at every node is over the PLAN columns -
% the N head inputs stacked column-wise, then the N_b tail
% accelerations,
%
%   x_k = egoStateMatrix(:, :, k+1) * plan + egoStateOffset(:, k+1),
%
% k = 0..N+N_b, node 1 being the measured state (zero matrix); head
% nodes have zero tail columns. Under the Euler step the s, d and ePsi
% rows of B_d vanish, so the pose at node 2 (k = 1) is a FACT of the
% measured state, and the input's authority over a node's position
% grows like the fourth power of the node index.
%
% egoStateErrorBound carries the declared row-tightening radii in the
% path coordinates: the measured estimation radii at node 1 (position
% radii summed, conservatively, into both s and d), the one-step
% reachable set |A_1| E + Ts W at node 2, and the measured radii held
% constant beyond, tail included. referenceInput is the schedule
% reference (curvature-feedforward steering, zero acceleration) and
% referencePlan that reference followed by its braking tail: the
% linearization nominal when no previous plan is held.

    if nargin < 2
        storedSchedule = [];
    end
    cfg = model.cfg;
    horizonSteps = model.horizonSteps;
    sampleTime = model.sampleTime;
    inputDimension = model.inputDimension;
    controlCount = inputDimension*horizonSteps;
    tailSteps = model.tailSteps;
    planCount = controlCount+tailSteps;
    headNodeCount = horizonSteps+1;
    nodeCount = headNodeCount+tailSteps;
    initialState = model.initialEgoState;
    accelerationBias = model.longitudinalAccelerationBias;

    % ---- the episode schedule. A compatible stored schedule is consumed
    % verbatim after the caller shifts it by one node and appends a rest
    % node. Otherwise this is the episode anchor schedule.
    scheduleShifted = localStoredScheduleValid(storedSchedule, nodeCount);
    if scheduleShifted
        vBar = storedSchedule.speed;
        scheduleStation = storedSchedule.station;
        scheduleSpeedProfile = storedSchedule.speedProfile;
        scheduleCurvature = storedSchedule.curvature;
        tailSchedule = storedSchedule.tailAcceleration;
    else
        vBar = max(initialState(4), cfg.model.scheduleSpeedFloor);
        scheduleStation = zeros(1, nodeCount);
        scheduleSpeedProfile = zeros(1, nodeCount);
        scheduleStation(1:headNodeCount) = ...
            initialState(1)+(0:horizonSteps)*vBar*sampleTime;
        scheduleSpeedProfile(1:headNodeCount) = vBar;
        tailSchedule = kinematicBrakingTail("profile", cfg, tailSteps, ...
            min(vBar, cfg.model.speedMaximum));
        for stageIdx = 1:tailSteps
            nodeIdx = headNodeCount+stageIdx;
            scheduleStation(nodeIdx) = scheduleStation(nodeIdx-1) ...
                + sampleTime*scheduleSpeedProfile(nodeIdx-1) ...
                + 0.5*sampleTime^2*tailSchedule(stageIdx);
            scheduleSpeedProfile(nodeIdx) = max(0.0, ...
                scheduleSpeedProfile(nodeIdx-1) ...
                    + sampleTime*tailSchedule(stageIdx));
        end
        scheduleCurvature = zeros(1, nodeCount);
        for nodeIdx = 1:nodeCount
            scheduleCurvature(nodeIdx) = laneCurvatureAtStation( ...
                scheduleStation(nodeIdx), model.lane);
        end
    end

    % ---- the head: one forward-Euler stage per sample, condensed.
    egoStateMatrix = zeros(6, planCount, nodeCount);
    egoStateOffset = zeros(6, nodeCount);
    egoStateOffset(:, 1) = initialState;
    condensedMatrix = zeros(6, planCount);
    condensedOffset = initialState;
    stageMatrixA = zeros(6, 6, horizonSteps);
    stageMatrixB = zeros(6, inputDimension, horizonSteps);
    stageAffine = zeros(6, horizonSteps);
    for stageIdx = 1:horizonSteps
        [stateMatrix, inputMatrix, affineVector] = ...
            ltvBicycleStageMatrices(scheduleCurvature(stageIdx), vBar, ...
                sampleTime, cfg);
        % Declared longitudinal model bias (offset-free disturbance
        % term of Ge et al. 2022): one Euler step of the estimator's
        % published mismatch between vxdot = a + vy r and the realized
        % longitudinal acceleration. Zero when nothing is published.
        affineVector(4) = affineVector(4)+sampleTime*accelerationBias;
        stageMatrixA(:, :, stageIdx) = stateMatrix;
        stageMatrixB(:, :, stageIdx) = inputMatrix;
        stageAffine(:, stageIdx) = affineVector;
        inputRange = (stageIdx-1)*inputDimension+(1:inputDimension);
        condensedMatrix = stateMatrix*condensedMatrix;
        condensedMatrix(:, inputRange) = ...
            condensedMatrix(:, inputRange)+inputMatrix;
        condensedOffset = stateMatrix*condensedOffset+affineVector;
        egoStateMatrix(:, :, stageIdx+1) = condensedMatrix;
        egoStateOffset(:, stageIdx+1) = condensedOffset;
    end

    % ---- the tail: the kinematic braking stages from the terminal
    % state, the lateral states frozen there.
    terminalMatrix = egoStateMatrix(:, :, headNodeCount);
    terminalOffset = egoStateOffset(:, headNodeCount);
    stationRow = terminalMatrix(1, :);
    stationOffset = terminalOffset(1);
    speedRow = terminalMatrix(4, :);
    speedOffset = terminalOffset(4);
    for stageIdx = 1:tailSteps
        nodeIdx = headNodeCount+stageIdx;
        column = controlCount+stageIdx;
        coupling = sampleTime*scheduleCurvature(nodeIdx-1) ...
            * scheduleSpeedProfile(nodeIdx-1);
        nextStationRow = stationRow+sampleTime*speedRow ...
            + coupling*terminalMatrix(2, :);
        nextStationRow(column) = nextStationRow(column)+0.5*sampleTime^2;
        nextStationOffset = stationOffset+sampleTime*speedOffset ...
            + coupling*terminalOffset(2);
        nextSpeedRow = speedRow;
        nextSpeedRow(column) = nextSpeedRow(column)+sampleTime;
        egoStateMatrix(:, :, nodeIdx) = terminalMatrix;
        egoStateOffset(:, nodeIdx) = terminalOffset;
        egoStateMatrix(1, :, nodeIdx) = nextStationRow;
        egoStateOffset(1, nodeIdx) = nextStationOffset;
        egoStateMatrix(4, :, nodeIdx) = nextSpeedRow;
        egoStateOffset(4, nodeIdx) = speedOffset;
        stationRow = nextStationRow;
        stationOffset = nextStationOffset;
        speedRow = nextSpeedRow;
    end

    % Measured radii in path coordinates: the Cartesian position radii
    % are summed into both s and d (a box rotated into the path frame
    % lies inside that), the heading, speed, lateral-speed and yaw-rate
    % radii carry over.
    measuredRadii = model.measuredEgoStateErrorBound(:);
    pathRadii = [sum(measuredRadii(1:2)); sum(measuredRadii(1:2)); ...
        measuredRadii(3:6)];
    egoStateErrorBound = repmat(pathRadii, 1, nodeCount);
    egoStateErrorBound(:, 2) = abs(stageMatrixA(:, :, 1))*pathRadii ...
        + sampleTime*(cfg.model.ltvModelErrorRateBound ...
            + cfg.model.plantModelResidualRateBound);

    referenceInput = zeros(inputDimension, horizonSteps);
    referenceInput(1, :) = atan(cfg.vehicle.wheelbase ...
        * scheduleCurvature(1:horizonSteps));
    referencePlan = [referenceInput(:); zeros(tailSteps, 1)];
    referenceTerminalSpeed = egoStateMatrix(4, :, headNodeCount) ...
        * referencePlan+egoStateOffset(4, headNodeCount);
    referenceTail = kinematicBrakingTail("profile", cfg, tailSteps, ...
        min(max(referenceTerminalSpeed, 0.0), cfg.model.speedMaximum));
    referencePlan(controlCount+(1:tailSteps)) = referenceTail(:);

    prediction = struct();
    prediction.scheduleSpeed = vBar;
    prediction.scheduleShifted = scheduleShifted;
    prediction.scheduleSpeedProfile = scheduleSpeedProfile;
    prediction.scheduleStation = scheduleStation;
    prediction.scheduleCurvature = scheduleCurvature;
    prediction.tailSchedule = tailSchedule;
    prediction.referenceInput = referenceInput;
    prediction.referencePlan = referencePlan;
    prediction.stageMatrixA = stageMatrixA;
    prediction.stageMatrixB = stageMatrixB;
    prediction.stageAffine = stageAffine;
    prediction.inputCount = controlCount;
    prediction.tailSteps = tailSteps;
    prediction.planCount = planCount;
    prediction.tailIndex = controlCount+(1:tailSteps);
    prediction.headNodeCount = headNodeCount;
    prediction.nodeCount = nodeCount;
    prediction.tailNodeIndex = headNodeCount+(1:tailSteps);
    prediction.egoStateMatrix = egoStateMatrix;
    prediction.egoStateOffset = egoStateOffset;
    prediction.egoStateErrorBound = egoStateErrorBound;
    prediction.scheduleForStore = struct( ...
        "speed", vBar, ...
        "station", scheduleStation, ...
        "speedProfile", scheduleSpeedProfile, ...
        "curvature", scheduleCurvature, ...
        "tailAcceleration", tailSchedule);
end

function valid = localStoredScheduleValid(schedule, nodeCount)
    valid = isstruct(schedule) && isscalar(schedule) ...
        && all(isfield(schedule, ["speed", "station", "speedProfile", ...
            "curvature", "tailAcceleration"])) ...
        && isnumeric(schedule.speed) && isscalar(schedule.speed) ...
        && isfinite(schedule.speed) ...
        && isequal(size(schedule.station), [1, nodeCount]) ...
        && isequal(size(schedule.speedProfile), [1, nodeCount]) ...
        && isequal(size(schedule.curvature), [1, nodeCount]) ...
        && all(isfinite(schedule.station)) ...
        && all(isfinite(schedule.speedProfile)) ...
        && all(isfinite(schedule.curvature)) ...
        && all(isfinite(schedule.tailAcceleration));
end
