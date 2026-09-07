function report = measurePlantModelResidualRateBound(results)
% measurePlantModelResidualRateBound One-step plant-model residual rates.
%
% Inspect recorded plant traces against the current scheduled Frenet model.
% componentMaximumRate reports sampled derivative residuals in
% [s; d; ePsi; vx; vy; r] units per second. Numerical differentiation and
% finite coverage make this an empirical model-validation measurement, not
% a proof of a continuous-time disturbance bound or a global operating set.
% Endpoint residuals divided by sample time are reported separately; they
% must never be substituted for a continuous-time disturbance bound.

    if nargin < 1
        results = localRunAutomatedScenarioSet();
    end
    if ~iscell(results)
        results = {results};
    end

    stateOrder = ["station", "lateralPosition", "headingError", ...
        "longitudinalVelocity", "lateralVelocity", "yawRate"];
    residualRate = zeros(6, 0);
    excludedSampleCount = 0;
    endpointRate = zeros(6,0);
    scenarios = struct( ...
        "description", strings(numel(results), 1), ...
        "sampleCount", zeros(numel(results), 1), ...
        "componentMaximumRate", zeros(6, numel(results)));
    for resultIdx = 1:numel(results)
        result = results{resultIdx};
        [scenarioRate, scenarioExcluded, scenarioEndpoint] = ...
            localScenarioResidualRates(result);
        residualRate = [residualRate, scenarioRate]; %#ok<AGROW>
        endpointRate = [endpointRate,scenarioEndpoint]; %#ok<AGROW>
        excludedSampleCount = excludedSampleCount + scenarioExcluded;
        scenarios.description(resultIdx) = ...
            string(result.scenario.description);
        scenarios.sampleCount(resultIdx) = size(scenarioRate, 2);
        scenarios.componentMaximumRate(:, resultIdx) = ...
            max([scenarioRate, zeros(6, 1)], [], 2);
    end
    if isempty(residualRate)
        error("measurePlantModelResidualRateBound:noSamples", ...
            "No residual sample lies at or above the schedule floor.");
    end

    report = struct( ...
        "stateOrder", stateOrder, ...
        "componentMaximumRate", max(residualRate, [], 2), ...
        "componentPercentile99", ...
            prctile(residualRate.', 99.0).', ...
        "sampleCount", size(residualRate, 2), ...
        "excludedSampleCount", excludedSampleCount, ...
        "scenarios", scenarios,"endpointMaximumRate",max(endpointRate,[],2), ...
        "continuousTimeBoundProven",false,"method","sampled trace finite differences against the current frozen Frenet generator");
end
function [residualRate,excludedCount,endpointRate] = localScenarioResidualRates(result)
    cfg = result.controllerConfiguration;
    sampleTime = cfg.controller.sampleTime;
    trace = result.plantTrace;
    residualRate = zeros(6,0);endpointRate = zeros(6,0);excludedCount = 0;
    if isempty(result.command) || isempty(trace.time),return;end
    time = trace.time(:).';
    cartesian = [trace.positionX,trace.positionY,trace.yaw, ...
        trace.longitudinalVelocity,trace.lateralVelocity,trace.yawRate].';
    geometry = struct("centerline",result.scenario.centerline,"boundaries",struct([]));
    if isfield(result.scenario.geometry,"referenceCurve")
        geometry.referenceCurve = result.scenario.geometry.referenceCurve;
    end
    ego = struct("position",cartesian(1:2,1),"yaw",cartesian(3,1),"speed",cartesian(4,1));
    [~,lane] = readPlanningInputs(ego,[],geometry,cfg);
    frenet = localFrenet(cartesian,lane);
    dt = diff(time);
    derivative = diff(frenet,1,2)./dt;
    midpoint = (time(1:end-1)+time(2:end))/2;
    states = (frenet(:,1:end-1)+frenet(:,2:end))/2;
    residualRate = zeros(6,0);endpointRate = zeros(6,0);excludedCount = nnz(dt<1e-6);
    for step = 1:numel(result.command)
        selected = dt>=1e-6 & midpoint>=(step-1)*sampleTime & midpoint<step*sampleTime;
        if ~any(selected), continue; end
        raw = result.controlState(step,:).';
        initial = localFrenet(raw,lane);
        if raw(4)<cfg.model.scheduleSpeedFloor
            excludedCount = excludedCount+nnz(selected);
            continue;
        end
        if isfield(result.attempts,"controllerEgoEstimate")
            estimate = result.attempts.controllerEgoEstimate{step};
            [observed,~] = readPlanningInputs(estimate,[],geometry,cfg);
            scheduledSpeed = observed.modelState(4);
            projection = laneGeometry.project(observed.position,lane);
            station = projection.station;
            bias = observed.longitudinalAccelerationBias;
        else
            scheduledSpeed = initial(4);station = initial(1);bias = 0;
        end
        curvature = laneGeometry.curvature(station,lane);
        if isfield(result.attempts,"metadata") && numel(result.attempts.metadata)>=step ...
                && isfield(result.attempts.metadata{step},"executedContinuousGenerator")
            generator = result.attempts.metadata{step}.executedContinuousGenerator;
            a = generator(:,1:6);b = generator(:,7:8);c = generator(:,9);
        elseif isfield(cfg.model,"linearizationPolicy") && string(cfg.model.linearizationPolicy)~="cruise"
            error("measurePlantModelResidualRateBound:missingExecutedModel", ...
                "Trajectory linearization requires its recorded executed generator; a previous input cannot reconstruct it.");
        else
            [a,b,c] = ltvBicycleModel.continuousMatrices(curvature,scheduledSpeed,cfg,[],bias);
        end
        input = result.command{step}.actuatorInput;
        residualRate = [residualRate,abs(derivative(:,selected)-a*states(:,selected)-b*input-c)]; %#ok<AGROW>
        transition = expm(sampleTime*[a,b,c;zeros(3,9)]);
        predicted = transition(1:6,:)*[initial;input;1];
        actual = localFrenet(result.controlState(step+1,:).',lane);
        endpointRate(:,end+1) = abs(actual-predicted)/sampleTime; %#ok<AGROW>
    end
end

function state = localFrenet(cartesian,lane)
    projection = laneGeometry.project(cartesian(1:2,:),lane);
    heading = cartesian(3,:)-projection.heading;
    state = [projection.station;projection.lateralPosition; ...
        atan2(sin(heading),cos(heading));cartesian(4:6,:)];
end

function results = localRunAutomatedScenarioSet()
    localAddRepositoryPaths();
    scenarioFunctions = { ...
        @() runStraightCenterlineCruiseScenario( ...
            Plot=false, Report=false); ...
        @() runCircularCenterlineCruiseScenario( ...
            Plot=false, Report=false); ...
        @() runOncomingVehicleAvoidanceScenario( ...
            Plot=false, Report=false); ...
        @() runCircularCenterlineStraightTargetAvoidanceScenario( ...
            Plot=false, Report=false)};
    results = cell(numel(scenarioFunctions), 1);
    for scenarioIdx = 1:numel(scenarioFunctions)
        results{scenarioIdx} = scenarioFunctions{scenarioIdx}();
    end
end

function localAddRepositoryPaths()
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    for folder = ["scripts", "controller", "config", "estimator"]
        candidate = fullfile(repositoryRoot, folder);
        if isfolder(candidate)
            addpath(candidate);
        end
    end
end
