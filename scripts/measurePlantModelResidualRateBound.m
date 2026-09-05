function report = measurePlantModelResidualRateBound(results)
% measurePlantModelResidualRateBound One-step plant-model residual rates.
%
% report = measurePlantModelResidualRateBound(results) measures, over
% one or more plant-in-the-loop scenario results from the
% runCenterlineCruiseScenario family, the componentwise one-step
% residual between the PassVeh14DOF plant advance and the declared
% LTV bicycle one-step map under the same commanded input,
%
%   residual(k) = xPlant(k+1) - f(xPlant(k), u(k)),
%
% divided by the control sample time into a per-second rate in the
% controller state order [x; y; yaw; vx; vy; r] (the yaw residual is
% wrapped). This is the measurement that validates
% model.plantModelResidualRateBound (and, against a higher-fidelity
% reference simulation, model.ltvModelErrorRateBound):
% a bound at or above the measured componentwise maximum makes the
% declared one-step growth of the first-step reachable node contain the
% plant advance along the measured envelope, which is premise C1 of the
% PCBF descent theorem. Only samples at or above the schedule speed floor
% enter the bound; the excluded count is reported, never hidden.
%
% Called with no argument, the function runs the automated
% plant-in-the-loop scenario set itself (straight cruise, circular
% cruise, oncoming avoidance, circular-centerline straight-target
% avoidance) and measures over all of them.
%
% report fields:
%   stateOrder             the six state labels
%   componentMaximumRate   6-by-1 measured maximum residual rate (per s)
%   componentPercentile99  6-by-1 99th-percentile residual rate (per s)
%   sampleCount            residual samples inside the validity domain
%   excludedSampleCount    samples below the validity speed
%   scenarios              per-scenario provenance with its own maxima

    if nargin < 1
        results = localRunAutomatedScenarioSet();
    end
    if ~iscell(results)
        results = {results};
    end

    stateOrder = ["positionX", "positionY", "yaw", ...
        "longitudinalVelocity", "lateralVelocity", "yawRate"];
    residualRate = zeros(6, 0);
    excludedSampleCount = 0;
    scenarios = struct( ...
        "description", strings(numel(results), 1), ...
        "sampleCount", zeros(numel(results), 1), ...
        "componentMaximumRate", zeros(6, numel(results)));
    for resultIdx = 1:numel(results)
        result = results{resultIdx};
        [scenarioRate, scenarioExcluded] = ...
            localScenarioResidualRates(result);
        residualRate = [residualRate, scenarioRate]; %#ok<AGROW>
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
        "scenarios", scenarios);
end
function [residualRate, excludedCount] = ...
        localScenarioResidualRates(result)
    cfg = result.controllerConfiguration;
    sampleTime = cfg.controller.sampleTime;
    validitySpeed = cfg.model.scheduleSpeedFloor;
    state = result.controlState;
    stepCount = numel(result.command);
    residualRate = zeros(6, stepCount);
    included = false(1, stepCount);
    excludedCount = 0;
    for stepIdx = 1:stepCount
        command = result.command{stepIdx};
        if isempty(command) || stepIdx + 1 > size(state, 1)
            break;
        end
        plantState = state(stepIdx, :).';
        if plantState(4) < validitySpeed
            excludedCount = excludedCount + 1;
            continue;
        end
        % One LTV bicycle step at the locally scheduled point (yaw
        % and yaw rate frozen at the measured values); the lane-frozen
        % schedule deviation is covered by the heading-domain rows.
        [stageA, stageB, stageC] = ltvBicycleStageMatrices( ...
            plantState(3), max(plantState(4), validitySpeed), ...
            plantState(6), sampleTime, cfg);
        modelNext = stageA * plantState ...
            + stageB * [command.longitudinalForce; ...
                command.frontWheelSteeringAngle] + stageC;
        residual = state(stepIdx + 1, :).' - modelNext;
        residual(3) = atan2(sin(residual(3)), cos(residual(3)));
        residualRate(:, stepIdx) = abs(residual) / sampleTime;
        included(stepIdx) = true;
    end
    residualRate = residualRate(:, included);
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
