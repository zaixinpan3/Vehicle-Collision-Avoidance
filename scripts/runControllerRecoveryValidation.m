function rows = runControllerRecoveryValidation(outputDirectory, suite)
%runControllerRecoveryValidation Audit avoidance and sustained path recovery.
% Raw results stay in the caller's external output directory. This experiment
% does not change the controller or treat an early stop as cruise recovery.
    arguments
        outputDirectory (1,1) string
        suite (1,1) string {mustBeMember(suite,["exact","vehicle","summarizeExact","summarizeVehicle"])}
    end
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root,"scripts"),fullfile(root,"config"),fullfile(root,"controller"));
    if ~isfolder(outputDirectory), mkdir(outputDirectory); end
    diary(fullfile(outputDirectory,suite+"-console.txt"));
    cleanup = onCleanup(@() diary('off'));
    rows = struct([]);
    if suite == "summarizeExact"
        files = [dir(fullfile(outputDirectory,'*_k*.mat')); ...
            dir(fullfile(outputDirectory,'*_straight_boundaries.mat'))];
        for index = 1:numel(files)
            loaded = load(fullfile(files(index).folder,files(index).name),'result');
            [~,name] = fileparts(files(index).name);
            rows = localSave(rows,localExactSummary(loaded.result,string(name)), ...
                loaded.result,outputDirectory,"exact",false);
        end
    elseif suite == "summarizeVehicle"
        names = ["straight_cruise","circular_cruise","straight_oncoming", ...
            "circular_oncoming","varying_curvature_oncoming", ...
            "estimated_straight_oncoming","estimated_circular_oncoming"];
        for name = names
            loaded = load(fullfile(outputDirectory,name+".mat"),'result');
            rows = localSave(rows,localVehicleSummary(loaded.result,name), ...
                loaded.result,outputDirectory,"vehicle",false);
        end
    elseif suite == "exact"
        for curvature = [0,.01,-.01]
            for scenario = ["cruise","stationary","oncoming","crossing"]
                name = string(sprintf('%s_k%+.2f',scenario,curvature));
                fprintf('\nSTART %s\n',name);
                result = runExactStateRecursiveFeasibilityScenario( ...
                    Scenario=scenario,RoadCurvature=curvature,SampleCount=600, ...
                    AuditSubsteps=10,DeadlineSeconds=30,SearchTimeLimitSeconds=30, ...
                    RethrowFailure=false,Seed=20260925);
                row = localExactSummary(result,name);
                rows = localSave(rows,row,result,outputDirectory,suite);
            end
        end
        for scenario = ["cruise","stationary","oncoming"]
            name = scenario+"_straight_boundaries";
            fprintf('\nSTART %s\n',name);
            result = runExactStateRecursiveFeasibilityScenario( ...
                Scenario=scenario,UseRoadBoundaries=true,SampleCount=600, ...
                AuditSubsteps=10,DeadlineSeconds=30,SearchTimeLimitSeconds=30, ...
                RethrowFailure=false,Seed=20260925);
            rows = localSave(rows,localExactSummary(result,name),result,outputDirectory,suite);
        end
    else
        names = ["straight_cruise","circular_cruise","straight_oncoming", ...
            "circular_oncoming","varying_curvature_oncoming", ...
            "estimated_straight_oncoming","estimated_circular_oncoming"];
        for name = names
            fprintf('\nSTART %s\n',name);
            try
                common = {'Duration',18,'ReferenceSpeed',10,'Plot',false,'Report',false};
                avoidance = [common,{'TargetSpeed',10,'RecoveryWindow',2}];
                switch name
                    case "straight_cruise"
                        result = runStraightCenterlineCruiseScenario(common{:});
                    case "circular_cruise"
                        result = runCircularCenterlineCruiseScenario(common{:});
                    case "straight_oncoming"
                        result = runOncomingVehicleAvoidanceScenario(avoidance{:});
                    case "circular_oncoming"
                        result = runCircularCenterlineStraightTargetAvoidanceScenario(avoidance{:});
                    case "varying_curvature_oncoming"
                        result = runVaryingCurvatureStraightTargetAvoidanceScenario(avoidance{:});
                    otherwise
                        configuration = estimatorControllerIntegrationConfig();
                        configuration.randomSeed = 20260925;
                        configuration.vehicle.targetSpeed = 10;
                        configuration.initialization.targetSpeedPrior = 10;
                        estimated = [avoidance,{'UseStateEstimator',true, ...
                            'EstimatorConfiguration',configuration}];
                        if name == "estimated_straight_oncoming"
                            result = runOncomingVehicleAvoidanceScenario(estimated{:});
                        else
                            result = runCircularCenterlineStraightTargetAvoidanceScenario(estimated{:});
                        end
                end
                row = localVehicleSummary(result,name);
            catch exception
                result = struct('exception',getReport(exception,'extended','hyperlinks','off'));
                row = localRow(name);
                row.failureIdentifier = string(exception.identifier);
                row.failureMessage = string(exception.message);
            end
            rows = localSave(rows,row,result,outputDirectory,suite);
        end
    end
end

function row = localRow(name)
    row = struct('name',name,'completed',false,'executedSeconds',0, ...
        'collisionFree',false,'minimumBodyGap',NaN,'roadBoundariesEnabled',false, ...
        'minimumRoadMargin',NaN,'passedTarget',false,'recoveredCruise',false, ...
        'maximumRecoverySpeedError',NaN,'maximumRecoveryLateralError',NaN, ...
        'maximumRecoveryCourseError',NaN,'minimumSpeed',NaN, ...
        'maximumLateralExcursion',NaN,'maximumFrameSeconds',NaN, ...
        'deadlineMisses',NaN,'failureIdentifier',"",'failureMessage',"", ...
        'baselineMinimumGap',NaN,'baselineCollides',NaN, ...
        'passingTime',NaN,'recoveryTime',NaN);
end

function row = localExactSummary(result,name)
    row = localRow(name);
    row.completed = result.completed;
    row.executedSeconds = result.time(end);
    row.collisionFree = result.executedHolds>0 && result.sampledCollisionFree;
    row.minimumBodyGap = result.minimumSampledBodyGap;
    row.roadBoundariesEnabled = result.roadBoundariesEnabled;
    row.minimumRoadMargin = result.minimumSampledRoadMargin;
    row.minimumSpeed = min(result.state(4,:));
    row.maximumLateralExcursion = max(abs(result.state(2,:)));
    row.maximumFrameSeconds = result.runtime.maximumSeconds;
    row.deadlineMisses = result.runtime.deadlineMisses;
    row.failureIdentifier = result.failureIdentifier;
    row.failureMessage = result.failureMessage;
    % Course, rather than body yaw, measures motion along the reference path.
    course = result.state(3,:)+atan2(result.state(5,:),result.state(4,:));
    course = atan2(sin(course),cos(course));
    mask = result.time>=30-2-1e-10;
    if result.completed && any(mask)
        row.maximumRecoverySpeedError = max(abs(result.state(4,mask)-result.configuration.referenceSpeed));
        row.maximumRecoveryLateralError = max(abs(result.state(2,mask)));
        row.maximumRecoveryCourseError = max(abs(course(mask)));
        row.recoveredCruise = row.maximumRecoverySpeedError<=.5 ...
            && row.maximumRecoveryLateralError<=.2 && row.maximumRecoveryCourseError<=.02;
    end
    inCruise = abs(result.state(4,:)-result.configuration.referenceSpeed)<=.5 ...
        & abs(result.state(2,:))<=.2 & abs(course)<=.02;
    if result.scenario == "cruise"
        row.passedTarget = true;
        row.passingTime = 0;
    elseif result.executedHolds>0
        for index = 1:numel(result.time)
            [position,heading] = localPose(result.state(:,index),result);
            target = nrmmTargetTruth(result.targetMotion,result.time(index));
            behind = dot(target(1:2)-position,[cos(heading);sin(heading)]) ...
                < -(result.configuration.vehicle.length/2+result.targetMotion.halfLength);
            if behind && ~row.passedTarget
                row.passedTarget = true;
                row.passingTime = result.time(index);
            end
        end
        % Independently check whether undisturbed cruise would hit the target.
        % Use the same 5 ms geometry audit density as the controlled trial.
        baselineGap = Inf;
        footprintHalfSizes = [result.configuration.vehicle.length/2;result.configuration.vehicle.width/2; ...
            result.targetMotion.halfLength;result.targetMotion.halfWidth];
        for time = 0:result.configuration.controller.sampleTime/result.auditSubsteps:30
            state = result.cruiseState;
            state(1) = result.state(1,1)+result.configuration.referenceSpeed*time;
            [position,heading] = localPose(state,result);
            target = nrmmTargetTruth(result.targetMotion,time);
            baselineGap = min(baselineGap,avoidanceSafetyGeometry.rectangleDistance( ...
                position,heading,target(1:2),target(7),footprintHalfSizes));
        end
        row.baselineMinimumGap = baselineGap;
        row.baselineCollides = baselineGap<0;
    end
    if row.recoveredCruise && row.passedTarget
        lastViolation = find(~inCruise,1,'last');
        if isempty(lastViolation),lastViolation = 0;end
        index = max(lastViolation+1,find(result.time>=row.passingTime,1));
        row.recoveryTime = result.time(index);
    end
end

function [position,heading] = localPose(state,result)
    if result.roadCurvature == 0
        position = [state(1)+result.road.centerline(1,1);state(2)];
        heading = state(3);
    else
        [position,heading] = laneGeometry.fromFrenet(state,result.road);
    end
end

function row = localVehicleSummary(result,name)
    row = localRow(name);
    row.completed = result.metrics.controllerCompletedScenario && ~result.failure.occurred;
    row.executedSeconds = result.metrics.completedControlSteps*result.scenario.sampleTime;
    row.failureIdentifier = string(result.failure.identifier);
    row.failureMessage = string(result.failure.message);
    row.maximumFrameSeconds = max(result.runtime.frameSeconds);
    row.deadlineMisses = nnz(result.runtime.frameSeconds>result.scenario.sampleTime);
    row.maximumLateralExcursion = max(abs(result.controlTracking.lateralError));
    if ~isempty(result.plantTrace.time)
        row.minimumSpeed = min(result.plantTrace.longitudinalVelocity);
    end
    if isfield(result,'avoidance')
        row.collisionFree = row.executedSeconds>0 && result.avoidance.collisionFree;
        row.minimumBodyGap = result.avoidance.minimumControlledSeparatingAxisMargin;
        row.passedTarget = result.avoidance.passedTarget;
        row.recoveredCruise = row.completed && result.avoidance.recoveredCruise;
        if row.completed
            row.maximumRecoverySpeedError = result.avoidance.maximumRecoverySpeedError;
            row.maximumRecoveryLateralError = result.avoidance.maximumRecoveryLateralError;
            row.maximumRecoveryCourseError = result.avoidance.maximumRecoveryHeadingError;
        end
    else
        row.collisionFree = row.completed;
        row.passedTarget = true;
        if row.completed
            mask = result.controlTime>=result.controlTime(end)-2-1e-10;
            course = result.controlTracking.headingError+atan2(result.controlState(:,5),result.controlState(:,4));
            course = atan2(sin(course),cos(course));
            row.maximumRecoverySpeedError = max(abs(result.controlState(mask,4)-result.scenario.referenceSpeed));
            row.maximumRecoveryLateralError = max(abs(result.controlTracking.lateralError(mask)));
            row.maximumRecoveryCourseError = max(abs(course(mask)));
            row.recoveredCruise = row.maximumRecoverySpeedError<=.5 ...
                && row.maximumRecoveryLateralError<=.2 && row.maximumRecoveryCourseError<=.02;
        end
    end
    row.roadBoundariesEnabled = ~isempty(result.perception.roadBoundaryOffsets);
    row.minimumRoadMargin = result.roadSafety.minimumBoundaryFunctionMargin;
end

function rows = localSave(rows,row,result,outputDirectory,suite,saveRaw)
    if nargin<6,saveRaw = true;end
    if saveRaw,save(fullfile(outputDirectory,row.name+".mat"),'result');end
    rows = [rows;row];
    writetable(struct2table(rows),fullfile(outputDirectory,suite+"-summary.csv"));
    file = fopen(fullfile(outputDirectory,suite+"-summary.json"),'w');
    cleanup = onCleanup(@() fclose(file));
    fprintf(file,'%s\n',jsonencode(rows,PrettyPrint=true));
    fprintf('%s: completed=%d, duration=%.2f s, gap=%.6g m, recovered=%d, %s\n', ...
        row.name,row.completed,row.executedSeconds,row.minimumBodyGap, ...
        row.recoveredCruise,row.failureIdentifier);
end
