function rows = analyzeControllerRecoveryFailures(inputDirectory, outputDirectory)
%analyzeControllerRecoveryFailures Offline counterfactual failure-frame audit.
% Removed uncertainty/road/target constraints diagnose feasibility only. No
% counterfactual decision is sent to a plant or claimed to be a safe policy.
    arguments
        inputDirectory (1,1) string
        outputDirectory (1,1) string
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'),fullfile(root,'estimator'));
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    rows = struct([]);
    names = ["straight_oncoming","circular_oncoming", ...
        "estimated_straight_oncoming","estimated_circular_oncoming"];
    for name = names
        loaded = load(fullfile(inputDirectory,name+'.mat'),'result');
        source = loaded.result;
        variants = ["fresh","noTarget","noRoadRows","noRoad", ...
            "predictedSuccessor","zeroEgoBounds","zeroTargetBounds", ...
            "zeroAllBounds","exactTargetMotion","zeroTargetYawBound", ...
            "zeroTargetVelocityBound","zeroTargetVelocityYawBounds", ...
            "zeroTargetVelocityAccelerationBounds","tightVelocityBox"];
        for variant = variants
            row = struct('scenario',name,'variant',variant,'time',source.failure.time, ...
                'feasible',false,'exitFlag',NaN,'identifier',"",'message',"", ...
                'horizonSteps',NaN,'nativeSolves',NaN,'restorationSolves',NaN, ...
                'finalRestorationSlack',NaN,'targetPositionRadius1s',NaN, ...
                'targetSpeedLower',NaN,'targetSpeedUpper',NaN, ...
                'targetCourseWidth',NaN,'elapsedSeconds',NaN);
            model = [];program = [];search = [];solve = [];
            timer = tic;
            try
                model = localModel(source,variant);
                [program,~,~] = formulateAvoidanceProblem(model);
                if isfield(program,'jointCertificate')
                    [program,solve,search] = solveHardCbfClf.fixedDirections(program,model,model.cfg);
                    row.nativeSolves = search.nativeSolves;
                    row.restorationSolves = search.restorationSolves;
                    if isfield(search,'restoration') && ~isempty(search.restoration.slackHistory)
                        row.finalRestorationSlack = search.restoration.slackHistory(end);
                    end
                else
                    solve = solveHardCbfClf.constrained(program,model.cfg);
                end
                row.feasible = solve.feasible;row.exitFlag = solve.exitFlag;
                row.message = string(solve.message);
                row.horizonSteps = program.layout.horizonSteps;
                if ~isempty(model.encounter)
                    [~,radius] = targetPrediction.finiteFlow(model.encounter,1);
                    row.targetPositionRadius1s = norm(radius(1:2));
                    row.targetSpeedLower = model.encounter.parameters.speed(1);
                    row.targetSpeedUpper = model.encounter.parameters.speed(2);
                    row.targetCourseWidth = diff(model.encounter.parameters.course);
                end
            catch exception
                row.identifier = string(exception.identifier);
                row.message = string(exception.message);
            end
            row.elapsedSeconds = toc(timer);
            rows = [rows;row]; %#ok<AGROW>
            save(fullfile(outputDirectory,name+'_'+variant+'.mat'),'row','model','program','search','solve');
            writetable(struct2table(rows),fullfile(outputDirectory,'counterfactuals.csv'));
            fprintf('%s / %s: feasible=%d, H=%g, %s\n',name,variant,row.feasible,row.horizonSteps,row.identifier);
        end
    end
    localPlantAudit(inputDirectory,outputDirectory);
    localInitializationAudit(inputDirectory,outputDirectory);
    localReferenceAudit(inputDirectory,outputDirectory);
end

function localInitializationAudit(inputDirectory,outputDirectory)
    rows = cell(0,10);
    for name = ["estimated_straight_oncoming","estimated_circular_oncoming"]
        loaded = load(fullfile(inputDirectory,name+'.mat'),'result');source = loaded.result;
        target = source.failureContext.targetEstimate;
        ego = source.attempts.controllerEgoEstimate{end};
        rows(end+1,:) = {name,source.failure.time,ego.targetAcquisitionTime, ...
            target.measurementHistoryEnclosure.samples,target.targetSpeed, ...
            target.targetSpeedErrorBound,target.targetCourseErrorBound, ...
            target.targetYawErrorBound,ego.egoYawErrorBound, ...
            source.estimator.initialization.observerSamplePeriod}; %#ok<AGROW>
    end
    writetable(cell2table(rows,VariableNames={'Scenario','FailureTimeS','AcquisitionTimeS', ...
        'HistorySamples','EstimatedSpeedMps','PublishedSpeedRadiusMps', ...
        'PublishedCourseRadiusRad','PublishedYawRadiusRad','EgoYawRadiusRad', ...
        'ObserverSamplePeriodS'}),fullfile(outputDirectory,'initialization.csv'));
end

function localPlantAudit(inputDirectory,outputDirectory)
    rows = cell(0,17);flags = cell(0,5);
    for name = ["straight_oncoming","circular_oncoming"]
        loaded = load(fullfile(inputDirectory,name+'.mat'),'result');source = loaded.result;
        metadata = source.attempts.metadata(1:end-1);cfg = source.controllerConfiguration;
        tire = modifiedFialaTire.parameters(cfg);
        flags(end+1,:) = {name,numel(metadata),sum(cellfun(@(m)m.readmittedAfterRoadRefit,metadata)), ...
            sum(cellfun(@(m)m.readmittedAfterInconsistentObservation,metadata)), ...
            sum(cellfun(@(m)m.inheritedFeasibleFamily,metadata))}; %#ok<AGROW>
        for index = 1:numel(metadata)
            command = source.command{index};state = source.controlState(index,:).';
            slip = atan2([state(5)+cfg.vehicle.lf*state(6);state(5)-cfg.vehicle.lr*state(6)],state(4)) ...
                -[command.actuatorInput(1);0];
            force = modifiedFialaTire.evaluate(slip,command.brakingRatio,cfg);
            capacity = tire.longitudinalForceScale*sqrt(1-command.brakingRatio^2);
            predicted = metadata{index}.predictedNextState;
            successor = source.controlState(index+1,:).';
            rows(end+1,:) = {name,source.controlTime(index),command.actuatorInput(1), ...
                command.brakingRatio,slip(1),command.axleLateralForce(1),force(1),capacity(1), ...
                abs(command.axleLateralForce(1))/capacity(1),predicted(5),successor(5), ...
                predicted(6),successor(6),successor(4)-predicted(4), ...
                successor(5)-predicted(5),successor(6)-predicted(6),metadata{index}.horizonSteps}; %#ok<AGROW>
        end
    end
    writetable(cell2table(rows,VariableNames={'Scenario','TimeS','SteerRad','BrakingRatio', ...
        'FrontSlipRad','ReportedAffineFrontForceN','IndependentFialaFrontForceN', ...
        'StaticCombinedSlipCapacityN','AffineForceCapacityRatio','PredictedVyMps','ActualVyMps', ...
        'PredictedYawRateRadps','ActualYawRateRadps','VxPredictionErrorMps', ...
        'VyPredictionErrorMps','YawRatePredictionErrorRadps','HorizonSteps'}), ...
        fullfile(outputDirectory,'plant-dynamics.csv'));
    writetable(cell2table(flags,VariableNames={'Scenario','ExecutedHolds','RoadRefits', ...
        'ExplicitInconsistentReadmissions','InheritedFamilies'}),fullfile(outputDirectory,'transfer-flags.csv'));
end

function localReferenceAudit(inputDirectory,outputDirectory)
    loaded = load(fullfile(inputDirectory,'varying_curvature_oncoming.mat'),'result');
    source = loaded.result;context = source.failureContext;cfg = source.controllerConfiguration;
    cfg.solver.frameDeadlineSeconds = Inf;cfg.solver.certificateSearchTimeLimit = 30;
    station = (0:.1:220).';curvature = .01*sin(2*pi*station/80);curvature(station>80) = 0;
    curve = struct('origin',[0;0],'heading',0,'curvature',0,'length',220, ...
        'curvatureProfile',[station,curvature],'continuation',"constantCurvature");
    rows = cell(0,5);
    for variant = ["originalPolyline","curvatureProfile","curvatureProfileNoRoad","profileWithLegacyTarget"]
        road = context.controllerRoadGeometry;target = [];
        if variant~="originalPolyline",road.referenceCurve = curve;end
        if variant == "curvatureProfileNoRoad"
            road.boundaries = [];road.lateralClearance = [];
        elseif variant == "profileWithLegacyTarget"
            % Interface probe only: expose the constructed target at t=0.
            target = source.attempts.targetTruth{1};
        end
        success = false;identifier = "";message = "";elapsed = tic;
        try
            collisionAvoidanceController(context.controllerState,target,road,cfg,[]);
            success = true;
        catch exception
            identifier = string(exception.identifier);message = string(exception.message);
        end
        rows(end+1,:) = {variant,success,identifier,message,toc(elapsed)}; %#ok<AGROW>
        writetable(cell2table(rows,VariableNames={'Variant','FirstFrameAccepted','Identifier','Message','Seconds'}), ...
            fullfile(outputDirectory,'reference-interface.csv'));
        fprintf('Reference %s: accepted=%d %s\n',variant,success,identifier);
    end
end

function model = localModel(source,variant)
    cfg = source.controllerConfiguration;
    cfg.solver.frameDeadlineSeconds = Inf;
    cfg.solver.certificateSearchTimeLimit = 30;
    cfg.solver.workTimer = tic;cfg.solver.workTimeLimit = 30;
    context = source.failureContext;
    [ego,lane,road,target] = readPlanningInputs(context.controllerState, ...
        context.targetEstimate,context.controllerRoadGeometry,cfg);
    if variant == "noTarget",target = [];end
    if any(variant == ["noRoadRows","noRoad"]),road.boundaries = [];end
    if variant == "noRoad",road.lateralClearance = [];end
    if variant == "predictedSuccessor"
        previous = source.attempts.metadata{end-1};
        x = previous.predictedNextState;
        [ego.position,ego.yaw] = laneGeometry.fromFrenet(x,lane);
        ego.modelState = [ego.position;ego.yaw;x(4:6)];
    end
    if any(variant == ["zeroEgoBounds","zeroAllBounds"]),ego.stateErrorBound(:) = 0;end
    if any(variant == ["zeroTargetBounds","zeroAllBounds"])
        for field = ["positionErrorBound","velocityErrorBound","accelerationErrorBound", ...
                "predictionAccelerationErrorBound","yawErrorBound","yawRateErrorBound"]
            target.(field)(:) = 0;
        end
        target.parameterErrorBounds = [];
    elseif variant == "exactTargetMotion"
        % Keep the published position uncertainty; remove motion uncertainty.
        for field = ["velocityErrorBound","accelerationErrorBound", ...
                "predictionAccelerationErrorBound","yawErrorBound","yawRateErrorBound"]
            target.(field)(:) = 0;
        end
        target.parameterErrorBounds = [];
    elseif variant == "zeroTargetYawBound"
        target.yawErrorBound = 0;
    elseif any(variant == ["zeroTargetVelocityBound","zeroTargetVelocityYawBounds", ...
            "zeroTargetVelocityAccelerationBounds"])
        target.velocityErrorBound(:) = 0;
        target.parameterErrorBounds = [];
        if variant == "zeroTargetVelocityYawBounds"
            target.yawErrorBound = 0;
            target.yawRateErrorBound = 0;
        elseif variant == "zeroTargetVelocityAccelerationBounds"
            target.accelerationErrorBound(:) = 0;
            target.predictionAccelerationErrorBound(:) = 0;
        end
    end
    projection = laneGeometry.project(ego.position,lane);
    heading = atan2(sin(ego.yaw-projection.heading),cos(ego.yaw-projection.heading));
    [radius,~] = stateUncertainty.toFrenet(ego.modelState,ego.stateErrorBound,lane);
    model = struct('cfg',cfg,'lane',lane,'road',road,'stateTime',ego.stateTime, ...
        'sampleTime',cfg.controller.sampleTime,'horizonSteps',cfg.controller.horizonSteps, ...
        'referenceSpeed',cfg.referenceSpeed, ...
        'initialEgoState',[projection.station;projection.lateralPosition;heading;ego.modelState(4:6)], ...
        'initialFrenetErrorBound',radius,'longitudinalAccelerationBias',ego.longitudinalAccelerationBias, ...
        'previousInput',ego.heldActuatorInput,'requiredMargin',0,'confirmation',[]);
    identity = struct('configuration',rmfield(cfg,'solver'),'lane',lane,'road',road, ...
        'accelerationBias',ego.longitudinalAccelerationBias);
    [model,~] = hardEncounterBarrier.prepare(model,ego,target,[],identity);
    if variant == "tightVelocityBox"
        center = model.encounter.center(3:4);radius = model.encounter.radius(3:4);
        low = center-radius;high = center+radius;
        speedLow = norm(max(abs(center)-radius,0));
        speedHigh = norm(abs(center)+radius);
        model.encounter.parameters.speed = [max(model.encounter.parameters.speed(1),speedLow); ...
            min(model.encounter.parameters.speed(2),speedHigh)];
        if speedLow>0
            corners = [low(1),low(1),high(1),high(1);low(2),high(2),low(2),high(2)];
            nominal = atan2(center(2),center(1));
            delta = atan2(sin(atan2(corners(2,:),corners(1,:))-nominal), ...
                cos(atan2(corners(2,:),corners(1,:))-nominal));
            model.encounter.parameters.course = [max(model.encounter.parameters.course(1),nominal+min(delta)); ...
                min(model.encounter.parameters.course(2),nominal+max(delta))];
        end
    end
end
