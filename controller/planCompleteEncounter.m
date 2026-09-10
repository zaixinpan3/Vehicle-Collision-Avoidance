function [model,prediction,qp,result,check,timing] = planCompleteEncounter(model)
%planCompleteEncounter Search a complete witness beyond the initial planning window.
% A partial safe prefix is never executable as an encounter certificate.
% The accepted finite completion time is retained to prove eventual exit;
% controller.horizonSteps is a search seed, not an exit-time constraint.
    cfg=model.cfg;
    timing=struct("predictionSeconds",0,"formulationSeconds",0, ...
        "solveSeconds",0,"verificationSeconds",0,"attempts",0);
    calls=0;searchTimer=tic;
    while true
        if timing.attempts>0 && toc(searchTimer)>=cfg.solver.certificateSearchTimeLimit
            error("collisionAvoidanceController:certificateSearchLimit", ...
                "The numerical search budget expired without a complete witness; infeasibility is not established.");
        end
        timing.attempts=timing.attempts+1;
        model.exitSteps=repmat(model.horizonSteps,numel(model.encounters),1);
        model.exitMargin=inf;
        phase=tic;
        prediction=ltvBicycleModel.finitePredict(model,[]);
        timing.predictionSeconds=timing.predictionSeconds+toc(phase);
        anchor=localRateLimitedAnchor(reshape(prediction.referencePlan,2,[]),model);
        phase=tic;
        try
            [model,prediction,anchor]=localPlanningWindow(model,prediction,anchor);
            qp=formulateAvoidanceProblem(model,prediction,anchor);
        catch exception
            if any(string(exception.identifier)==["collisionAvoidanceController:roadBoundaryCoverageGap", ...
                    "collisionAvoidanceController:unsupportedReferenceJump"])
                error("collisionAvoidanceController:noCertifiedContinuation","%s",exception.message);
            end
            rethrow(exception);
        end
        timing.formulationSeconds=timing.formulationSeconds+toc(phase);
        phase=tic;[result,qp]=solveHardCbfClf(qp,cfg);
        calls=calls+result.solverCalls;
        timing.solveSeconds=timing.solveSeconds+toc(phase);
        phase=tic;check=certifyAvoidancePlan(qp,prediction,model,result.decision);
        timing.verificationSeconds=timing.verificationSeconds+toc(phase);
        if result.feasible && check.accepted
            result.solverCalls=calls;
            return;
        end
        if isempty(model.encounters) || ~ismember(result.exitFlag,[-2,0,-7])
            localReject(result,check);
        end
        % A rejected terminal time need not reject a safe prefix. This probe
        % only determines whether further extension is worth constructing.
        % It is never returned to the controller or counted as safe admission.
        probe=qp;
        keep=true(size(probe.physicalBound));keep(probe.barrier.completionRows)=false;
        probe.inequalityMatrix=probe.inequalityMatrix(keep,:);
        probe.physicalBound=probe.physicalBound(keep);
        probe.inequalityBound=probe.barrier.baseBound(keep);
        probe.safetyRows=probe.safetyRows(keep);
        probe.barrier.baseBound=probe.barrier.baseBound(keep);
        probe.barrier.scale=probe.barrier.scale(keep);
        probe.barrier.completionRows=zeros(0,1);
        probe.requiredMargin=0;
        probe.certifiedInfeasible=any(probe.inequalityBound(~any(probe.inequalityMatrix,2))<0);
        probe.stageProgram=avoidanceStageQp(probe);
        phase=tic;[prefix,probe]=solveHardCbfClf(probe,cfg);
        calls=calls+prefix.solverCalls;
        timing.solveSeconds=timing.solveSeconds+toc(phase);
        phase=tic;prefixCheck=certifyAvoidancePlan(probe,prediction,model,prefix.decision);
        timing.verificationSeconds=timing.verificationSeconds+toc(phase);
        if ~prefix.feasible || ~prefixCheck.accepted
            localReject(result,check);
        end
        model.horizonSteps=model.horizonSteps+1;
    end
end

function localReject(result,check)
    error("collisionAvoidanceController:noCertifiedContinuation", ...
        "No complete encounter witness was certified: %s; %s.", ...
        result.message,strjoin(check.failedConditions,","));
end

function anchor = localRateLimitedAnchor(inputs,model)
% Seed the nonlinear prediction with controls inside the actuator envelope.
% This initializes optimization only; it is never an executable fallback.
    cfg = model.cfg;
    lower = [-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum];
    upper = [cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum];
    change = model.sampleTime*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
    prior = model.previousInput;
    for stage = 1:size(inputs,2)
        inputs(:,stage) = min(max(inputs(:,stage),max(lower,prior-change)),min(upper,prior+change));
        prior = inputs(:,stage);
    end
    anchor = inputs(:);
end


function [model, prediction, anchor] = localPlanningWindow(model, prediction, anchor)
% Require road coverage of the entire retained certificate.
    cfg = model.cfg;
    if nargin<3,anchor = prediction.referencePlan;end
    if isempty(model.road.boundaries),return;end
    count = prediction.stageCount;
    [frames,nominal] = laneGeometry.sweptCellFrames(model,prediction.cells,anchor);
    for index = 1:numel(prediction.cells)
        tube = prediction.cells(index);
        frame = frames(index);
        covered = true;
        for boundary = model.road.boundaries(:).'
            direction = boundary.longitudinalDirection;
            stations = direction.'*frame.tangent*[frame.stationLower,frame.stationUpper];
            extent = abs(direction.'*frame.lateral)*cfg.model.lateralDomainRadius ...
                +hypot(cfg.vehicle.length/2,cfg.vehicle.width/2) ...
                +abs(direction).'*frame.positionErrorBound;
            range = [min(stations)-extent,max(stations)+extent] ...
                +direction.'*(frame.origin-boundary.origin);
            covered = covered && range(1)>=boundary.parameterRange(1) ...
                && range(2)<=boundary.parameterRange(2);
        end
        if ~covered
            count = tube.stage-1;
            break;
        end
    end
    if count < prediction.stageCount
        error("collisionAvoidanceController:roadBoundaryCoverageGap", ...
            "The complete retained deadline must lie inside certified road coverage.");
    else
        prediction.geometryAnchor = anchor;
        prediction.geometryFrames = frames;
        prediction.geometryNominal = nominal;
    end
end
