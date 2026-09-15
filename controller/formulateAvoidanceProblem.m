function [program,prediction,clf] = formulateAvoidanceProblem(model)
%formulateAvoidanceProblem One SOCP with hard safety and a soft sampled CLF.
% Decision order: held steering, held braking ratio, nonnegative CLF slack.
    cfg = model.cfg;
    cruise = ltvBicycleModel.sampledCruise(model);
    model.anchorPlan = cruise.input;
    prediction = ltvBicycleModel.fixedPredict(model,cruise.input,cruise.stage);
    % Geometry is affine in the actual held input, not a fixed rollout.
    prediction = rmfield(prediction,'fixedInputs');
    geometryModel = model;
    geometryModel.encounters = struct("key",{},"radius",{},"contract",{});
    geometry = avoidanceSafetyGeometry.build(geometryModel,prediction);
    % The obstacle barrier module supplies all obstacle constraints. Permanent
    % road, chart, domain and slip rows remain for both target/no-target cases.
    permanent = ~startsWith(geometry.label,"collision:");
    [cbfMatrix,cbfBound,barrier] = hardEncounterBarrier.rows(model,prediction,geometry.frames);
    lower = [-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum];
    upper = [cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum];
    rate = model.sampleTime*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
    lower = max(lower,model.previousInput-rate);
    upper = min(upper,model.previousInput+rate);
    matrix = [geometry.matrix(permanent,:);cbfMatrix;eye(2);-eye(2)];
    bound = [geometry.physicalBound(permanent);cbfBound;upper;-lower];
    reach = [cfg.model.frontWheelSteeringAngleMaximum; ...
        max(abs([cfg.actuation.brakingRatioMinimum,cfg.actuation.brakingRatioMaximum]))];
    scale = 1+abs(bound)+abs(matrix)*reach;
    reserve = 4*max(cfg.encounter.numericalMargin,cfg.solver.constraintTolerance) ...
        *scale.*any(matrix~=0,2);
    bound = bound-reserve;
    % Only the CLF has a slack column. Every physical row keeps coefficient 0.
    matrix = [matrix,zeros(numel(bound),1);0,0,-1];
    bound = [bound;0];
    linearCount = numel(bound);
    root = chol(cruise.matrix);
    trackingError = model.initialEgoState(2:6)-cruise.state(2:6);
    radius = model.initialFrenetErrorBound(2:6);
    contraction = 1-cruise.decayPerHold;
    middle = (contraction+cruise.contraction)/2;
    stateMap = cruise.transition(2:6,2:6);
    inputMap = cruise.transition(2:6,7:8);
    drift = cruise.transition(2:6,:)*[cruise.state;cruise.input;1]-cruise.state(2:6);
    nominalOffset = stateMap*trackingError-inputMap*cruise.input+drift;
    inputRoot = root*inputMap;
    numeric = 100*cfg.solver.constraintTolerance*(1+norm(inputRoot,'fro')*norm(reach));
    coneRadius = sqrt(middle)*norm(root*trackingError)+numeric;
    matrix = [matrix;0,0,-1;-inputRoot,zeros(5,1)];
    bound = [bound;coneRadius;root*nominalOffset];
    disturbance = sqrt(middle)*norm(abs(root)*radius) ...
        +norm(abs(root*stateMap)*radius)+2*numeric;
    clf = struct('cruise',cruise,'initialValue',trackingError.'*cruise.matrix*trackingError, ...
        'decayPerHold',cruise.decayPerHold, ...
        'disturbanceBound',contraction/(contraction-middle)*disturbance^2, ...
        'normDisturbance',disturbance,'youngFactor',contraction/(contraction-middle));
    weight = diag([cfg.clf.frontWheelSteeringAngleWeight,cfg.clf.brakingRatioWeight]);
    objectiveMap = root*inputMap;
    objectiveOffset = root*nominalOffset;
    hessian = 2*blkdiag(objectiveMap.'*objectiveMap+weight,cfg.clf.relaxationWeight);
    linear = [2*(objectiveMap.'*objectiveOffset-weight*cruise.input);0];
    program = struct('P',sparse(hessian),'q',linear,'A',sparse(matrix),'b',bound, ...
        'cones',[0;linearCount;6],'decisionRadius',reach, ...
        'obstacleCbfRowCount',numel(cbfBound),'barrier',barrier);
end
