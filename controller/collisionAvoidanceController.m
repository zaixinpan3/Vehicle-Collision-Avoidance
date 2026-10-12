function [command,predictedInput,prediction,controllerState] = ...
        collisionAvoidanceController(egoState,targetEstimate,laneCenterline,cfg,previousState)
%collisionAvoidanceController Two-stage PCBF/CLF optimization; no fallback algorithm.
% Controller state retains a target prediction and a linearization trajectory.
% Every current target estimate updates the prediction; its A and beta are
% held constant within this frame, independently of proof-metadata availability.
% Passing [] as previousState starts a new problem and a new target epoch.
% The target epoch is the latest target estimate and the sample index at
% which it was taken (targetEpochIndex); the forecast runs forward from it,
% because braking that ends at rest cannot be run backward.
% Collision interaction is active inside the configured position radius.
% A complete current sensor scan with no target clears the previous forecast.
% collisionAvoidanceController("warmUp", cfg) runs one synthetic encounter
% (lane following, then a head-on target that appears and is tracked) and
% resets: every code path has then been compiled, and the compiled kernels
% and the straight-road trim loaded, before the first sample. Call it at
% start-up; the first sample would otherwise take about a second.
    persistent lastState
    if nargin<=2 && (ischar(egoState) || isstring(egoState))
        action=string(egoState);
        if ~isscalar(action) || ~any(action==["resetNominalTrajectory","warmUp"])
            error('collisionAvoidanceController:invalidAction','Use resetNominalTrajectory or warmUp.');
        end
        if action=="warmUp"
            if nargin<2,targetEstimate=[];end
            localWarmUp(targetEstimate);
        end
        lastState=[];command=[];predictedInput=[];prediction=[];controllerState=[];return;
    end
    explicitState=nargin>=5;
    if ~explicitState,previousState=lastState;end
    if isstruct(previousState) && (~isfield(previousState,'version') || previousState.version~=79),previousState=[];end
    if nargin<4,cfg=[];end
    if nargin<3,laneCenterline=[];end
    if nargin<2,targetEstimate=[];end
    timer=tic;cfg=collisionAvoidanceControllerConfig(cfg);
    [ego,lane,road,observation]=readControllerInputs(egoState,targetEstimate,laneCenterline,cfg);
    if isempty(targetEstimate) && isfield(egoState,'targetEstimate'),targetEstimate=egoState.targetEstimate;end
    observed=predictiveSafetyGeometry.target(observation,targetEstimate,cfg);
    uncertainty=predictiveSafetyGeometry.observerUncertainty(ego,observation,observed);
    frame=predictiveSafetyGeometry.roadFrame(lane,road);
    context={cfg.vehicle,cfg.tire,cfg.roadLoad,cfg.model,cfg.actuation,cfg.collision, ...
        cfg.controller,cfg.referenceSpeed,cfg.clf,cfg.nominalClf,cfg.initialization, ...
        cfg.nonlinear.integrationStep,cfg.terminal,frame,lane};
    index=0;epochTime=ego.stateTime;targetEpoch=observed;targetEpochIndex=0;
    if isstruct(previousState)
        if ~isequaln(context,previousState.context)
            error('collisionAvoidanceController:changedContinuationProblem', ...
                'Model, constraints or reference path changed. Pass an empty previousState to initialize a new problem.');
        end
        index=previousState.sampleIndex+1;epochTime=previousState.epochTime;targetEpoch=previousState.targetEpoch;
        targetEpochIndex=previousState.targetEpochIndex;
        expectedTime=epochTime+index*cfg.controller.sampleTime;
        if isfinite(ego.stateTime) && isfinite(epochTime) && abs(ego.stateTime-expectedTime)>1e-9*max(1,abs(expectedTime))
            error('collisionAvoidanceController:invalidSampleTime','Continuation requires consecutive absolute sample times.');
        end
    end
    if isempty(observed) && isfield(egoState,'perception')
        perception=egoState.perception;
        if isstruct(perception) && isscalar(perception) ...
                && all(isfield(perception,{'time','completeWithinRange'})) ...
                && isequal(perception.completeWithinRange,true) && isequal(perception.time,ego.stateTime)
            targetEpoch=[];
        end
    end
    q=predictiveSafetyGeometry.predictTarget(targetEpoch,(index-targetEpochIndex)*cfg.controller.sampleTime);
    if ~isempty(observed) && ~isempty(q)
        observed(3)=q(3)+atan2(sin(observed(3)-q(3)),cos(observed(3)-q(3)));
    end
    % A target that the previous plan did not know of (onset): its shifted
    % plan is not a useful anchor, and the problem starts from a fresh rollout.
    targetOnset=~isempty(observed) && isstruct(previousState) && isempty(previousState.targetEpoch);
    if ~isempty(observed)
        % Reanchor the prediction at the current estimate without resetting
        % the ego input warm start. Cross-frame constancy is not required.
        targetEpoch=observed;targetEpochIndex=index;
        q=observed;
    end
    previous=zeros(2,1);
    if ~isempty(ego.heldActuatorInput),previous=ego.heldActuatorInput;
    elseif isstruct(previousState),previous=previousState.appliedInput;
    elseif isfinite(cfg.model.brakingRatioRateMaximum)
        error('collisionAvoidanceController:missingInputMemory','Finite slew limits require the previous applied input.');
    end
    jointState=ego.modelState;targetParameters=zeros(7,0);
    if ~isempty(q),jointState=[jointState;q(1:4)];targetParameters=q(5:11);end
    nominalReference=nonlinearBicycleModel.cruise(cfg,frame(4));
    model=struct('cfg',cfg,'initialState',ego.modelState,'jointState',jointState, ...
        'targetParameters',targetParameters,'previousInput',previous,'target',q,'targetEpoch',targetEpoch, ...
        'targetEpochIndex',targetEpochIndex,'targetOnset',targetOnset,'sampleIndex',index,'epochTime',epochTime,'lane',lane,'road',road,'frame',frame, ...
        'nominalReference',nominalReference,'stateTime',ego.stateTime, ...
        'uncertainty',uncertainty);
    [solution,search,model]=solvePredictiveControl(model,previousState,timer);
    if isempty(solution)
        error('collisionAvoidanceController:noOptimizationSolution', ...
            'The two-stage affine optimization did not complete (%s).',search.terminationReason);
    end
    predictedInput=solution.inputs;first=predictedInput(:,1);
    command=localCommand(first,ego.modelState,cfg);
    command.measurementTime=ego.stateTime;command.actuationTime=ego.stateTime;command.holdSeconds=cfg.controller.sampleTime;
    predictedJointState=solution.states;
    if ~isempty(q)
        targetStates=zeros(4,size(solution.states,2));
        for step=1:size(targetStates,2)
            future=predictiveSafetyGeometry.predictTarget(targetEpoch,(index-targetEpochIndex+step-1)*cfg.controller.sampleTime);
            targetStates(:,step)=future(1:4);
        end
        predictedJointState=[predictedJointState;targetStates];
    end
    metadata=struct('optimizationReturned',search.returned,'predictiveBarrierValue',solution.safety, ...
        'zeroSlack',solution.safety<=cfg.solver.lexicographicTieTolerance+cfg.solver.feasibilityTolerance,'safetyScope',"affineSampledConstraints", ...
        'predictionModel',"nonlinearPredictionOfUnmodifiedOptimizedInputs",'affineValidationPerformed',false,'nonlinearValidationPerformed',false, ...
        'recursiveFeasibilityScope',"idealMathematicalProblemOnly", ...
        'hasTarget',~isempty(q),'targetMotionModel',"constantTangentialAccelerationAndSideslip", ...
        'jointStateDimension',numel(jointState), ...
        'safetyObjective',"sumOfPrefixSafetySlacks",'primaryOptimumComputed',true, ...
        'safetyValueInterpretation',"optimizedAffineSlackSum", ...
        'secondaryObjective',"minimumClfSlackWithBoundedProximalTie", ...
        'secondaryOptimumApplied',true, ...
        'tertiaryObjective',"none",'clfTieTolerance',cfg.solver.clfTieTolerance, ...
        'preplannedAvoidanceTrajectoryRequired',false,'drivingModeSwitching',false, ...
        'clfSlack',solution.clfSlack,'clfInitialValue',solution.clfInitialValue, ...
        'clfNextValue',solution.clfNextValue,'clfSatisfied',search.clfLowerBound,'clfContraction',nominalReference.contraction,'clfConvergenceTimeConstantSeconds',cfg.clf.convergenceTimeConstantSeconds, ...
        'clfFunction',solution.clfFunction,'clfRequiredDecrease',solution.clfRequiredDecrease, ...
        'clfTieBound',solution.clfTieBound,'clfLocalModel',"quadraticOnAffineFirstSuccessor", ...
        'nominalTrackingMatrix',nominalReference.matrix,'clfReferenceState',nominalReference.state, ...
        'clfOperatingInput',nominalReference.input,'clfOperatingCurvature',frame(4), ...
        'controlSource',search.source,'solverAlgorithm',"budgetedConicQp", ...
        'nonlinearPredictionEvaluated',search.rollout.available, ...
        'solverCallCount',search.solverCalls,'search',search,'hardConstraintResidual',solution.hard, ...
        'optimizationConverged',search.converged,'affineFeasibilityTolerance',cfg.solver.feasibilityTolerance,'minimumCollisionMargin',solution.minimumCollisionMargin, ...
        'roadConstraintsEnforced',~isempty(road.lateralClearance),'roadConstraintScope',"rectangleAtEveryPredictedNode", ...
        'horizonSteps',size(predictedInput,2), ...
        'prefixSteps',cfg.controller.horizonSteps,'completionSteps',size(predictedInput,2)-cfg.controller.horizonSteps, ...
        'absoluteSampleIndex',index,'endpointIndex',index+size(predictedInput,2), ...
        'terminalSet',"brakingContinuationEncounterSafe", ...
        'terminalValue',solution.terminalValue,'terminalMarginMeters',solution.terminalMarginMeters, ...
        'terminalStopSeconds',solution.terminalStopSeconds, ...
        'terminalExitSeconds',solution.terminalExitSeconds,'terminalEnd',solution.terminalEnd, ...
        'terminalDistanceMeters',solution.terminalDistanceMeters, ...
        'terminalRoadMarginMeters',solution.terminalRoadMarginMeters, ...
        'postSolveInputModification',false,'postSolveFeasibilityCheck',false, ...
        'anchorCertified',search.anchorCertified,'appendedTerminalSteps',search.appendedTerminalSteps, ...
        'appendedBrakingSteps',search.appendedBrakingSteps,'appendedGuidedSteps',search.appendedGuidedSteps, ...
        'extensionStopped',search.extensionStopped, ...
        'linearizationGapMeters',solution.linearizationGap, ...
        'encounterRangeMeters',cfg.collision.encounterRangeMeters,'encounterExitStep',solution.encounterExit, ...
        'predictedNextState',solution.states(:,2),'initialEgoState',ego.modelState,'runtimeSeconds',toc(timer));
    metadata.trust=search.trust;
    if ~search.rollout.available,metadata.predictionModel="affinePredictionNonlinearRolloutUnavailable";end
    metadata.uncertaintyIncluded=uncertainty.specified;
    metadata.commonPoseCancelled=uncertainty.relativeFrame;
    metadata.targetUncertaintyModel="constantParametersRestartedEachHoldFromCurrentObserverSet";
    metadata.egoUncertaintyModel="currentPosteriorEnclosurePropagatedThroughEachHold";
    metadata.robustNonlinearSafetyCertified=false;
    metadata.maximumCollisionTighteningMeters=solution.maximumCollisionTighteningMeters;
    metadata.clfWorstNextValue=solution.clfWorstNextValue;
    metadata.clfCurrentBudget=solution.clfCurrentBudget;
    if uncertainty.specified
        metadata.safetyScope="observerTightenedAffineSampledConstraints";
    end
    if model.robustnessRelaxation
        metadata.safetyObjective="sumOfRobustnessDeficitsWithHardNominalCollisionRows";
        metadata.safetyScope="affineNominalCollisionWithRelaxedObserverMargins";
    end
    controllerState=struct('version',79,'trust',model.trust,'stateTime',ego.stateTime,'sampleTime',cfg.controller.sampleTime, ...
        'sampleIndex',index,'epochTime',epochTime,'targetEpoch',targetEpoch,'targetEpochIndex',targetEpochIndex,'context',{context}, ...
        'jointState',jointState,'predictedJointState',predictedJointState, ...
        'appliedInput',first,'inputTrajectory',predictedInput,'stateTrajectory',solution.states, ...
        'stageSlacks',solution.stageSlacks,'safetyBudget',solution.safety, ...
        'target',q,'uncertaintySpecified',uncertainty.specified);
    prediction=struct('problemClass',"jointPredictiveCbfClf",'model',model, ...
        'inputTrajectory',predictedInput,'predictedState',solution.states,'predictedJointState',predictedJointState, ...
        'solution',solution,'metadata',metadata);
    if ~explicitState,lastState=controllerState;end
end

function command=localCommand(input,state,cfg)
    tire=modifiedFialaTire.parameters(cfg);force=modifiedFialaTire.longitudinalForce(input(2),cfg);
    slip=atan2([state(5)+cfg.vehicle.lf*state(6);state(5)-cfg.vehicle.lr*state(6)],state(4))-[input(1);0];
    lateral=modifiedFialaTire.evaluate(slip,input(2),cfg);dx=nonlinearBicycleModel.derivative(state,input,cfg);
    [road,~,components]=nonlinearBicycleModel.roadLoad(state(4),cfg);
    rolling=components.rollingResistanceForce*tire.staticNormalLoad/(cfg.vehicle.m*cfg.vehicle.gravity);
    command=struct('actuatorInput',input,'frontWheelSteeringAngle',input(1),'brakingRatio',input(2), ...
        'actuatorInputOrder',["frontWheelSteeringAngle","brakingRatio"], ...
        'longitudinalAcceleration',modifiedFialaTire.accelerationGain(cfg)*input(2), ...
        'bodyLongitudinalVelocityDerivative',dx(4),'lateralAcceleration',dx(5),'yawAcceleration',dx(6), ...
        'totalLongitudinalActuatorForce',sum(force),'totalLongitudinalTireForce',sum(force-rolling), ...
        'axleLongitudinalTireForce',force-rolling,'axleLongitudinalForce',force, ...
        'axleLateralForce',lateral,'axleNormalLoad',tire.staticNormalLoad,'tireSideslipAngle',slip, ...
        'aerodynamicResistanceForce',components.aerodynamicForce, ...
        'rollingResistanceForce',components.rollingResistanceForce,'totalRoadLoadForce',road);
end

function localWarmUp(cfg)
    % One synthetic encounter on a straight road with an explicit state:
    % four target-free samples, then a head-on target 40 m ahead at 8 m/s
    % that appears and is tracked for one second. A sample without a
    % solution ends it early; what it reached has been compiled.
    cfg=collisionAvoidanceControllerConfig(cfg);h=cfg.controller.sampleTime;
    road=struct('centerline',[-100,0;1000,0],'lateralClearance',[8.5344;12.192]);
    x=[0;0;0;cfg.referenceSpeed;0;0];input=[0;0];state=[];
    q=[40;0;pi;8;0;0;cfg.target.rearAxleDistance;cfg.target.defaultLength/2;cfg.target.defaultWidth/2;0;0];
    for frame=1:24
        target=[];
        if frame>4
            p=predictiveSafetyGeometry.predictTarget(q,(frame-5)*h);
            target=struct('targetPositionInertial',p(1:2),'targetVelocityInertial',p(4)*[cos(p(3));sin(p(3))], ...
                'targetYawInertial',p(3));
        end
        ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6), ...
            'heldActuatorInput',input,'stateTime',(frame-1)*h);
        try
            [command,~,~,state]=collisionAvoidanceController(ego,target,road,cfg,state);
        catch exception
            if ~startsWith(string(exception.identifier),"collisionAvoidanceController:"),rethrow(exception);end
            return;
        end
        input=command.actuatorInput;x=nonlinearBicycleModel.sample(x,input,cfg);
    end
end
