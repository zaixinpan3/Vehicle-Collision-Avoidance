function [command,predictedInput,prediction,controllerState] = ...
        collisionAvoidanceController(egoState,targetEstimate,laneCenterline,cfg,previousState)
%collisionAvoidanceController RTI PCBF/CLF control with adaptive horizon search.
% Controller state retains a target prediction and a linearization trajectory.
% Every current target estimate updates the prediction; its A and beta are
% held constant within this frame, independently of proof-metadata availability.
% Passing [] as previousState starts a new problem and a new target epoch.
% The target epoch is the latest target estimate and the sample index at
% which it was taken (targetEpochIndex); the forecast runs forward from it,
% because braking that ends at rest cannot be run backward.
% Collision interaction is active inside the configured position radius.
% A complete current sensor scan with no target clears the previous forecast.
% collisionAvoidanceController("warmUp", cfg, road) primes exact cruise and
% bounded low-speed opposing-target probes, then resets its state.
% The optional road also primes its actual curvature and avoidance geometry.
% It loads the compiled kernels and straight-road trim before timed samples.
% Integrations with observer bounds also prime their bounded-input path.
% Startup preparation is separate from the online computation budget.
    persistent lastState
    if nargin<=3 && (ischar(egoState) || isstring(egoState))
        action=string(egoState);
        if ~isscalar(action) || ~any(action==["resetNominalTrajectory","warmUp"])
            error('collisionAvoidanceController:invalidAction','Use resetNominalTrajectory or warmUp.');
        end
        if action=="warmUp"
            if nargin<2,targetEstimate=[];end
            if nargin<3,laneCenterline=[];end
            localWarmUp(targetEstimate,laneCenterline);
        end
        lastState=[];command=[];predictedInput=[];prediction=[];controllerState=[];return;
    end
    explicitState=nargin>=5;
    if ~explicitState,previousState=lastState;end
    if isstruct(previousState) && (~isfield(previousState,'version') || previousState.version~=92),previousState=[];end
    if nargin<4,cfg=[];end
    if nargin<3,laneCenterline=[];end
    if nargin<2,targetEstimate=[];end
    timer=tic;cfg=collisionAvoidanceControllerConfig(cfg);
    [ego,lane,road,observation]=readControllerInputs(egoState,targetEstimate,laneCenterline,cfg);
    if isempty(targetEstimate) && isfield(egoState,'targetEstimate'),targetEstimate=egoState.targetEstimate;end
    observed=predictiveSafetyGeometry.target(observation,targetEstimate,cfg);
    uncertainty=predictiveSafetyGeometry.observerUncertainty(ego,observation,observed);
    observedOutside=false;
    if ~isempty(observed)
        reserve=sum(vecnorm(uncertainty.collisionGenerator(1:2,:)));
        if ~isempty(uncertainty.target),reserve=reserve+uncertainty.target.positionRadius;end
        observedOutside=norm(observed(1:2)-ego.modelState(1:2))-reserve>cfg.collision.encounterRangeMeters;
        if observedOutside,observed=[];end
    end
    frame=predictiveSafetyGeometry.roadFrame(lane,road);
    context={cfg.vehicle,cfg.tire,cfg.roadLoad,cfg.model,cfg.actuation,cfg.collision, ...
        cfg.controller,cfg.referenceSpeed,cfg.clf,cfg.nominalClf,cfg.rti,cfg.planner, ...
        cfg.nonlinear.integrationStep,cfg.terminal,frame,lane};
    index=0;epochTime=ego.stateTime;targetEpoch=observed;targetEpochIndex=0;
    encounterId=double(~isempty(observed));
    if isstruct(previousState)
        if ~isequaln(context,previousState.context)
            error('collisionAvoidanceController:changedContinuationProblem', ...
                'Model, constraints or reference path changed. Pass an empty previousState to initialize a new problem.');
        end
        index=previousState.sampleIndex+1;epochTime=previousState.epochTime;targetEpoch=previousState.targetEpoch;
        targetEpochIndex=previousState.targetEpochIndex;
        encounterId=previousState.encounterId;
        expectedTime=epochTime+index*cfg.controller.sampleTime;
        if isfinite(ego.stateTime) && isfinite(epochTime) && abs(ego.stateTime-expectedTime)>1e-9*max(1,abs(expectedTime))
            error('collisionAvoidanceController:invalidSampleTime','Continuation requires consecutive absolute sample times.');
        end
    end
    if observedOutside,targetEpoch=[];end
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
    % A newly observed encounter requests an avoidance reference instead of
    % inheriting a cruise seed built while no target was present.
    targetOnset=~isempty(observed) && isstruct(previousState) && isempty(previousState.targetEpoch);
    if targetOnset,encounterId=encounterId+1;end
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
            'RTI did not produce a completed optimization within its budget (%s; %s).', ...
            search.terminationReason,search.stopReason);
    end
    predictedInput=solution.inputs;first=predictedInput(:,1);
    command=localCommand(first,ego.modelState,cfg);
    command.measurementTime=ego.stateTime;command.actuationTime=ego.stateTime;command.holdSeconds=cfg.controller.sampleTime;
    predictedJointState=solution.states;
    if ~isempty(q)
        future=predictiveSafetyGeometry.predictTarget(targetEpoch, ...
            (index-targetEpochIndex+(0:size(solution.states,2)-1))*cfg.controller.sampleTime);
        targetStates=future(1:4,:);
        predictedJointState=[predictedJointState;targetStates];
    end
    metadata=struct('optimizationReturned',search.returned,'predictiveBarrierValue',solution.safety, ...
        'zeroSlack',solution.safety<=cfg.solver.lexicographicTieTolerance+cfg.solver.feasibilityTolerance,'safetyScope',"nonlinearSampledConstraints", ...
        'predictionModel',"nonlinearPredictionOfUnmodifiedOptimizedInputs",'affineValidationPerformed',false,'nonlinearValidationPerformed',solution.nonlinearValidationPerformed, ...
        'recursiveFeasibilityScope',"idealMathematicalProblemOnly", ...
        'hasTarget',~isempty(q),'targetMotionModel',"constantTangentialAccelerationAndSideslip", ...
        'jointStateDimension',numel(jointState), ...
        'safetyObjective',"sumOfPrefixSafetySlacks",'primaryOptimumComputed',true, ...
        'safetyValueInterpretation',"validatedNonlinearSlackSum", ...
        'secondaryObjective',"clfSlackAndReferenceDeviation", ...
        'secondaryOptimumApplied',true, ...
        'tertiaryObjective',"none", ...
        'preplannedAvoidanceTrajectoryRequired',false,'drivingModeSwitching',false, ...
        'clfSlack',solution.clfSlack,'clfInitialValue',solution.clfInitialValue, ...
        'clfNextValue',solution.clfNextValue,'clfSatisfied',search.clfLowerBound,'clfContraction',nominalReference.contraction,'clfConvergenceTimeConstantSeconds',cfg.clf.convergenceTimeConstantSeconds, ...
        'clfFunction',solution.clfFunction,'clfRequiredDecrease',solution.clfRequiredDecrease, ...
        'referenceDeviationWeight',solution.referenceDeviationWeight, ...
        'referenceStateDeviation',solution.referenceStateDeviation,'referenceInputDeviation',solution.referenceInputDeviation, ...
        'referenceDeviationCost',solution.referenceDeviationCost,'secondaryObjectiveValue',solution.secondaryObjectiveValue, ...
        'clfLocalModel',"quadraticOnAffineFirstSuccessor", ...
        'nominalTrackingMatrix',nominalReference.matrix,'clfReferenceState',nominalReference.state, ...
        'clfOperatingInput',nominalReference.input,'clfOperatingCurvature',frame(4), ...
        'controlSource',search.source,'solverAlgorithm',"realTimeIteration", ...
        'nonlinearPredictionEvaluated',search.rollout.available, ...
        'solverCallCount',search.solverCalls,'search',search,'hardConstraintResidual',solution.hard, ...
        'optimizationConverged',search.converged,'affineFeasibilityTolerance',cfg.solver.feasibilityTolerance,'minimumCollisionMargin',solution.minimumCollisionMargin, ...
        'roadConstraintsEnforced',~isempty(road.lateralClearance),'roadConstraintScope',"rectangleAtEveryPredictedNode", ...
        'horizonSteps',size(predictedInput,2), ...
        'prefixSteps',cfg.controller.horizonSteps,'completionSteps',size(predictedInput,2)-cfg.controller.horizonSteps, ...
        'absoluteSampleIndex',index,'endpointIndex',index+size(predictedInput,2), ...
        'terminalSet',"jointEncounterBrakingSafe", ...
        'terminalPolicy',"heldSteeringConstantBrakingWithAbsorbingStop", ...
        'encounterId',encounterId,'encounterExitRecord',model.encounterExitRecord, ...
        'terminalValue',solution.terminalValue,'terminalMarginMeters',solution.terminalMarginMeters, ...
        'terminalStopSeconds',solution.terminalStopSeconds, ...
        'terminalExitSeconds',solution.terminalExitSeconds,'terminalEnd',solution.terminalEnd, ...
        'terminalDistanceMeters',solution.terminalDistanceMeters, ...
        'terminalRoadMarginMeters',solution.terminalRoadMarginMeters, ...
        'postSolveInputModification',false,'postSolveFeasibilityCheck',true, ...
        'anchorCertified',search.anchorCertified,'appendedTerminalSteps',search.appendedTerminalSteps, ...
        'appendedBrakingSteps',search.appendedBrakingSteps, ...
        'linearizationGapMeters',solution.linearizationGap, ...
        'encounterRangeMeters',cfg.collision.encounterRangeMeters,'encounterExitStep',solution.encounterExit, ...
        'predictedNextState',solution.states(:,2),'initialEgoState',ego.modelState,'runtimeSeconds',toc(timer));
    metadata.rtiIterations=search.rtiIterations;
    metadata.affineSubproblemCompleted=search.convexSolveCompleted;
    metadata.nonlinearHardResidual=solution.hard;
    metadata.affineSafetyValue=solution.affineSafety;
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
        metadata.safetyScope="nonlinearSampledConstraintsWithObserverMargins";
    end
    if model.robustnessRelaxation
        metadata.safetyObjective="sumOfRobustnessDeficitsWithHardNominalCollisionRows";
        metadata.safetyScope="nonlinearNominalCollisionWithRelaxedObserverMargins";
    end
    avoidanceInitialized=search.planner.used;
    if ~model.targetOnset && isstruct(previousState) && isfield(previousState,'avoidanceInitialized')
        avoidanceInitialized=avoidanceInitialized || previousState.avoidanceInitialized;
    end
    controllerState=struct('avoidanceInitialized',avoidanceInitialized,'version',92,'encounterId',encounterId,'stateTime',ego.stateTime,'sampleTime',cfg.controller.sampleTime, ...
        'sampleIndex',index,'epochTime',epochTime,'targetEpoch',targetEpoch,'targetEpochIndex',targetEpochIndex,'context',{context}, ...
        'jointState',jointState,'predictedJointState',predictedJointState, ...
        'appliedInput',first,'inputTrajectory',predictedInput,'stateTrajectory',solution.states, ...
        'stageSlacks',solution.stageSlacks,'safetyBudget',solution.safety, ...
        'collisionEscapeNormals',solution.collisionEscapeNormals,'collisionSide',solution.collisionSide, ...
        'retainCollisionSide',solution.retainCollisionSide,'circulationDirection',solution.circulationDirection, ...
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

function localWarmUp(cfg,referenceRoad)
    % Startup probes cover cruise and bounded low-speed feedback, including
    % a curved target. Every command and controller state is discarded.
    cfg=collisionAvoidanceControllerConfig(cfg);h=cfg.controller.sampleTime;
    cfg.solver.timeLimitSeconds=max(1,cfg.solver.timeLimitSeconds);
    cfg.solver.maxIterations=max(80,cfg.solver.maxIterations);
    % Discarded compilation probes include deliberately infeasible states.
    % Prime their kernels on a bounded horizon instead of searching until
    % the real encounter's deadline. This local copy never changes run-time
    % horizon selection or the caller's configuration.
    cfg.controller.maximumHorizonSteps=min(cfg.controller.maximumHorizonSteps,2*cfg.controller.horizonSteps);
    road=struct('centerline',[-100,0;1000,0],'lateralClearance',[8.5344;12.192]);
    % Prime the cold planner as well as nominal and bounded feedback paths.
    probe=struct('position',[0;0],'yaw',0,'speed',cfg.referenceSpeed, ...
        'lateralVelocity',0,'yawRate',0,'heldActuatorInput',[0;0],'stateTime',0);
    opposing=struct('targetPositionInertial',[24;0], ...
        'targetVelocityInertial',[-8;0],'targetYawInertial',pi);
    try
        collisionAvoidanceController(probe,opposing,road,cfg,[]);
    catch exception
        if ~startsWith(string(exception.identifier),"collisionAvoidanceController:"),rethrow(exception);end
    end
    for bounded=[false,true]
        x=[0;0;0;cfg.referenceSpeed;0;0];input=[0;0];state=[];
        q=[40;3;pi;8;0;0;cfg.target.rearAxleDistance;cfg.target.defaultLength/2;cfg.target.defaultWidth/2;0;0];
        if bounded
            x(4)=0;q(1:6)=[-9;3.5;pi-.02;8;.1;.02];
        end
        for frame=1:24
            time=(frame-1)*h;target=[];
            rotation=[cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
            if frame>4 || bounded
                if bounded && frame>12,q(3)=pi+.02;q(5)=-3;q(6)=-.02;end
                p=predictiveSafetyGeometry.predictTarget(q,max(0,(frame-5)*h));
                course=p(3)+p(6);
                target=struct('targetPositionInertial',p(1:2), ...
                    'targetVelocityInertial',p(4)*[cos(course);sin(course)], ...
                    'targetYawInertial',p(3),'targetSideslip',p(6),'targetTangentialAcceleration',p(5));
            end
            ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6), ...
                'heldActuatorInput',input,'stateTime',time);
            if bounded
                velocity=rotation*x(4:5);
                ego.egoState=[x(1);velocity(1);0;x(2);velocity(2);0];ego.egoYaw=x(3);ego.egoYawRate=x(6);
                bounds=[.001;.001;1e-4;1e-4;1e-4;1e-4];
                ego.controllerErrorBound=struct('kind',"ego-state-v1",'time',time,'bounds',bounds, ...
                    'available',true,'generator',diag(bounds),'generatorConvention',"commonPoseColumnsFirst");
                target.controllerErrorBound=struct('kind',"target-state-v1",'time',time, ...
                    'bounds',.001*ones(8,1),'available',true);
                target.predictionErrorSet=struct('kind',"nrmm-constant-parameter-set-v1",'time',time,'available',true, ...
                    'referenceEgoPose',x(1:3),'relativePosition',rotation.'*(p(1:2)-x(1:2)), ...
                    'positionRadius',.001,'courseCenter',course-x(3),'courseRadius',1e-4, ...
                    'speedInterval',p(4)+[-.001;.001],'accelerationInterval',p(5)+[-.001;.001], ...
                    'curvatureInterval',sin(p(6))/p(7)+[-1e-5;1e-5],'rearAxleDistance',p(7));
            end
            try
                [command,~,~,state]=collisionAvoidanceController(ego,target,road,cfg,state);
            catch exception
                if ~startsWith(string(exception.identifier),"collisionAvoidanceController:"),rethrow(exception);end
                break;
            end
            input=command.actuatorInput;x=nonlinearBicycleModel.sample(x,input,cfg);
        end
    end
    if isstruct(referenceRoad) && isfield(referenceRoad,'referenceCurve')
        % Prepare the supplied road outside the online budget. The opposing
        % target meets path cruise after 1.6 s; every probe output is discarded.
        curve=laneGeometry.validateReferenceCurve(referenceRoad.referenceCurve);
        reference=nonlinearBicycleModel.cruise(cfg,curve.curvature);
        x=reference.state;x(1:2)=curve.origin;x(3)=x(3)+curve.heading;
        [position,heading]=laneGeometry.referencePose(cfg.referenceSpeed*1.6,0,curve);
        beta=asin(-curve.curvature*cfg.target.rearAxleDistance);
        impact=[position;heading+pi-beta;8;0;beta;cfg.target.rearAxleDistance; ...
            cfg.target.defaultLength/2;cfg.target.defaultWidth/2;0;0];
        q=predictiveSafetyGeometry.predictTarget(impact,-1.6);
        ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6), ...
            'heldActuatorInput',[0;0],'stateTime',0);
        target=struct('targetPositionInertial',q(1:2), ...
            'targetVelocityInertial',q(4)*predictiveSafetyGeometry.direction(q(3)+q(6)), ...
            'targetYawInertial',q(3),'targetSideslip',q(6),'targetTangentialAcceleration',q(5));
        try
            collisionAvoidanceController(ego,target,referenceRoad,cfg,[]);
        catch exception
            if ~startsWith(string(exception.identifier),"collisionAvoidanceController:"),rethrow(exception);end
        end
    end
end
