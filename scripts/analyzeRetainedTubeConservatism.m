function result = analyzeRetainedTubeConservatism()
%analyzeRetainedTubeConservatism Reproduce and isolate a conservative admission failure.
% The original controller/observer run is unchanged. Counterfactuals only
% re-enclose its captured failing hold; they are not a closed-loop campaign.
% No plots, reports or data files are written. The same bounded sensor and
% persistent tire-mismatch fixture as retainedTubeControllerTest is used.
% All enclosure arguments are in real arithmetic, not directed rounding.
    root=fileparts(fileparts(mfilename('fullpath')));oldPath=path;
    cleanup=onCleanup(@()path(oldPath));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'estimator'));
    f=localFixture();model=f.model;previous=[];runtime=f.runtime;x=f.x;held=f.u;
    failure=[];
    for k=0:7
        [solution,search]=solvePredictiveControl('probe',struct('operation','retained', ...
            'model',model,'previousState',previous,'anchor',f.anchor));
        if solution.terminalControlRequired
            failure=struct('sampleIndex',k,'time',k*f.cfg.controller.sampleTime, ...
                'model',model,'solution',solution,'posterior',search.posterior, ...
                'reason',search.refinementFailure,'trueState',x);break;
        end
        [runtime,x,output]=localAdvance(runtime,x,held,solution.inputs(:,1),f.cfg,k*f.cfg.controller.sampleTime);
        cfg=f.cfg;cfg.controller.retainedTube=true;
        [ego,~,~,observation]=readControllerInputs(output,[],[],cfg);
        held=solution.inputs(:,1);model.initialState=ego.modelState;model.previousInput=held;
        model.stateTime=output.stateTime;model.sampleIndex=k+1;model.targetOnset=false;
        model.uncertainty=predictiveSafetyGeometry.observerUncertainty(ego,observation,model.target);
        model.uncertainty.target=f.model.uncertainty.target;
        previous=struct('retainedCertificate',solution.retainedCertificate);
    end
    assert(~isempty(failure),'analysis:fixtureChanged','The reference failure was not reproduced.');
    initial=failure.posterior.proposedOuter;
    if ~predictiveSafetyGeometry.stateSetAdmissible(initial(1:6),f.cfg)
        old=failure.posterior.base;old(7:11)=nrmmSet(initial.center(7:11));
        old(16:17)=nrmmSet(initial.center(16:17));old(18:21)=initial(18:21);initial=old;
    end
    context=localContinuationContext(initial,failure.model);
    roots={initial,initial,localPosteriorBox(failure.posterior)};
    [lo,hi]=bounds(failure.posterior.proposedOuter(6));roots{2}(6)=nrmmSet.range(lo,hi);
    names=["retainedFallback","retainGyroMeasurement","constrainedPosteriorBox"];
    cases=struct('name',{},'currentRadius',{},'nextRadius',{},'sideUpper',{},'stateAdmissible',{});
    for knownFirst=[false,true]
        actual=context;
        if knownFirst,actual=localConditionFirst(actual);end
        for j=1:numel(roots)
            [item,~]=localHoldCheck(roots{j},actual);
            item.name=names(j)+"_knownFirst"+knownFirst;cases(end+1)=item; %#ok<AGROW>
        end
    end
    [contracted,contraction]=localImuContract(roots{3},context);
    known=localConditionFirst(context);[item,image]=localHoldCheck(contracted,known);
    item.name="currentImuConsistency";cases(end+1)=item;
    base=failure.solution.retainedCertificate;certificates=struct('controlRadius',{}, ...
        'stateAdmissible',{},'inputAdmissible',{},'sampledGaps',{}, ...
        'terminalValid',{},'terminalMinimumGap',{});
    for scale=[0,1]
        radius=scale*[.02;.05];policy=struct('context',known,'lower',-radius,'upper',radius);
        tube=nrmmJointPrediction('policyTube',contracted,policy);
        terminalContext=terminalSafeSet.context(failure.model);terminalContext.required=0;terminalContext.inflation=0;
        terminalContext.heldInputSet=tube.sets{end}(16:17);
        terminalContext.targetEpoch=base.target;terminalContext.targetEpochIndex=0;terminalContext.sampleIndex=0;
        time=base.epochSeconds+f.cfg.controller.sampleTime;
        terminal=terminalSafeSet.certifySet(terminalContext,tube.sets{end}(1:6),base.target,base.targetError,time);
        gaps=zeros(1,2);stateAdmissible=true;
        for j=1:2
            time=base.epochSeconds+(j-1)*f.cfg.controller.sampleTime;
            target=predictiveSafetyGeometry.targetPoseSet(base.target,base.targetError,time);
            normal=[];
            if j<=size(base.normals,2) && norm(base.normals(:,j))>.5,normal=base.normals(:,j);end
            gaps(j)=predictiveSafetyGeometry.setSeparation(tube.sets{j}(1:3), ...
                terminalContext.shape,target,base.target(8:11),normal);
            stateAdmissible=stateAdmissible && predictiveSafetyGeometry.stateSetAdmissible(tube.sets{j}(1:6),f.cfg);
        end
        [lo,hi]=bounds(tube.inputs{1}(2));
        inputAdmissible=lo>=f.cfg.actuation.brakingRatioMinimum && hi<=f.cfg.actuation.brakingRatioMaximum;
        [lo,hi]=bounds(tube.inputs{1}(2)-tube.sets{1}(17));
        inputAdmissible=inputAdmissible && max(abs([lo,hi]))<=f.cfg.model.brakingRatioRateMaximum*f.cfg.controller.sampleTime;
        [lo,hi]=bounds(tube.sets{end}(17));
        inputAdmissible=inputAdmissible && max(abs(terminalContext.brakingRatio-[lo,hi]))<=terminalContext.brakingChangeMaximum;
        certificates(end+1)=struct('controlRadius',radius, ...
            'stateAdmissible',stateAdmissible,'inputAdmissible',inputAdmissible, ...
            'sampledGaps',gaps,'terminalValid',terminal.valid,'terminalMinimumGap',terminal.minimumGap); %#ok<AGROW>
    end
    original=promote(initial);raw=nrmmJointEnclosure('hold',original.center,original.generator,context);
    row=zeros(1,29);row(4)=-tan(f.cfg.model.sideslipMaximum);row(5)=1;
    n=size(original.generator,2);noiseCount=7*round(context.duration/context.contract.samplePeriod);
    parts=[sum(abs(row*raw.generator(:,1:n))), ...
        sum(abs(row*raw.generator(:,n+(1:noiseCount)))), ...
        sum(abs(row*raw.generator(:,n+noiseCount+9:end)))];
    assert(~cases(1).stateAdmissible && cases(end).stateAdmissible);
    assert(failure.trueState(5)>=contraction.interval(1) && failure.trueState(5)<=contraction.interval(2));
    assert(all([certificates.stateAdmissible]) && all([certificates.inputAdmissible]));
    assert(all([certificates.terminalValid]) && all([certificates.sampledGaps]>0));
    result=struct('failureTime',failure.time,'failureReason',failure.reason, ...
        'cases',cases,'imuContraction',contraction,'certificates',certificates, ...
        'sideSupportContributions',parts,'contributionOrder',["incoming","freshSensorNoise","newNonlinearRemainders"], ...
        'trueStateAtFailure',failure.trueState,'verifiedNextSet',image(1:21), ...
        'scope',"single-failing-hold counterfactual; unchanged online algorithm; real arithmetic");
    fprintf('Original driving extension rejected at %.3f s (%s).\n',failure.time,failure.reason);
    for j=1:numel(cases)
        c=cases(j);fprintf('%s: current vy/r radius %s, next vy/r radius %s, cone upper %s, admissible %d\n', ...
            c.name,mat2str(c.currentRadius(5:6)',6),mat2str(c.nextRadius(5:6)',6),mat2str(c.sideUpper',6),c.stateAdmissible);
    end
    fprintf('IMU ay %.9g +/- %.6g m/s^2; retained vy interval %s m/s; %d range evaluations.\n', ...
        contraction.measurement,contraction.noiseRadius,mat2str(contraction.interval,8),contraction.evaluations);
    for j=1:numel(certificates)
        c=certificates(j);fprintf('Control radius %s: sampled gaps %s m, terminal valid %d, terminal gap %.6g m.\n', ...
            mat2str(c.controlRadius',4),mat2str(c.sampledGaps,6),c.terminalValid,c.terminalMinimumGap);
    end
end

function context=localContinuationContext(set,model)
    cfg=model.cfg;contract=model.uncertainty.jointPrediction;
    input=nonlinearBicycleModel.nominalFeedback(set.center(1:6),set.center(16:17), ...
        model.lane,model.nominalReference,cfg,nonlinearBicycleModel.nominalGuidanceParameters(cfg,0));
    anchor=struct('states',[set.center(1:6),set.center(1:6)],'inputs',input);
    gain=nrmmJointPrediction('feedback',anchor,cfg,cfg.controller.sampleTime);
    estimate=set([7:11,6]);reserve=sum(abs(gain*estimate.generator),2)+abs(gain)*estimate.remainder ...
        +abs(gain(:,6))*contract.noiseRadius(7);
    if reserve(2)>0
        authority=min(cfg.actuation.brakingRatioMaximum-input(2),input(2)-cfg.actuation.brakingRatioMinimum);
        gain(2,:)=gain(2,:)*min(1,max(0,authority)*(1-32*eps)/reserve(2));
    end
    context=struct('cfg',cfg,'contract',contract,'reference',[set.center(7:11);set.center(6)], ...
        'input',input,'gain',gain,'duration',cfg.controller.sampleTime,'first',false);
end

function context=localConditionFirst(context)
    observed=[context.contract.point;context.contract.currentMeasurement.yawRate];
    difference=observed-context.reference;difference(3)=atan2(sin(difference(3)),cos(difference(3)));
    context.input=context.input+context.gain*difference;
    context.gain=zeros(2,6);context.first=true;
end

function [result,next]=localHoldCheck(root,context)
    root=promote(root);image=nrmmJointEnclosure('hold',root.center,root.generator,context);
    next=nrmmSet(image.point,image.generator);s=tan(context.cfg.model.sideslipMaximum);
    rows=[0,0,0,-s,1,0;0,0,0,-s,-1,0];[~,upper]=bounds(rows*next(1:6));
    result=struct('name',"",'currentRadius',radius(root(1:6)),'nextRadius',radius(next(1:6)), ...
        'sideUpper',upper,'stateAdmissible',predictiveSafetyGeometry.stateSetAdmissible(next(1:6),context.cfg));
end

function set=localPosteriorBox(posterior)
    % Use both generator constraints and the measured physical error set.
    % Dual residual support bounds, not primal optima, enclose the projection.
    assert(all(posterior.primitiveLower==-1) && all(posterior.primitiveUpper==1));
    m=size(posterior.currentGenerator,2);
    lo=zeros(21,1);hi=lo;options=optimoptions('linprog','Display','none');
    for j=1:21
        c=[posterior.base.generator(j,:),zeros(1,m)]';
        lo(j)=posterior.base.center(j)+localLowerSupport(c,posterior,options);
        hi(j)=posterior.base.center(j)-localLowerSupport(-c,posterior,options);
    end
    set=nrmmSet.range(lo,hi);
end

function value=localLowerSupport(c,p,options)
    [~,~,flag,~,lambda]=linprog(c,p.a,p.b,p.equal,p.rhs,p.primitiveLower,p.primitiveUpper,options);
    assert(flag>0,'analysis:posteriorLp','The diagnostic support solve did not complete.');
    y=max(0,lambda.ineqlin);z=lambda.eqlin;
    value=-p.b'*y-p.rhs'*z-sum(abs(c+p.a'*y+p.equal'*z));
end

function [root,info]=localImuContract(root,context)
    % Cover the entire incoming lateral-velocity interval. Reject a cell
    % only when its FULL Fiala force image cannot match the current IMU.
    % Parameter/noise budgets are unchanged. Unresolved cells stay included.
    [lo,hi]=bounds(root);measurement=context.contract.currentMeasurement;
    noise=context.contract.noiseRadius(6);queue=[lo(5),hi(5)];kept=zeros(0,2);evaluations=0;
    while ~isempty(queue) && evaluations<10000
        interval=queue(end,:);queue(end,:)=[];
        state=nrmmSet.range(lo(1:6),hi(1:6));state(5)=nrmmSet.range(interval(1),interval(2));
        field=nrmmJointEnclosure('field',state,nrmmSet(measurement.heldInput), ...
            nrmmSet.range(lo(12:15),hi(12:15)),context.cfg);evaluations=evaluations+1;
        acceleration=field(5)+state(4)*state(6);[lower,upper]=bounds(acceleration);
        if upper<measurement.bodyAcceleration(2)-noise || lower>measurement.bodyAcceleration(2)+noise,continue;end
        if diff(interval)<=.0005
            kept(end+1,:)=interval; %#ok<AGROW>
        else
            middle=mean(interval);queue=[queue;interval(1),middle;middle,interval(2)]; %#ok<AGROW>
        end
    end
    kept=[kept;queue];assert(~isempty(kept),'analysis:inconsistentImu','No declared physical state matches this IMU.');
    interval=[min(kept(:,1)),max(kept(:,2))];root(5)=nrmmSet.range(interval(1),interval(2));
    info=struct('originalInterval',[lo(5),hi(5)],'interval',interval,'evaluations',evaluations, ...
        'measurement',measurement.bodyAcceleration(2),'noiseRadius',noise,'unresolvedCells',size(queue,1));
end

function f=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'controller',struct('horizonSteps',2),'solver',struct('timeLimitSeconds',120)));
    estimator=estimatorConfigurationFromController(estimatorControllerIntegrationConfig(),cfg);
    design=synthesizeNrmmObserverGains(estimator.observer);
    reference=nonlinearBicycleModel.cruise(cfg,0);x=reference.state;u=reference.input;
    options=struct('initialTime',0,'egoInitialPosition',x(1:2),'egoInitialYaw',x(3), ...
        'egoInitialBodyVelocity',x(4:5),'targetInitialState',[25;0;0;0;0;0], ...
        'inertialVelocityInitialization',struct('value',x(4:5),'radius',.05));
    runtime=onlineNrmmTrackingRuntime('initialize',estimator.observer,options,design);
    frame=struct('time',0,'xGps',x(1),'yGps',x(2),'longitudinalAcceleration',0, ...
        'lateralAcceleration',0,'yawRateMeasured',0,'radarDetectionAvailable',false, ...
        'radarRelativePosition',[NaN;NaN],'heldSteeringAngle',u(1),'heldBrakingRatio',u(2));
    output=onlineNrmmTrackingRuntime('output',runtime,frame);output.targetEstimate=[];
    [~,~,prediction]=collisionAvoidanceController(output,[],[],cfg,[]);
    model=prediction.model;model.target=[25;0;0;0;0;0;cfg.vehicle.lr;cfg.vehicle.length/2;cfg.vehicle.width/2;0;0];
    model.targetEpoch=model.target;model.targetOnset=true;model.cfg.controller.retainedTube=true;
    model.uncertainty.jointPrediction=output.controllerErrorBound.jointPrediction;
    model.uncertainty.target=struct('positionRadius',.02,'courseRadius',.001, ...
        'speedRadius',0,'accelerationRadius',0,'curvatureRadius',0,'sideslipRadius',0);
    anchor=struct('states',repmat(x,1,3),'inputs',repmat(u,1,2));
    for k=1:2,anchor.states(:,k+1)=nonlinearBicycleModel.sample(anchor.states(:,k),u,cfg);end
    f=struct('cfg',cfg,'x',x,'u',u,'contract',output.controllerErrorBound.jointPrediction, ...
        'generator',output.controllerErrorBound.generator,'model',model,'anchor',anchor,'output',output,'runtime',runtime);
end

function [runtime,x,output]=localAdvance(runtime,x,previous,input,cfg,time)
    plant=cfg;plant.tire.corneringStiffness=cfg.tire.corneringStiffness.*[1.01;.99];
    plant.tire.frictionCoefficient=cfg.tire.frictionCoefficient.*[1.02;.98];
    dt=runtime.samplePeriod;count=round(cfg.controller.sampleTime/dt);
    for k=1:count
        held=input;if k==1,held=previous;end
        frame=localFrame(x,held,plant,time+(k-1)*dt);
        runtime=onlineNrmmTrackingRuntime('step',runtime,frame);
        field=@(z)nonlinearBicycleModel.derivative(z,input,plant);
        a=field(x);b=field(x+dt*a/2);c=field(x+dt*b/2);d=field(x+dt*c);
        x=x+dt*(a+2*b+2*c+d)/6;
    end
    x=nonlinearBicycleModel.sampleEndState(x,input,plant);
    output=onlineNrmmTrackingRuntime('output',runtime,localFrame(x,input,plant,time+cfg.controller.sampleTime));
    output.targetEstimate=[];
end

function frame=localFrame(x,input,cfg,time)
    field=nonlinearBicycleModel.derivative(x,input,cfg);
    frame=struct('time',time,'xGps',x(1)+.004*sin(11*time),'yGps',x(2)+.004*sin(13*time), ...
        'longitudinalAcceleration',field(4)-x(5)*x(6)+.003*sin(17*time), ...
        'lateralAcceleration',field(5)+x(4)*x(6)+.003*sin(19*time), ...
        'yawRateMeasured',x(6)+.0001*sin(23*time),'radarDetectionAvailable',false, ...
        'radarRelativePosition',[NaN;NaN],'heldSteeringAngle',input(1),'heldBrakingRatio',input(2));
end
