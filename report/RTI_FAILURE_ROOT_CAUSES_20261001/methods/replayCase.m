function record = replayCase(speed,name,frames,outFile,captureTimes,overrideDir)
%replayCase Instrumented closed-loop replay of one campaign case.
% Uses the frozen snapshot controller unchanged and the same fixture,
% target conversion and ODE45 plant step as runNonlinearPredictiveSafetyValidation.
% Per frame it stores the issued input, the returned affine plan, the
% linearization anchor, solver stages, CLF values and offline geometry.
    source='/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/source';
    addpath(fullfile(source,'controller'),fullfile(source,'config'),fullfile(source,'scripts'));
    if nargin>5 && strlength(string(overrideDir))>0
        addpath(overrideDir);clear solvePredictiveControl;rehash; % DIAGNOSTIC override placed first on the path
        assert(startsWith(which('solvePredictiveControl'),overrideDir),'Override not active.');
    end
    prefix=8;if speed==15,prefix=16;end
    configuration=struct('referenceSpeed',speed,'controller',struct('horizonSteps',prefix));
    [x,q,road,cfg]=collisionThreatScenario(name,configuration);
    target=localTarget(q);ego=localEgo(x,0,[0;0]);prior=[];
    h=cfg.controller.sampleTime;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    collisionAvoidanceController("resetNominalTrajectory");
    frame=struct([]);capture=struct('time',{},'previousState',{},'model',{},'ego',{},'target',{});
    if nargin<5,captureTimes=[];end
    for k=1:frames
        if any(abs(captureTimes-(k-1)*h)<1e-9)
            capture(end+1)=struct('time',(k-1)*h,'previousState',prior,'model',[],'ego',ego,'target',target); %#ok<AGROW>
        end
        timer=tic;
        [command,plan,problem,next]=collisionAvoidanceController(ego,target,road,cfg,prior);
        if ~isempty(capture) && abs(capture(end).time-(k-1)*h)<1e-9,capture(end).model=problem.model;end
        seconds=toc(timer);
        model=problem.model;search=problem.metadata.search;x=model.initialState;u=command.actuatorInput;
        [times,states]=ode45(@(~,s)nonlinearBicycleModel.derivative(s,u,cfg),linspace(0,h,31),x, ...
            odeset('RelTol',1e-11,'AbsTol',1e-12));
        clearance=Inf;
        for j=1:numel(times)
            p=predictiveSafetyGeometry.targetFlow(model.targetEpoch,model.sampleIndex*h+times(j));
            clearance=min(clearance,predictiveSafetyGeometry.rectangle(states(j,1:3).',shape,p(1:3),p(8:11)));
        end
        successor=states(end,:).';
        error0=nonlinearBicycleModel.error(x,model.lane,model.nominalReference);
        error1=nonlinearBicycleModel.error(successor,model.lane,model.nominalReference);
        actualNext=norm(model.nominalReference.factor*error1)^2;
        % Offline geometry of the returned affine plan and of its nonlinear rollout.
        [affineMargin,affineStage]=localPlanMargin(problem.solution.states,model,shape,h);
        rollout=zeros(6,size(plan,2)+1);rollout(:,1)=x;rolloutOk=true;
        try
            for j=1:size(plan,2),rollout(:,j+1)=nonlinearBicycleModel.sample(rollout(:,j),plan(:,j),cfg);end
        catch
            rolloutOk=false;
        end
        rolloutMargin=NaN;rolloutStage=NaN;
        if rolloutOk,[rolloutMargin,rolloutStage]=localPlanMargin(rollout,model,shape,h);end
        anchor=model.linearization;
        entry=struct('time',(k-1)*h,'sampleIndex',model.sampleIndex,'state',x,'input',u,'seconds',seconds, ...
            'successor',successor,'clearance',clearance,'plan',plan,'affineStates',problem.solution.states, ...
            'anchorInputs',anchor.inputs,'anchorStates',anchor.states,'initialization',search.initialization, ...
            'flowRestarted',search.flowRestarted,'initializationFailure',search.initializationFailure, ...
            'primaryOptimum',search.primaryOptimum,'stageFlags',[search.stages.exitFlag], ...
            'stageValues',[search.stages.value],'stageSlacks',problem.solution.stageSlacks, ...
            'clfInitialValue',problem.metadata.clfInitialValue,'clfNextValue',problem.metadata.clfNextValue, ...
            'clfSlack',problem.metadata.clfSlack,'actualClfNextValue',actualNext,'error0',error0,'error1',error1, ...
            'encounterExit',problem.solution.encounterExit,'affineMargin',affineMargin,'affineMarginStage',affineStage, ...
            'rolloutMargin',rolloutMargin,'rolloutMarginStage',rolloutStage,'rolloutOk',rolloutOk, ...
            'rolloutPositionError',max(vecnorm(rollout(1:2,:)-problem.solution.states(1:2,:),2,1)), ...
            'endpointTerminal',terminalContinuation.membership([problem.solution.states(:,end);plan(:,end)], ...
                model.sampleIndex+size(plan,2),model.terminal));
        if isempty(frame),frame=entry;else,frame(end+1)=entry;end %#ok<AGROW>
        prior=next;ego=localEgo(successor,ego.stateTime+h,u);
        p=predictiveSafetyGeometry.targetFlow(model.targetEpoch,(model.sampleIndex+1)*h);target=localTarget(p);
    end
    record=struct('speed',speed,'scenario',name,'configuration',cfg,'targetEpoch',q,'road',road,'frame',frame,'capture',capture);
    if nargin>3 && strlength(string(outFile))>0,save(outFile,'record','-v7.3');end
end

function [margin,stage]=localPlanMargin(states,model,shape,h)
    % Minimum rectangle distance at the plan's hold nodes up to the encounter exit.
    margin=Inf;stage=NaN;
    for j=1:size(states,2)
        p=predictiveSafetyGeometry.targetFlow(model.targetEpoch,(model.sampleIndex+j-1)*h);
        d=predictiveSafetyGeometry.rectangle(states(1:3,j),shape,p(1:3),p(8:11));
        if d<margin,margin=d;stage=j-1;end
    end
end

function ego=localEgo(x,time,input)
    ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6), ...
        'stateTime',time,'heldActuatorInput',input);
end

function target=localTarget(q)
    direction=[cos(q(3)+q(6));sin(q(3)+q(6))];velocity=q(4)*direction;
    yawRate=q(4)*sin(q(6))/q(7);
    target=struct('targetPositionInertial',q(1:2),'targetVelocityInertial',velocity, ...
        'targetYawInertial',q(3),'targetYawRate',yawRate,'targetSideslip',q(6), ...
        'targetTangentialAcceleration',q(5),'targetRearAxleDistance',q(7), ...
        'targetAccelerationInertial',q(5)*direction+yawRate*[-velocity(2);velocity(1)], ...
        'targetLength',2*q(8),'targetWidth',2*q(9),'targetRectangleOffset',q(10:11));
end
