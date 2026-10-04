function report = auditCollisionMarginSampling(campaignDirectory,outputDirectory)
%auditCollisionMarginSampling Diagnose recorded sub-margin holds offline.
% Replays issued decisions from recorded measurements, then compares collision
% rows, affine poses and dense ODE motion. No controller constraints change.
    arguments
        campaignDirectory (1,1) string
        outputDirectory (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    files=dir(fullfile(campaignDirectory,'speed*.json'));
    cases=struct([]);captures={};
    for file=files.'
        source=fullfile(file.folder,file.name);saved=jsondecode(fileread(source));r=saved.results;
        margin=r.configuration.collision.safetyMarginMeters;
        if r.minimumReplayClearanceMeters>=margin,continue;end
        % JSON represents infinite configuration limits as null. The paired
        % MATLAB continuation retains those exact limits for solver replay.
        configurationSource=replace(source,'.json','.mat');
        original=load(configurationSource,'continuation');cfg=original.continuation.result.configuration;
        assert(nonlinearBicycleModel.meshCount(cfg)==1,'This audit describes the one-step hold and its interpolated midpoint.');
        trace=r.trace;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
        minimum=Inf;critical=0;sampleIndex=0;
        for k=1:numel(trace)
            for j=1:numel(trace(k).auditTimes)
                gap=localGap(trace(k).auditStates(j,:).',trace(k).time+trace(k).auditTimes(j),shape,r.targetInitialState);
                if gap<minimum,minimum=gap;critical=k;sampleIndex=j;end
            end
        end
        assert(abs(minimum-r.minimumReplayClearanceMeters)<1e-9,'Recorded gap does not match independent replay geometry.');
        [~,~,road]=collisionThreatScenario(string(r.scenario),cfg);
        prior=[];maximumInputError=0;held=zeros(2,1);
        for k=1:critical
            x=trace(k).state;t=trace(k).time;q=predictiveSafetyGeometry.predictTarget(r.targetInitialState,t);
            ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6), ...
                'stateTime',t,'heldActuatorInput',held);
            [command,~,prediction,prior]=collisionAvoidanceController(ego,localTarget(q),road,cfg,prior);
            maximumInputError=max(maximumInputError,norm(command.actuatorInput-trace(k).input,inf));
            held=trace(k).input;
        end
        assert(maximumInputError<1e-9,'Controller replay no longer reproduces the saved inputs.');
        hold=trace(critical);h=cfg.controller.sampleTime;time=hold.time;
        x=hold.state;input=hold.input;model=prediction.model;anchor=model.linearization;
        affine=prediction.solution.states(:,1:2);
        integration=ode45(@(~,state)nonlinearBicycleModel.derivative(state,input,cfg), ...
            [0,h],x,odeset('RelTol',1e-11,'AbsTol',1e-12));
        denseTimes=linspace(0,h,1001);denseStates=deval(integration,denseTimes);
        actualGaps=zeros(size(denseTimes));affineGaps=actualGaps;
        for j=1:numel(denseTimes)
            fraction=denseTimes(j)/h;pose=(1-fraction)*affine(:,1)+fraction*affine(:,2);
            actualGaps(j)=localGap(denseStates(:,j),time+denseTimes(j),shape,r.targetInitialState);
            affineGaps(j)=localGap(pose,time+denseTimes(j),shape,r.targetInitialState);
        end
        [~,j]=min(actualGaps);
        objective=@(dt)localGap(deval(integration,dt),time+dt,shape,r.targetInitialState);
        [closestOffset,closestGap]=fminbnd(objective,denseTimes(max(1,j-1)), ...
            denseTimes(min(numel(denseTimes),j+1)),optimset('TolX',1e-12,'Display','off'));
        nodes=struct([]);
        for fraction=[0,.5,1]
            dt=fraction*h;pose=(1-fraction)*affine(:,1)+fraction*affine(:,2);
            center=(1-fraction)*anchor.states(:,1)+fraction*anchor.states(:,2);
            q=predictiveSafetyGeometry.predictTarget(r.targetInitialState,time+dt);
            dual=predictiveSafetyGeometry.dualLinearization(center(1:3),shape,q(1:3),q(8:11));
            linear=min(dual.value+dual.jacobian*(pose(1:3)-center(1:3)));
            fixed=min(predictiveSafetyGeometry.fixedDualRows(pose(1:3),shape,q(1:3),q(8:11),dual.lambda));
            actual=deval(integration,dt);
            nodes=[nodes,struct('timeSeconds',time+dt,'affineRowClearanceMeters',linear, ...
                'fixedDualClearanceMeters',fixed,'affinePoseClearanceMeters',localGap(pose,time+dt,shape,r.targetInitialState), ...
                'odeClearanceMeters',localGap(actual,time+dt,shape,r.targetInitialState))]; %#ok<AGROW>
        end
        fraction=closestOffset/h;pose=(1-fraction)*affine(:,1)+fraction*affine(:,2);
        actual=deval(integration,closestOffset);reach=norm(shape(1:2))+norm(shape(3:4));
        poseError=norm(pose(1:2)-actual(1:2))+reach*abs(pose(3)-actual(3));
        next=nonlinearBicycleModel.sample(x,input,cfg);
        endpointError=norm(next(1:2)-denseStates(1:2,end))+reach*abs(next(3)-denseStates(3,end));
        row=struct('scenario',string(r.scenario),'speedMetersPerSecond',cfg.referenceSpeed,'source',source, ...
            'configurationSource',configurationSource,'frame',critical,'holdStartSeconds',time,'holdSeconds',h,'marginMeters',margin, ...
            'recordedMinimumMeters',minimum,'recordedMinimumSeconds',time+hold.auditTimes(sampleIndex), ...
            'refinedMinimumMeters',closestGap,'refinedMinimumSeconds',time+closestOffset, ...
            'affineInterpolantMinimumMeters',min(affineGaps), ...
            'affineAtActualClosestMeters',localGap(pose,time+closestOffset,shape,r.targetInitialState), ...
            'affineBodyPoseErrorAtClosestMeters',poseError,'rk4EndpointBodyErrorMeters',endpointError, ...
            'firstStageSlackMeters',prediction.solution.stageSlacks(1),'safetySlackSum',prediction.solution.safety, ...
            'maximumReplayInputError',maximumInputError,'nodes',nodes);
        cases=[cases,row];captures{end+1}=struct('prediction',prediction,'ode',integration, ...
            'denseTimes',denseTimes,'actualGaps',actualGaps,'affineGaps',affineGaps); %#ok<AGROW>
        fprintf('%g %s frame %d: sampled %.9f, refined %.9f, affine interval %.9f; replay input error %.3g\n', ...
            cfg.referenceSpeed,r.scenario,critical,minimum,closestGap,min(affineGaps),maximumInputError);
    end
    report=struct('scope',"Offline diagnostic of already recorded sub-margin holds; no controller change", ...
        'integration',"ODE45, RelTol 1e-11, AbsTol 1e-12; 1001 hold samples plus local scalar refinement", ...
        'claimLimit',"Numerical minimum diagnosis, not a validated continuous-time bound",'cases',cases);
    file=fopen(fullfile(outputDirectory,'margin-diagnosis.json'),'w');assert(file>=0);
    fprintf(file,'%s\n',jsonencode(report));fclose(file);
    save(fullfile(outputDirectory,'margin-captures.mat'),'report','captures','-v7.3');
end

function gap=localGap(x,time,shape,epoch)
    q=predictiveSafetyGeometry.predictTarget(epoch,time);
    gap=predictiveSafetyGeometry.rectangle(x(1:3),shape,q(1:3),q(8:11));
end

function target=localTarget(q)
    direction=[cos(q(3)+q(6));sin(q(3)+q(6))];velocity=q(4)*direction;yawRate=q(4)*sin(q(6))/q(7);
    target=struct('targetPositionInertial',q(1:2),'targetVelocityInertial',velocity, ...
        'targetYawInertial',q(3),'targetYawRate',yawRate,'targetSideslip',q(6), ...
        'targetTangentialAcceleration',q(5),'targetRearAxleDistance',q(7), ...
        'targetAccelerationInertial',q(5)*direction+yawRate*[-velocity(2);velocity(1)], ...
        'targetLength',2*q(8),'targetWidth',2*q(9),'targetRectangleOffset',q(10:11));
end
