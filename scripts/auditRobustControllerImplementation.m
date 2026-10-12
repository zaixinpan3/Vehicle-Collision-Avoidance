function results = auditRobustControllerImplementation(outputFile)
%auditRobustControllerImplementation Witnesses for missing robust certificates.
% Deterministic declared-model examples; this is an implementation audit,
% not a closed-loop experiment or a validated reachable-set algorithm.
    if nargin<1,outputFile="";end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    cfg=collisionAvoidanceControllerConfig();
    results=struct('sampleTimeSeconds',cfg.controller.sampleTime, ...
        'target',localTarget(cfg),'nonlinearImage',localNonlinearImage(cfg), ...
        'terminal',localTerminal(cfg));
    if strlength(string(outputFile))>0
        file=fopen(outputFile,'w');assert(file>=0,'Cannot open result file.');
        cleaner=onCleanup(@()fclose(file));
        fprintf(file,'%s\n',jsonencode(results,PrettyPrint=true));
    end
    disp(results);
end

function result=localTarget(cfg)
    q=[0;0;0;8;0;0;1.6;2.4;.95;0;0];
    errorSet=struct('positionRadius',.03,'courseRadius',.01,'speedRadius',.1, ...
        'accelerationRadius',.2,'curvatureRadius',.001,'sideslipRadius',.002);
    time=[1,3,5];h=cfg.controller.sampleTime;
    full=predictiveSafetyGeometry.targetErrorTube(q,errorSet,time);
    starts=predictiveSafetyGeometry.predictTarget(q,time-h);
    reset=predictiveSafetyGeometry.targetErrorTube(starts,errorSet,h+zeros(size(time)));
    actual=q;actual(5)=errorSet.accelerationRadius;
    nominal=predictiveSafetyGeometry.predictTarget(q,time);
    perturbed=predictiveSafetyGeometry.predictTarget(actual,time);
    distance=vecnorm(perturbed(1:2,:)-nominal(1:2,:),2,1);
    assert(all(distance<=full.positionRadius));
    assert(all(distance>reset.positionRadius));
    result=struct('nominalSpeedMetersPerSecond',q(4),'errorSet',errorSet, ...
        'timeSeconds',time,'fullHorizonPositionRadiusMeters',full.positionRadius, ...
        'resetEachHoldPositionRadiusMeters',reset.positionRadius, ...
        'admittedAccelerationErrorMetersPerSecondSquared',actual(5)-q(5), ...
        'exactPositionErrorMeters',distance);
end

function result=localNonlinearImage(cfg)
    x=[0;0;0;15;0;0];u=[0;0];d=[0;0;.1;0;0;0];
    [nominal,a]=nonlinearBicycleModel.sample(x,u,cfg);
    perturbed=nonlinearBicycleModel.sample(x+d,u,cfg);
    tangentRadius=abs(a(1,:)*d);
    displacement=perturbed(1)-nominal(1);
    assert(abs(displacement)>tangentRadius+1e-4);
    result=struct('initialState',x,'input',u,'yawPerturbationRadians',d(3), ...
        'tangentLongitudinalRadiusMeters',tangentRadius, ...
        'exactLongitudinalErrorMeters',displacement, ...
        'remainderNorm',norm(perturbed-nominal-a*d));
end

function result=localTerminal(cfg)
    states=[0,0;0,0;0,0;15,15;0,.3;0,.03];
    reference=nonlinearBicycleModel.cruise(cfg,0);
    result=repmat(struct('initialState',[],'terminalMember',false, ...
        'sideslipRadians',NaN,'lateralAccelerationMetersPerSecondSquared',NaN, ...
        'input',[],'positionDifferenceMeters',NaN,'bodyVelocityDifferenceMetersPerSecond',[], ...
        'yawRateDifferenceRadiansPerSecond',NaN),1,size(states,2));
    for index=1:size(states,2)
        x=states(:,index);
        raw=struct('position',x(1:2),'yaw',x(3),'speed',x(4), ...
            'lateralVelocity',x(5),'yawRate',x(6));
        rawRoad=struct('centerline',[-100,0;1000,0],'lateralClearance',[100;100]);
        [~,lane,road]=readControllerInputs(raw,[],rawRoad,cfg);
        model=struct('cfg',cfg,'initialState',x,'target',[],'targetEpoch',[], ...
            'targetEpochIndex',0,'sampleIndex',0,'lane',lane,'road',road, ...
            'nominalReference',reference);
        context=terminalSafeSet.context(model);
        member=terminalSafeSet.member(context,x,0);assert(member);
        input=[0;context.brakingRatio];
        actual=nonlinearBicycleModel.sample(x,input,cfg);
        formal=terminalSafeSet.successor(context,x);
        assert(norm(actual-formal)>1e-5);
        result(index)=struct('initialState',x,'terminalMember',member, ...
            'sideslipRadians',atan2(x(5),x(4)), ...
            'lateralAccelerationMetersPerSecondSquared',abs(x(6))*hypot(x(4),x(5)), ...
            'input',input,'positionDifferenceMeters',norm(actual(1:2)-formal(1:2)), ...
            'bodyVelocityDifferenceMetersPerSecond',actual(4:5)-formal(4:5), ...
            'yawRateDifferenceRadiansPerSecond',actual(6)-formal(6));
    end
end
