function summary = verifyOncomingAvoidanceWitness(options)
%verifyOncomingAvoidanceWitness Audit a held nonlinear avoidance trajectory.
% Validated flow enclosures cover each hold; rectangle support additionally
% checks collision, model domains and physical tire-slip angles over every cell.
    arguments
        options.WitnessFile (1,1) string
        options.DiagnosisFile (1,1) string
        options.OutputDirectory (1,1) string
        options.BuildVerifier (1,1) logical = true
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    loaded=load(options.WitnessFile,'result');w=loaded.result;
    source=load(options.DiagnosisFile,'ego','target','problem');
    cfg=w.configuration;h=cfg.controller.sampleTime;
    assert(h==.1 && size(w.input,2)==40);
    assert(isequal(source.target.targetVelocityInertial,[-8;0]) && ...
        isequal(source.target.targetAccelerationInertial,[0;0]) && ...
        source.target.targetHeadingInertial==pi && source.target.targetYawRate==0);
    assert(isequal(w.initialState,[source.ego.position;source.ego.yaw; ...
        source.ego.speed;source.ego.lateralVelocity;source.ego.yawRate]));
    if ~isfolder(options.OutputDirectory),mkdir(options.OutputDirectory);end
    nativeDirectory=fullfile(options.OutputDirectory,'native');
    if options.BuildVerifier,buildFialaIntervalVerifier(nativeDirectory);end
    addpath(nativeDirectory);
    timer=tic;
    sequence=fialaCertificate.sequence(w.initialState,w.initialState,w.initialState, ...
        w.input,zeros(2,6),zeros(6,1),source.ego.heldActuatorInput,cfg, ...
        maximumCellDuration=.0025,maximumGenerators=128);
    enclosureSeconds=toc(timer);
    clearance=Inf;domain=Inf;slip=Inf;criticalCombination=-Inf;cellCount=0;
    egoSize=[cfg.vehicle.length/2;cfg.vehicle.width/2];
    targetSize=[cfg.target.defaultLength/2;cfg.target.defaultWidth/2];
    lower=[-cfg.model.lateralDomainRadius;-cfg.model.headingDomainRadius; ...
        cfg.model.speedMinimum;-cfg.model.lateralVelocityMaximum;-cfg.model.yawRateMaximum];
    upper=[cfg.model.lateralDomainRadius;cfg.model.headingDomainRadius; ...
        cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum];
    for stage=1:sequence.completedSamples
        certificate=sequence.samples{stage};
        for index=1:numel(certificate.cells)
            cell=certificate.cells(index);box=cell.swept;
            first=(stage-1)*h+cell.start;last=(stage-1)*h+cell.end;
            targetCenter=source.target.targetPositionInertial+[-8;0]*(first+last)/2;
            targetRadius=[4*(last-first);0];
            center=mean(box(1:6,:),2);radius=diff(box(1:6,:),1,2)/2;
            [~,normal]=avoidanceSafetyGeometry.rectangleDistance(center(1:2),center(3), ...
                targetCenter,pi,[egoSize;targetSize]);
            egoSupport=targetPrediction.rectangleSupport(egoSize(1),egoSize(2),normal,center(3),radius(3));
            targetSupport=targetPrediction.rectangleSupport(targetSize(1),targetSize(2),normal,pi,0);
            allowance=1e-10*(1+norm(center(1:2))+norm(targetCenter));
            margin=normal.'*(center(1:2)-targetCenter)-abs(normal).'*(radius(1:2)+targetRadius) ...
                -egoSupport-targetSupport-allowance;
            clearance=min(clearance,margin);
            domain=min([domain;box(2:6,1)-lower;upper-box(2:6,2)]);
            slip=min(slip,localSlipMargin(box,cfg));
            if first<=.842857142857143 && last>=.842857142857143
                criticalCombination=box(2,2)-2.4*box(3,1);
            end
            cellCount=cellCount+1;
        end
    end
    m=source.problem.model;m.initialEgoState=w.initialState;
    m.sampleTime=.001;coarse=ltvBicycleModel.nominalRollout(m,repelem(w.input,1,100));
    m.sampleTime=.0005;fine=ltvBicycleModel.nominalRollout(m,repelem(w.input,1,200));
    numericalDifference=max(abs(coarse-fine(:,1:2:end)),[],'all');
    times=(0:size(fine,2)-1)*.0005;
    sampledClearance=Inf;
    for index=1:size(fine,2)
        target=source.target.targetPositionInertial+[-8;0]*times(index);
        sampledClearance=min(sampledClearance,avoidanceSafetyGeometry.rectangleDistance( ...
            fine(1:2,index),fine(3,index),target,pi,[egoSize;targetSize]));
    end
    finalBox=nan(6,2);exitMargin=-Inf;
    if sequence.completedSamples>0
        finalBox=sequence.samples{sequence.completedSamples}.endpoint(1:6,:);
        finalTarget=source.target.targetPositionInertial+[-8;0]*sequence.completedSamples*h;
        exitMargin=finalBox(1,1)-finalTarget(1)-targetSize(1)-source.ego.perception.range;
    end
    summary=struct('flowAccepted',sequence.accepted,'completedHolds',sequence.completedSamples, ...
        'durationSeconds',size(w.input,2)*h,'cellCount',cellCount,'enclosureSeconds',enclosureSeconds, ...
        'minimumContinuousExcessSeparation',clearance,'minimumContinuousDomainMargin',domain, ...
        'minimumContinuousSlipMarginRad',slip,'minimumSampledExcessSeparation',sampledClearance, ...
        'maximumReplayDifference',numericalDifference,'finalState',fine(:,end), ...
        'finalStateEnclosure',finalBox,'finalEntireTargetExitMargin',exitMargin, ...
        'maximumLateralPosition',max(abs(fine(2,:))),'maximumHeadingRad',max(abs(fine(3,:))), ...
        'minimumSpeed',min(fine(4,:)),'maximumSpeed',max(fine(4,:)), ...
        'inputMinimum',min(w.input,[],2),'inputMaximum',max(w.input,[],2), ...
        'criticalLateralCombinationUpperBound',criticalCombination, ...
        'safetyVerified',sequence.accepted && min([clearance,domain,slip,exitMargin])>=0, ...
        'scope',"Finite exact-state Fiala-model witness; no real-vehicle, online-time or recursive-controller guarantee");
    save(fullfile(options.OutputDirectory,'witness-verification.mat'),'summary','sequence','fine','times');
    stream=fopen(fullfile(options.OutputDirectory,'verification.json'),'w');
    cleanup=onCleanup(@()fclose(stream));fprintf(stream,'%s\n',jsonencode(summary,PrettyPrint=true));
    disp(summary);
end

function margin=localSlipMargin(box,cfg)
    margin=Inf;lever=[cfg.vehicle.lf;-cfg.vehicle.lr];
    speed=max(box(4,:),cfg.model.scheduleSpeedFloor);
    for axle=1:2
        lateral=box(5,:)+sort(lever(axle)*box(6,:));
        angles=atan2([lateral(1),lateral(1),lateral(2),lateral(2)], ...
            [speed(1),speed(2),speed(1),speed(2)]);
        interval=[min(angles),max(angles)];
        if axle==1,interval=interval-box(7,[2,1]);end
        margin=min(margin,cfg.model.slipAngleMaximum(axle)-max(abs(interval))-1e-10);
    end
end
