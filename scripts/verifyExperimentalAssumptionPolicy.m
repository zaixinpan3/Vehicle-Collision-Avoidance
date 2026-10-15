function result = verifyExperimentalAssumptionPolicy(outputDirectory,frameCap)
%verifyExperimentalAssumptionPolicy Separate empirical outcomes from proof premises.
% The plant target always has constant A and beta. A deterministic observation
% perturbation revises their estimates every frame without a certified bound.
% This isolates prediction-update robustness; it is not a full sensor/observer
% closed loop. A separate probe consumes an actual NRMM runtime publication.
    arguments
        outputDirectory (1,1) string
        frameCap (1,1) double {mustBeInteger,mustBePositive} = 1200
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'estimator'));
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    runtimeProbe=localRuntimeProbe();
    summaries=struct([]);
    for perturb=false:true
        hook=[];label="exact";
        if perturb,hook=@localPerturbedEstimate;label="revisedEstimate";end
        for name=["headOn","acceleratingHeadOn","crossing"]
            stem=fullfile(outputDirectory,label+"-"+name);
            report=runNonlinearPredictiveSafetyValidation(Scenarios=name,Frames=frameCap, ...
                RequireCollisionThreat=true,RecoveryDwellSeconds=1, ...
                TargetEstimateFunction=hook,OutputFile=stem+".json",ContinuationFile=stem+".mat");
            row=rmfield(report.results,{'trace','configuration'});row.observation=label;
            summaries=[summaries,row]; %#ok<AGROW>
            result=struct('scope',"Empirical prediction-update robustness, not a recursive-feasibility proof", ...
                'targetTruthContract',"constant A and beta in every physical scenario", ...
                'perturbation',struct('positionAmplitudeMeters',[.03;.03], ...
                    'speedAmplitudeMetersPerSecond',.02,'accelerationAmplitudeMetersPerSecondSquared',.02, ...
                    'sideslipAmplitudeRadians',.0005,'decaySeconds',5,'randomSeed',[]), ...
                'runtimePublicationProbe',runtimeProbe,'scenarios',summaries);
            file=fopen(fullfile(outputDirectory,'summary.json'),'w');assert(file>=0);
            fprintf(file,'%s\n',jsonencode(result));fclose(file);
        end
    end
end

function target=localPerturbedEstimate(target,ego)
    if isempty(target),return;end
    time=ego.stateTime;envelope=exp(-time/5);
    target.targetPositionInertial=target.targetPositionInertial ...
        +.03*envelope*[sin(3*time);cos(5*time)];
    target.targetTangentialAcceleration=target.targetTangentialAcceleration+.02*envelope*cos(4*time);
    target.targetSideslip=target.targetSideslip+.0005*envelope*sin(6*time);
    speed=norm(target.targetVelocityInertial)+.02*envelope*sin(2*time);
    course=target.targetYawInertial+target.targetSideslip;
    direction=[cos(course);sin(course)];velocity=speed*direction;
    yawRate=speed*sin(target.targetSideslip)/target.targetRearAxleDistance;
    target.targetVelocityInertial=velocity;target.targetYawRate=yawRate;
    target.targetAccelerationInertial=target.targetTangentialAcceleration*direction ...
        +yawRate*[-velocity(2);velocity(1)];
    target.controllerErrorBound=struct('kind',"target-state-v1",'time',time, ...
        'bounds',Inf(8,1),'available',false);
end

function probe=localRuntimeProbe()
    cfg=nrmmTrackingConfig();
    options=struct('initialTime',0,'egoInitialPosition',[0;0],'egoInitialYaw',0, ...
        'egoInitialBodyVelocity',[10;0],'targetInitialState',[20;5;-12;0;0;0]);
    runtime=onlineNrmmTrackingRuntime("initialize",cfg,options);
    frame=struct('time',0,'xGps',0,'yGps',0,'vxGps',10,'vyGps',0, ...
        'longitudinalAcceleration',0,'lateralAcceleration',0,'yawRateMeasured',0,'radarRelativePosition',[20,5]);
    publication=onlineNrmmTrackingRuntime("output",runtime,frame);
    controllerCfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',10,'controller',struct('horizonSteps',8)));
    road=struct('centerline',[-50,0;200,0]);
    wall=tic;
    probe=struct('solvedControl',zeros(2,0),'optimizationReturned',false,'failure',"",'runtimeSeconds',NaN);
    try
        [command,~,prediction]=collisionAvoidanceController(publication,publication.targetEstimate,road,controllerCfg,[]);
        probe.solvedControl=command.actuatorInput;
        probe.optimizationReturned=prediction.metadata.optimizationReturned;
    catch exception
        probe.failure=string(exception.identifier)+": "+string(exception.message);
    end
    probe.runtimeSeconds=toc(wall);
end
