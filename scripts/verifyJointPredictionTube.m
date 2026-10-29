function results=verifyJointPredictionTube(options)
%verifyJointPredictionTube Compare causal joint tubes with nonlinear histories.
% No result is promoted to a uniform nonlinear enclosure certificate.
    arguments
        options.HorizonSteps (1,1) double {mustBeInteger,mustBePositive}=60
        options.WorldCount (1,1) double {mustBeInteger,mustBePositive}=8
        options.Cases (1,:) string=["cruise8","cruise15","brake8","turn8"]
        options.ModelAided (1,1) logical=true
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'estimator'));
    rows=struct([]);
    for name=options.Cases
        speed=8;if name=="cruise15",speed=15;end
        cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed,'solver',struct('timeLimitSeconds',120)));
        estimator=estimatorConfigurationFromController(estimatorControllerIntegrationConfig(),cfg);
        design=synthesizeNrmmObserverGains(estimator.observer);
        reference=nonlinearBicycleModel.cruise(cfg,0);x=reference.state;u=reference.input;
        initial=struct('initialTime',0,'egoInitialPosition',x(1:2),'egoInitialYaw',x(3), ...
            'egoInitialBodyVelocity',x(4:5),'targetInitialState',[20;0;speed;0;0;0], ...
            'inertialVelocityInitialization',struct('value',x(4:5),'radius',.05));
        runtime=onlineNrmmTrackingRuntime('initialize',estimator.observer,initial,design);
        frame=struct('time',0,'xGps',0,'yGps',0,'vxGps',speed,'vyGps',0, ...
            'longitudinalAcceleration',0,'lateralAcceleration',0,'yawRateMeasured',0, ...
            'radarDetectionAvailable',false,'radarRelativePosition',[NaN;NaN]);
        if options.ModelAided,frame.heldSteeringAngle=u(1);frame.heldBrakingRatio=u(2);end
        output=onlineNrmmTrackingRuntime('output',runtime,frame);
        contract=output.controllerErrorBound.jointPrediction;generator=output.controllerErrorBound.generator;
        count=options.HorizonSteps;h=cfg.controller.sampleTime;
        anchor=struct('states',zeros(6,count+1),'inputs',repmat(u,1,count));anchor.states(:,1)=x;
        for k=1:count
            if name=="brake8",anchor.inputs(2,k)=-.2;end
            if name=="turn8",anchor.inputs(1,k)=.035*sin(pi*(k-.5)/count);end
            anchor.states(:,k+1)=nonlinearBicycleModel.sample(anchor.states(:,k),anchor.inputs(:,k),cfg);
        end
        timer=tic;
        tube=nrmmJointPrediction('propagate',anchor,cfg,generator,contract,repmat(h,1,count));
        seconds=toc(timer);
        open=nrmmJointPrediction('propagate',anchor,cfg,generator,contract,repmat(h,1,count),struct('feedback',false));
        replay=localReplay(anchor,cfg,generator,contract,tube,options.WorldCount);
        row=struct('name',name,'seconds',seconds,'steps',count, ...
            'initialVelocity',anchor.states(4:5,1),'finalVelocity',anchor.states(4:5,end), ...
            'closedLoopTail',tube.physicalRadius(:,end),'openLoopTail',open.physicalRadius(:,end), ...
            'initialEstimation',tube.observerErrorRadius(:,1),'finalEstimation',tube.observerErrorRadius(:,end), ...
            'maximumInputRadius',max(tube.inputRadius,[],2),'replay',replay, ...
            'nonlinearCertificate',tube.nonlinearCertificate);
        rows=[rows;row]; %#ok<AGROW>
        fprintf('%s: %.3f s; position tail %.4f / %.4f m; physical / estimator excess %.6g / %.6g\n', ...
            name,seconds,row.closedLoopTail(1:2),replay.maximumPhysicalExcess,replay.maximumEstimationExcess);
    end
    results=struct('cases',rows,'seed',20261029,'worldCount',options.WorldCount, ...
        'scope',"joint affine supports and independent nonlinear histories", ...
        'fullNonlinearCertificate',false,'modelAided',options.ModelAided, ...
        'physicalCoordinateOrder',["x_m","y_m","yaw_rad","vx_mps","vy_mps","rate_radps"], ...
        'estimationCoordinateOrder',["x_m","y_m","yaw_rad","vx_mps","vy_mps"]);
end

function result=localReplay(anchor,cfg,generator,contract,tube,worldCount)
    stream=RandStream('mt19937ar','Seed',20261029);count=size(anchor.inputs,2);
    peak=zeros(6,1);estimationPeak=zeros(5,1);excess=0;estimationExcess=0;failures=strings(0,1);
    samples=round(cfg.controller.sampleTime/contract.samplePeriod);
    currentCommands=zeros(2,worldCount);worstPhysical=zeros(1,4);worstEstimation=zeros(1,4);
    for world=1:worldCount
        coefficients=2*rand(stream,size(generator,2),1)-1;
        factors=contract.parameterRadius.*(2*rand(stream,4,1)-1);
        previous=contract.currentMeasurement.heldInput;
        if isempty(previous),previous=anchor.inputs(:,1);end
        state=[anchor.states(:,1)+generator*coefficients;contract.point;factors;previous;contract.positionVelocity.point];
        for k=1:count
            % GNSS/IMU vector errors obey Euclidean balls, enclosed by the
            % independent coordinate boxes used for the tube calculation.
            noise=zeros(7,samples);
            for j=1:samples
                angle=2*pi*rand(stream,3,1);
                noise(1:6,j)=reshape([cos(angle).';sin(angle).'],6,1).*contract.noiseRadius(1:6);
                noise(7,j)=contract.noiseRadius(7)*(2*rand(stream)-1);
            end
            context=struct('cfg',cfg,'contract',contract,'reference',[tube.mean(7:11,k);tube.mean(6,k)], ...
                'input',anchor.inputs(:,k),'gain',tube.feedbackGain(:,:,k), ...
                'duration',cfg.controller.sampleTime,'first',k==1);
            try
                point=nrmmJointPrediction('hold',state,noise,context);state=point(1:21);
            catch exception
                failures(end+1,1)=sprintf('world %d hold %d: %s',world,k,exception.identifier); %#ok<AGROW>
                break;
            end
            if k==1,currentCommands(:,world)=point(28:29);end
            difference=abs(state(1:6)-anchor.states(:,k+1));peak=max(peak,difference);
            [gap,coordinate]=max(difference-tube.physicalRadius(:,2*k+1));
            if gap>excess,excess=gap;worstPhysical=[world,k,coordinate,gap];end
            error=abs(state(7:11)-state(1:5));estimationPeak=max(estimationPeak,error);
            [gap,coordinate]=max(error-tube.observerErrorRadius(:,k+1));
            if gap>estimationExcess,estimationExcess=gap;worstEstimation=[world,k,coordinate,gap];end
        end
    end
    result=struct('physicalPeak',peak,'estimationPeak',estimationPeak, ...
        'maximumPhysicalExcess',excess,'maximumEstimationExcess',estimationExcess, ...
        'worstPhysicalWorldHoldCoordinateExcess',worstPhysical, ...
        'worstEstimationWorldHoldCoordinateExcess',worstEstimation, ...
        'sampledContainmentPassed',excess<=1e-9 && estimationExcess<=1e-9 && isempty(failures), ...
        'failures',failures,'currentCommands',currentCommands);
end
