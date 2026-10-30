function result = analyzeRetainedTubeConservatism(options)
%analyzeRetainedTubeConservatism Run the online measurement-conditioned tube.
% Reuse the bounded-noise/mismatch certificate fixture and track root/next
% widths, actual containment, slack and terminal requests across frames.
% This stationary-target regression is not a steering-required scenario.
% A finite observation cap is diagnostic, not a successful driving outcome.
% No plots, reports or data files are written.
    arguments
        options.Frames (1,1) double {mustBeInteger,mustBePositive}=40
    end
    root=fileparts(fileparts(mfilename('fullpath')));oldPath=path;
    cleanup=onCleanup(@()path(oldPath));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'estimator'));
    f=localFixture();model=f.model;previous=[];runtime=f.runtime;x=f.x;held=f.u;
    trace=struct([]);failure="";failureTime=NaN;drivingFrames=0;
    for k=0:options.Frames-1
        timer=tic;
        try
            [solution,search]=solvePredictiveControl('probe',struct('operation','retained', ...
                'model',model,'previousState',previous,'anchor',f.anchor));
            if solution.terminalControlRequired
                failure="terminalControlRequired: "+search.refinementFailure;
                failureTime=k*f.cfg.controller.sampleTime;break;
            end
            certificate=solution.retainedCertificate;current=certificate.tube.sets{1};
            next=certificate.tube.sets{2};[lo,hi]=bounds(current(1:6));
            if isempty(search.posterior)
                conditioning=search.initialMeasurementConditioning;
            else
                conditioning=search.posterior.measurementConditioning;
            end
            item=struct('time',k*f.cfg.controller.sampleTime,'slack',solution.safety, ...
                'currentRadius',radius(current(1:6)),'nextRadius',radius(next(1:6)), ...
                'containmentExcess',max([lo-x;x-hi]),'terminalGap',certificate.terminal.minimumGap, ...
                'currentGap',certificate.gaps(1),'conditioning',conditioning,'elapsedSeconds',toc(timer));
            trace=[trace,item];drivingFrames=k+1; %#ok<AGROW>
            [runtime,x,output]=localAdvance(runtime,x,held,solution.inputs(:,1),f.cfg,k*f.cfg.controller.sampleTime);
            [ego,~,~,observation]=readControllerInputs(output,[],[],f.cfg);
            held=solution.inputs(:,1);model.initialState=ego.modelState;model.previousInput=held;
            model.stateTime=output.stateTime;model.sampleIndex=k+1;model.targetOnset=false;
            model.uncertainty=predictiveSafetyGeometry.observerUncertainty(ego,observation,model.target);
            model.uncertainty.target=f.model.uncertainty.target;
            previous=struct('retainedCertificate',certificate);
        catch exception
            failure=string(exception.identifier)+": "+exception.message;
            failureTime=k*f.cfg.controller.sampleTime;break;
        end
    end
    result=struct('drivingFrames',drivingFrames,'failure',failure,'failureTime',failureTime, ...
        'trace',trace,'finalTrueState',x,'scope',"online certificate regression; real arithmetic; no avoidance success claim");
    if ~isempty(trace)
        fprintf('Online driving frames %d, maximum slack %.6g, maximum containment excess %.6g m or rad.\n', ...
            drivingFrames,max([trace.slack]),max([trace.containmentExcess]));
        fprintf('Maximum current lateral radius %.6g m/s; maximum next lateral radius %.6g m/s.\n', ...
            max(arrayfun(@(t)t.currentRadius(5),trace)),max(arrayfun(@(t)t.nextRadius(5),trace)));
        fprintf('Median/max controller call %.6g / %.6g s.\n',median([trace.elapsedSeconds]),max([trace.elapsedSeconds]));
    end
    fprintf('Stop at %.6g s: %s\n',failureTime,failure);
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
