function report = runCurvedControllerValidation(options)
%runCurvedControllerValidation Exact-state nonlinear bicycle curve regressions.
% The 100 ms deadline includes online input assembly and the controller.
% It excludes the offline road construction, plant integration and trace audit.
% No observer, physical vehicle model, or global residual proof is represented.
    arguments
        options.Duration (1,1) double {mustBePositive} = 20
        options.OutputDirectory (1,1) string = ""
    end
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root,"controller"),fullfile(root,"config"));
    cfg = collisionAvoidanceControllerConfig();
    cfg.referenceSpeed = 10;
    cfg.controller.sampleTime = 0.1;
    cfg.controller.horizonSteps = 20;
    cfg.model.frontWheelSteeringRateMaximum = 0.5;
    cfg.model.brakingRatioRateMaximum = 2;
    % Diagnostic allowances selected after the recorded failed calibration
    % runs. Every executed ODE trace is independently audited against them.
    cfg.model.plantModelResidualRateBound = [.05;.03;.01;.2;.5;.5];
    roadHalfWidth = 6;
    % Signed distance to a straight line or circle is 1-Lipschitz. This
    % center strip therefore contains every rectangle orientation in the road.
    cfg.model.lateralDomainRadius = roadHalfWidth-hypot(cfg.vehicle.length/2,cfg.vehicle.width/2);
    report = struct("passed",false,"configuration",cfg,"roadHalfWidth",roadHalfWidth, ...
        "scope","Controller-only nonlinear bicycle; exact observations; 100 ms sampling with declared immediate execution; sampled residual and geometry audits", ...
        "trials",struct([]));
    curvatures = [0,1/400,1/100,-1/100];
    for curvature = curvatures
        for targetEnabled = [false,true]
            initialError = zeros(5,1);
            if ~targetEnabled,initialError = [.2;.01;-.5;0;0];end
            trial = runFiniteBicycleDiagnostic(cfg,options.Duration, ...
                RoadCurvature=curvature,InitialFrenetError=initialError,IncludeTarget=targetEnabled, ...
                EnforceModelResidual=true,EnforceRuntimeDeadline=true);
            tail = trial.time>=max(0,options.Duration-3);
            tailError = inf(1,5);
            if any(tail),tailError = max(abs(trial.cruiseError(tail,:)),[],1);end
            roadMargin = localRoadMargin(trial,roadHalfWidth,cfg);
            summary = struct("curvature",curvature,"targetEnabled",targetEnabled, ...
                "completed",~trial.failure.occurred && abs(trial.time(end)-options.Duration)<1e-9, ...
                "sampledRoadMargin",roadMargin,"sampledSeparationMargin",trial.minimumSampledSeparationMargin, ...
                "finalWindowMaximumError",tailError,"maximumFrameMilliseconds",1e3*max(trial.runtime.frameSeconds), ...
                "sampledResidualViolation",trial.sampledModelResidualViolation,"failure",trial.failure,"passed",false);
            summary.passed = summary.completed && trial.runtime.deadlineMet ...
                && roadMargin>0 && trial.minimumSampledSeparationMargin>cfg.collision.clearanceMargin ...
                && trial.sampledModelResidualViolation<=1e-7 ...
                && all(tailError([1,2,3])<=[.2,.02,.5]);
            report.trials = [report.trials;summary];
            fprintf('curvature=%g target=%d completed=%d maxFrame=%.3f ms passed=%d\n', ...
                curvature,targetEnabled,summary.completed,summary.maximumFrameMilliseconds,summary.passed);
            if strlength(options.OutputDirectory)>0
                if ~isfolder(options.OutputDirectory),mkdir(options.OutputDirectory);end
                save(fullfile(options.OutputDirectory,sprintf('curve-%g-target-%d.mat',curvature,targetEnabled)),"trial");
                save(fullfile(options.OutputDirectory,'curve-validation.mat'),"report");
            end
        end
    end
    report.passed = all([report.trials.passed]);
    if strlength(options.OutputDirectory)>0
        save(fullfile(options.OutputDirectory,'curve-validation.mat'),"report");
        fid = fopen(fullfile(options.OutputDirectory,'curve-validation.json'),'w');
        cleanup = onCleanup(@() fclose(fid));
        fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    end
end

function margin = localRoadMargin(trial,halfWidth,cfg)
    state = trial.plantTrace.state;
    points = cfg.vehicle.length/2*[1,1,-1,-1];
    lateral = cfg.vehicle.width/2*[1,-1,-1,1];
    x = state(:,1)+cos(state(:,3))*points-sin(state(:,3))*lateral;
    y = state(:,2)+sin(state(:,3))*points+cos(state(:,3))*lateral;
    if trial.roadCurvature==0
        distance = abs(y);
    else
        radius = 1/trial.roadCurvature;
        distance = abs(hypot(x,y-radius)-abs(radius));
    end
    margin = halfWidth-max(distance,[],"all");
end
