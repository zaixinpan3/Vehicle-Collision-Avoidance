function summary = certifyRecoveryClf(options)
%certifyRecoveryClf Sampled certificate of the recovery CLF (controller/RECOVERY_CLF.md).
% For each reference speed and path curvature it reports the trim offset, the
% linear-loop spectral radius and the Lyapunov-equation residual. It then runs
% the recovery feedback in the controller's own model from a grid of initial
% states: lateral offsets, headings relative to the path (all directions),
% speed factors, lateral velocities and yaw rates. A start passes when the
% errors stay inside the recovery tolerances for 5 s within options.Seconds and
% no model-domain error occurs. On every options.IdentityStride-th start, the
% cost-to-go identity V(f(x,kappa(x))) = V(x) - l(x) is checked.
% This is a sampled certificate on the stated grid, not a proof for all states.
    arguments
        options.OutputDirectory (1,1) string = ""
        options.Speeds (1,:) double = [8,15]
        options.Curvatures (1,:) double = [0,.005]
        options.Lateral (1,:) double = [-150,-100,-60,-30,-15,-5,-1,0,1,5,15,30,60,100,150]
        options.Headings (1,:) double = deg2rad(-180:20:160)
        % Columns: [speed factor; lateral velocity (m/s); yaw rate (rad/s)].
        options.Extras (3,:) double = [1 .5 .8 1.15 1 1 1 1 .8 1.15; 0 0 0 0 1.5 -1.5 0 0 1 -1; 0 0 0 0 0 0 .5 -.5 -.3 .3]
        options.Seconds (1,1) double {mustBePositive} = 150
        options.IdentityStride (1,1) double {mustBePositive,mustBeInteger} = 10
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    tolerance=[.1;pi/180;.1;.05;.01];rows={};terminals={};count=0;
    for speed=options.Speeds
        cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed));h=cfg.controller.sampleTime;
        for curvature=options.Curvatures
            reference=nonlinearBicycleModel.cruise(cfg,curvature);
            terminal=nonlinearBicycleModel.recoveryTerminal(cfg,curvature);
            lane=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200));
            A=terminal.closedLoop;P=terminal.matrix;
            terminals(end+1,:)={speed,curvature,terminal.steeringOffset,terminal.spectralRadius, ...
                norm(A.'*P*A-P+terminal.stageFactor^2)/norm(P),min(eig(P)),max(eig(P))}; %#ok<AGROW>
            for lateral=options.Lateral
                for heading=options.Headings
                    for extra=options.Extras
                        count=count+1;
                        [position,pathHeading]=laneGeometry.referencePose(0,lateral,lane.referenceCurve);
                        x=[position;pathHeading+reference.state(3)+heading;extra(1)*reference.state(4);extra(2);extra(3)];
                        identity=NaN;
                        if mod(count-1,options.IdentityStride)==0
                            [value,~,~,converged]=nonlinearBicycleModel.recoveryValue(x,reference.input,lane,reference,terminal,cfg);
                            u=nonlinearBicycleModel.recoveryInput(x,reference.input,lane,reference,cfg,terminal);
                            stage=sum((terminal.stageFactor*nonlinearBicycleModel.error(x,lane,reference)).^2);
                            next=nonlinearBicycleModel.recoveryValue(nonlinearBicycleModel.sample(x,u,cfg),u,lane,reference,terminal,cfg);
                            identity=abs(next-(value-stage))/max(1,value);
                            if ~converged,identity=Inf;end
                        end
                        u=reference.input;inside=0;recovered=NaN;failure="";peak=zeros(1,4);peak(2)=Inf;
                        for step=1:round(options.Seconds/h)
                            try
                                u=nonlinearBicycleModel.recoveryInput(x,u,lane,reference,cfg,terminal);
                                x=nonlinearBicycleModel.sample(x,u,cfg);
                            catch exception
                                failure=string(exception.identifier);break;
                            end
                            peak=[max(peak(1),abs(x(4)*x(6))),min(peak(2),x(4)),max(peak(3),abs(x(5))),max(peak(4),abs(x(6)))];
                            if all(abs(nonlinearBicycleModel.error(x,lane,reference))<=tolerance),inside=inside+1;else,inside=0;end
                            if inside*h>=5-1e-9,recovered=step*h-5;break;end
                        end
                        rows(end+1,:)={speed,curvature,lateral,heading,extra(1),extra(2),extra(3),recovered,failure, ...
                            peak(1),peak(2),peak(3),peak(4),identity}; %#ok<AGROW>
                    end
                end
            end
        end
    end
    starts=cell2table(rows,'VariableNames',["speed","curvature","lateral","heading","speedFactor", ...
        "lateralVelocity","yawRate","enteredToleranceAt","failure","maxLateralAcceleration","minLongitudinalSpeed", ...
        "maxLateralVelocity","maxYawRate","identityRelativeError"]);
    loops=cell2table(terminals,'VariableNames',["speed","curvature","steeringOffset","spectralRadius", ...
        "lyapunovRelativeResidual","minEigenvalueP","maxEigenvalueP"]);
    passed=isfinite(starts.enteredToleranceAt) & starts.failure=="";checked=isfinite(starts.identityRelativeError);
    summary=struct('starts',height(starts),'passed',nnz(passed),'failures',nnz(starts.failure~=""), ...
        'latestEntrySeconds',max(starts.enteredToleranceAt),'identityChecks',nnz(~isnan(starts.identityRelativeError)), ...
        'maxIdentityRelativeError',max(starts.identityRelativeError(checked)),'loops',loops,'startTable',starts);
    if strlength(options.OutputDirectory)>0
        if ~isfolder(options.OutputDirectory),mkdir(options.OutputDirectory);end
        writetable(starts,fullfile(options.OutputDirectory,'recovery-clf-starts.csv'));
        writetable(loops,fullfile(options.OutputDirectory,'recovery-clf-loops.csv'));
    end
    fprintf('Recovery CLF certificate: %d of %d starts enter and hold the tolerances (latest entry %.2f s), %d domain failures.\n', ...
        summary.passed,summary.starts,summary.latestEntrySeconds,summary.failures);
    fprintf('Cost-to-go identity on %d starts: max relative error %.2e.\n',summary.identityChecks,summary.maxIdentityRelativeError);
    disp(loops);
end
