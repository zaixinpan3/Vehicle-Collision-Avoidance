function summary = runSpeedBenchmark(sourceRoot,outputFile,options)
%runSpeedBenchmark Controller time per frame with a realistic target onset (study script).
% For the seven campaign geometries at 8 and 15 m/s, the ego first drives
% target-free on its lane for options.PreRollFrames frames (it starts that
% far back at the reference speed), and the target appears at the
% scenario's own t = 0 state, within the 50-m range. The plant is the
% declared model (RK4 holds of nonlinearBicycleModel.sample) with exact
% states. An encounter is recovered when the transverse errors stay within
% [0.1 m, 1 deg, 0.1 m/s, 0.05 m/s, 0.01 rad/s] for 1 s after the scenario's
% impact time, as in the campaign; it stops at options.Frames frames after
% the onset. Before the encounters, a start-up warm-up runs one synthetic
% encounter through the public interface (lane following, then a head-on
% target that appears and is tracked), as collisionAvoidanceController's
% "warmUp" action does, so that it also applies to versions without that
% action; its first call (the cold start) and its total are timed
% separately. Writes a JSON summary.
    arguments
        sourceRoot (1,1) string
        outputFile (1,1) string
        options.PreRollFrames (1,1) double = 40
        options.Frames (1,1) double = 400
        options.Names (1,:) string = ["headOn","acceleratingHeadOn","brakingLead","crossing","turningCrossing","curvedHeadOn","curvedCrossing"]
        options.Speeds (1,:) double = [8,15]
        options.Solver struct = struct([])
    end
    addpath(fullfile(sourceRoot,'controller'),fullfile(sourceRoot,'config'),fullfile(sourceRoot,'scripts'));
    tolerance=[.1;pi/180;.1;.05;.01];shape=[2.4;.95;0;0];
    % Start-up warm-up (cold start is its first call).
    t=tic;cold=localWarmUp(struct('referenceSpeed',8,'controller',struct('horizonSteps',8))); warmUp=toc(t);
    runs=struct([]);
    for speed=options.Speeds
        for name=options.Names
            cfgStruct=struct('referenceSpeed',speed,'controller',struct('horizonSteps',8+8*(speed==15)));
            if ~isempty(options.Solver),cfgStruct.solver=options.Solver;end
            [x0,q0,road,cfg,impact]=collisionThreatScenario(name,cfgStruct);
            h=cfg.controller.sampleTime;P=options.PreRollFrames;x=x0;
            curve=struct('origin',[0;0],'heading',0,'curvature',0,'length',1e4);
            if isfield(road,'referenceCurve'),curve=road.referenceCurve;end
            [p,heading]=laneGeometry.referencePose(-x0(4)*P*h,0,curve);[~,heading0]=laneGeometry.referencePose(0,0,curve);
            x(1:2)=p;x(3)=heading+(x0(3)-heading0);
            collisionAvoidanceController("resetNominalTrajectory");
            prior=[];input=[0;0];times=zeros(1,0);horizons=zeros(1,0);restarts=0;solves=zeros(1,0);
            outcome="notRecovered";least=Inf;inside=0;failure="";
            for frame=1:P+options.Frames
                time=(frame-1-P)*h;q=[];
                if time>=0
                    q=predictiveSafetyGeometry.predictTarget(q0,time);
                    if norm(q(1:2)-x(1:2))>cfg.collision.encounterRangeMeters,q=[];end
                end
                target=[];if ~isempty(q),target=localTarget(q);end
                t=tic;
                try
                    [command,~,problem,prior]=collisionAvoidanceController(localEgo(x,input,(frame-1)*h),target,road,cfg,prior);
                catch exception
                    times(end+1)=toc(t); %#ok<AGROW>
                    outcome="noSolution";failure=string(exception.message);break;
                end
                times(end+1)=toc(t); %#ok<AGROW>
                s=problem.metadata.search;horizons(end+1)=problem.metadata.horizonSteps; %#ok<AGROW>
                solves(end+1)=s.solverCalls; %#ok<AGROW>
                restarts=restarts+(isfield(s,'potentialFieldRestarted') && s.potentialFieldRestarted);
                input=command.actuatorInput;x=nonlinearBicycleModel.sample(x,input,cfg);
                if time>=0
                    qt=predictiveSafetyGeometry.predictTarget(q0,time+h);
                    least=min(least,predictiveSafetyGeometry.rectangle(x(1:3),shape,qt(1:3),qt(8:11)));
                    if least<=0,outcome="collision";break;end
                    e=nonlinearBicycleModel.error(x,problem.model.lane,problem.model.nominalReference);
                    if time+h>=impact && all(abs(e)<=tolerance),inside=inside+1;else,inside=0;end
                    if inside*h>=1,outcome="recovered";break;end
                end
            end
            onset=P+1;
            run=struct('name',name,'speed',speed,'outcome',outcome,'failure',failure, ...
                'framesAfterOnset',numel(times)-P,'leastClearance',least, ...
                'preRollMaximumMs',1000*max(times(2:min(P,end))),'onsetMs',1000*times(min(onset,end)), ...
                'afterOnsetMedianMs',1000*median(times(onset+1:end)),'afterOnsetP95Ms',1000*localQuantile(times(onset+1:end),.95), ...
                'afterOnsetMaxMs',1000*max([times(onset+1:end),0]),'framesOver50Ms',nnz(times(onset:end)>.05), ...
                'horizonMedian',median(horizons(onset:end)),'horizonMax',max(horizons(onset:end)), ...
                'restarts',restarts,'solverCallsMedian',median(solves(onset:end)),'timesMs',1000*times);
            fprintf('%-18s %2d m/s: %-12s onset %6.1f ms, after: median %5.1f p95 %6.1f max %6.1f ms, >50ms %3d frames, horizon median %3.0f max %3d\n', ...
                name,speed,outcome,run.onsetMs,run.afterOnsetMedianMs,run.afterOnsetP95Ms,run.afterOnsetMaxMs,run.framesOver50Ms,run.horizonMedian,run.horizonMax);
            if isempty(runs),runs=run;else,runs(end+1)=run;end %#ok<AGROW>
        end
    end
    allTimes=[runs.timesMs];
    summary=struct('sourceRoot',sourceRoot,'coldStartMs',1000*cold,'warmUpMs',1000*warmUp,'runs',runs, ...
        'recovered',nnz([runs.outcome]=="recovered"),'collisions',nnz([runs.outcome]=="collision"), ...
        'noSolution',nnz([runs.outcome]=="noSolution"), ...
        'onsetMedianMs',median([runs.onsetMs]),'onsetMaxMs',max([runs.onsetMs]), ...
        'allFramesMedianMs',median(allTimes),'allFramesP95Ms',localQuantile(allTimes,.95),'allFramesP99Ms',localQuantile(allTimes,.99), ...
        'allFramesMaxMs',max(allTimes),'framesOver50Ms',nnz(allTimes>50),'frames',numel(allTimes));
    fprintf('cold start %.0f ms (warm-up %.0f ms); recovered %d of %d; onset median %.1f max %.1f ms; all frames median %.1f p95 %.1f p99 %.1f max %.1f ms; %d of %d frames over 50 ms\n', ...
        summary.coldStartMs,summary.warmUpMs,summary.recovered,numel(runs),summary.onsetMedianMs,summary.onsetMaxMs, ...
        summary.allFramesMedianMs,summary.allFramesP95Ms,summary.allFramesP99Ms,summary.allFramesMaxMs,summary.framesOver50Ms,summary.frames);
    fid=fopen(outputFile,'w');fprintf(fid,'%s\n',jsonencode(summary));fclose(fid);
end

function cold=localWarmUp(cfg)
    % The warm-up encounter of collisionAvoidanceController("warmUp", cfg).
    cfg=collisionAvoidanceControllerConfig(cfg);h=cfg.controller.sampleTime;
    road=struct('centerline',[-100,0;1000,0],'lateralClearance',[8.5344;12.192]);
    x=[0;0;0;cfg.referenceSpeed;0;0];input=[0;0];state=[];cold=NaN;
    q=[40;0;pi;8;0;0;cfg.target.rearAxleDistance;cfg.target.defaultLength/2;cfg.target.defaultWidth/2;0;0];
    for frame=1:24
        target=[];
        if frame>4
            p=predictiveSafetyGeometry.predictTarget(q,(frame-5)*h);
            target=struct('targetPositionInertial',p(1:2),'targetVelocityInertial',p(4)*[cos(p(3));sin(p(3))],'targetYawInertial',p(3));
        end
        t=tic;
        try
            [command,~,~,state]=collisionAvoidanceController(localEgo(x,input,(frame-1)*h),target,road,cfg,state);
        catch
            return;
        end
        if frame==1,cold=toc(t);end
        input=command.actuatorInput;x=nonlinearBicycleModel.sample(x,input,cfg);
    end
end

function ego=localEgo(x,input,time)
    ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6), ...
        'heldActuatorInput',input,'stateTime',time);
end

function target=localTarget(q)
    velocity=q(4)*[cos(q(3)+q(6));sin(q(3)+q(6))];
    target=struct('targetPositionInertial',q(1:2),'targetVelocityInertial',velocity, ...
        'targetYawInertial',q(3),'targetTangentialAcceleration',q(5),'targetSideslip',q(6), ...
        'targetRearAxleDistance',q(7),'targetLength',2*q(8),'targetWidth',2*q(9));
end

function value=localQuantile(a,p)
    a=sort(a(:));if isempty(a),value=NaN;return;end
    x=p*(numel(a)-1);i=floor(x);f=x-i;
    if i+1>=numel(a),value=a(end);else,value=a(i+1)+(a(i+2)-a(i+1))*f;end
end
