function study = studyClfOptimizationTerminal(campaignDirectory,options)
%studyClfOptimizationTerminal Terminal set of the CLF-constrained optimization controller.
% Offline study; the controller is unchanged. The candidate terminal set is
% the set of states from which the controller itself, run without a target
% (the CLF-constrained optimization with its road constraint and nothing
% else), never collides with a target that keeps constant tangential
% acceleration and sideslip. The continuation is simulated by calling
% collisionAvoidanceController(ego,[],road,cfg,prior) every sample on the RK4
% plant, with the target predicted by its model.
%
% A continuation qualifies if it keeps at least safetyMarginMeters from the
% target, keeps every rectangle corner on the road, returns a solution at
% every sample, and reaches the no-collision certificate of
% studyClosedLoopTerminalFeasibility (straight road). On the curved road,
% where the infinite-time set is empty under the contract, it must instead
% stay clear for WindowSeconds.
%
% For each sampled recorded state k the study reports whether it lies in the
% set and how long the membership check took. The free horizon of k is the
% number of samples to the first later sampled state in the set (resolution
% FrameStride). Recorded states proxy the planned ones.

    arguments
        campaignDirectory (1,1) string
        options.FrameStride (1,1) double {mustBePositive,mustBeInteger} = 2
        options.MaximumTailSteps (1,1) double {mustBePositive,mustBeInteger} = 600
        options.WindowSeconds (1,1) double {mustBePositive} = 10
        options.Scenarios (1,:) string = ["headOn","acceleratingHeadOn","brakingLead","crossing", ...
            "turningCrossing","curvedHeadOn","curvedCrossing"]
        options.Speeds (1,:) double = [8 15]
        options.OutputFile (1,1) string = ""
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'));
    summary=struct([]);
    for speed=options.Speeds
        for name=options.Scenarios
            result=jsondecode(fileread(fullfile(campaignDirectory,sprintf('speed%g-%s.json',speed,name)))).results;
            trace=result.trace;
            configuration=struct('referenceSpeed',speed,'controller',struct('horizonSteps',8+8*(speed==15)));
            [~,q0,road,cfg]=collisionThreatScenario(name,configuration);
            [lane,reference]=localGeometry(road,cfg);
            curved=reference.curvature~=0;
            last=numel(trace);
            if isfield(result.recovery,'entryTimeSeconds') && ~isempty(result.recovery.entryTimeSeconds)
                last=min(last,find([trace.time]>=result.recovery.entryTimeSeconds,1));
            end
            frames=1:options.FrameStride:last;
            member=false(size(frames));seconds=zeros(size(frames));tail=NaN(size(frames));
            reason=strings(size(frames));
            for i=1:numel(frames)
                k=frames(i);input=[0;0];if k>1,input=trace(k-1).input(:);end
                wall=tic;
                [member(i),tail(i),reason(i)]=localMember(trace(k).state(:),input,trace(k).time,q0,lane, ...
                    reference,road,cfg,options.MaximumTailSteps,curved,options.WindowSeconds);
                seconds(i)=toc(wall);
            end
            free=NaN(size(frames));
            for i=1:numel(frames)
                j=find(member(i:end),1);
                if ~isempty(j),free(i)=frames(i+j-1)-frames(i);end
            end
            used=[trace(1:last).horizonSteps];
            reasons=unique(reason(~member));
            counts=arrayfun(@(r)nnz(reason==r),reasons);
            row=struct('speed',speed,'scenario',name,'sampledFrames',numel(frames), ...
                'memberFrames',nnz(member),'separatingHorizonMaximum',max(used), ...
                'freeMedian',median(free,'omitnan'),'freeMaximum',max(free),'freeUnreached',nnz(isnan(free)), ...
                'tailMedian',median(tail(member),'omitnan'),'tailMaximum',max(tail(member)), ...
                'checkSecondsMedian',median(seconds),'checkSecondsMaximum',max(seconds), ...
                'rejections',{cellstr(reasons+":"+string(counts))});
            summary=[summary,row]; %#ok<AGROW>
            fprintf(['CLFOPT %2d %-19s sampled=%3d member=%3d separating max=%3d  free(med/max/unreached)=%g/%g/%d  ' ...
                'tail(med/max)=%g/%g  check s(med/max)=%.2f/%.2f  rejected: %s%s\n'],speed,name,numel(frames), ...
                nnz(member),max(used),row.freeMedian,row.freeMaximum,row.freeUnreached,row.tailMedian,row.tailMaximum, ...
                row.checkSecondsMedian,row.checkSecondsMaximum,strjoin(reasons+"="+string(counts),', '), ...
                string(repmat(sprintf(' [curved: %g-s window]',options.WindowSeconds),1,curved)));
        end
    end
    study=struct('scope',"Offline study; recorded states proxy the plans; controller unchanged", ...
        'policy',"collisionAvoidanceController without a target",'campaignDirectory',campaignDirectory, ...
        'frameStride',options.FrameStride,'windowSeconds',options.WindowSeconds,'summary',summary);
    if strlength(options.OutputFile)>0
        fid=fopen(options.OutputFile,'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
        fprintf(fid,'%s\n',jsonencode(study));
    end
end

function [lane,reference]=localGeometry(road,cfg)
    ego=struct('position',[0;0],'yaw',0,'speed',cfg.referenceSpeed);
    collisionAvoidanceController("resetNominalTrajectory");
    [~,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
    lane=problem.model.lane;reference=problem.model.nominalReference;
end

function [member,steps,reason]=localMember(x,input,time,q0,lane,reference,road,cfg,maximum,curved,window)
    h=cfg.controller.sampleTime;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    member=false;steps=NaN;reason="";
    if curved,maximum=round(window/h);end
    collisionAvoidanceController("resetNominalTrajectory");prior=[];
    for step=0:maximum
        q=predictiveSafetyGeometry.predictTarget(q0,time+step*h);
        if predictiveSafetyGeometry.rectangle(x(1:3),shape,q(1:3),q(8:11))<cfg.collision.safetyMarginMeters
            reason="collision";return;
        end
        rotation=[cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
        corners=laneGeometry.project(x(1:2)+rotation*(shape(3:4)+shape(1:2).*[1,1,-1,-1;1,-1,1,-1]),lane);
        if any(corners.lateralPosition(:)>road.lateralClearance(2) | corners.lateralPosition(:)<-road.lateralClearance(1))
            reason="offRoad";return;
        end
        if ~curved && localCertificate(x,q,lane,reference,cfg,shape),member=true;steps=step;return;end
        if step==maximum,break;end
        ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6), ...
            'longitudinalVelocity',x(4),'stateTime',time+step*h,'heldActuatorInput',input);
        try
            [command,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,prior);
            input=command.actuatorInput;
            x=nonlinearBicycleModel.sample(x,input,cfg);
        catch exception
            reason=string(extractAfter(exception.identifier,":"));if reason=="",reason="error";end
            return;
        end
    end
    if curved,member=true;steps=maximum;else,reason="uncertified";end
end

function certified=localCertificate(x,q,lane,reference,cfg,shape)
    certified=false;
    e=nonlinearBicycleModel.error(x,lane,reference);
    if any(abs(e)>[.2;.02;.2;.1;.02]),return;end
    safe=norm(shape(1:2))+norm(shape(3:4))+norm(q(8:9))+norm(q(10:11))+cfg.collision.safetyMarginMeters;
    p=x(1:2);c=cos(x(3));s=sin(x(3));v=[c,-s;s,c]*x(4:5);tau=v/max(norm(v),eps);
    heading=q(3)+q(6);direction=[cos(heading);sin(heading)];speed=max(q(4),0);
    curvature=sin(q(6))/q(7);
    if speed>0 && abs(curvature)>1e-9
        centre=q(1:2)+[-sin(heading);cos(heading)]/curvature;
        certified=(p-centre).'*tau>=0 && norm(p-centre)-1/abs(curvature)>=safe;
        return;
    end
    delta=q(1:2)-p;w=speed*direction;
    if norm(delta)<safe || delta.'*(w-v)<0,return;end
    certified=speed<=0 || q(5)==0 ...
        || (q(5)>0 && delta.'*direction>=0 && speed>=v.'*direction) ...
        || (q(5)<0 && delta.'*direction<=0 && v.'*direction>=speed);
end
