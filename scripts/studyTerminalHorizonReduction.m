function study = studyTerminalHorizonReduction(campaignDirectory,options)
%studyTerminalHorizonReduction How many free prediction steps a closed-loop terminal set needs.
% Offline study; the controller is unchanged. A closed-loop terminal set is
% the set of states from which one fixed guidance policy, continued forever,
% never collides with a target that keeps constant tangential acceleration and
% sideslip. If that continuation is condensed onto the endpoint state, it adds
% rows but no decision steps, so the free horizon only has to reach the set.
%
% Policies: nominal path guidance (nonlinearBicycleModel.nominalFeedback)
% toward the path (offset 0) or toward a parallel lane centre at the given
% lateral offsets. (A former LQR-law option was removed with the LQR
% design; see studyClfOptimizationTerminal for the CLF-constrained
% optimization itself.) With a finite TerminalValueLimit the state must also
% satisfy V <= limit. Continuations that leave the model domain do not
% qualify. For every recorded ego state k of a campaign run, the study
% finds the smallest j>=k such that, from the recorded state j, some policy
% reaches a model-based no-collision certificate without touching the target.
% j-k is the free horizon that would have sufficed along the recorded
% trajectory, a proxy for the planned one. It is compared with the horizon
% the separating-terminal controller actually used in that frame.
%
% A continuation that comes closer than safetyMarginMeters to the target or
% puts a rectangle corner off the road does not qualify.
% Certificates (ego converged to its policy path, dSafe = circumscribed
% radii + safetyMarginMeters) are those of studyClosedLoopTerminalFeasibility,
% applied to the offset path. On the curved road the infinite-time set is
% empty under the contract; there the study also reports a finite validity
% window: no contact for WindowSeconds under the policy.

    arguments
        campaignDirectory (1,1) string
        options.Offsets (1,:) double = [0 3.6576 -3.6576 7.3152]
        options.MaximumTailSteps (1,1) double {mustBePositive,mustBeInteger} = 2000
        options.WindowSeconds (1,1) double {mustBePositive} = 10
        options.Policy (1,1) string {mustBeMember(options.Policy,"guidance")} = "guidance"
        options.TerminalValueLimit (1,1) double {mustBePositive} = Inf
        options.OutputFile (1,1) string = ""
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'));
    names=["headOn","acceleratingHeadOn","brakingLead","crossing","turningCrossing","curvedHeadOn","curvedCrossing"];
    families=struct('name',{"pathOnly","pathAndLanes"},'offsets',{0,options.Offsets});
    summary=struct([]);
    for speed=[8,15]
        for name=names
            result=jsondecode(fileread(fullfile(campaignDirectory,sprintf('speed%g-%s.json',speed,name)))).results;
            trace=result.trace;
            configuration=struct('referenceSpeed',speed,'controller',struct('horizonSteps',8+8*(speed==15)));
            [~,q0,road,cfg]=collisionThreatScenario(name,configuration);
            [lane,reference]=localGeometry(road,cfg);
            curved=reference.curvature~=0;
            count=numel(trace);
            % Frames before the recorded recovery (the encounter).
            last=count;
            if isfield(result.recovery,'entryTimeSeconds') && ~isempty(result.recovery.entryTimeSeconds)
                last=min(count,find([trace.time]>=result.recovery.entryTimeSeconds,1));
            end
            used=[trace(1:last).horizonSteps];
            % A parallel lane qualifies if the body stays on the road.
            half=hypot(cfg.vehicle.length,cfg.vehicle.width)/2;
            offsets=options.Offsets(options.Offsets+half<=road.lateralClearance(2) ...
                & -options.Offsets+half<=road.lateralClearance(1));
            safe=false(numel(offsets),last);clfSteps=0;clfViolations=0;domainExits=0;
            for k=1:last
                input=reference.input;if k>1,input=trace(k-1).input(:);end
                for m=1:numel(offsets)
                    [safe(m,k),steps,violations,domain]=localSafe(trace(k).state(:),input,trace(k).time,q0,lane,reference,cfg, ...
                        offsets(m),options.MaximumTailSteps,curved,options.WindowSeconds,road,options.Policy);
                    clfSteps=clfSteps+steps;clfViolations=clfViolations+violations;domainExits=domainExits+domain;
                    % Optionally the endpoint must also lie in the CLF sublevel set.
                    safe(m,k)=safe(m,k) && nonlinearBicycleModel.nominalValue(trace(k).state(:),lane,reference)<=options.TerminalValueLimit;
                end
            end
            row=struct('speed',speed,'scenario',name,'encounterFrames',last, ...
                'separatingHorizonMedian',median(used),'separatingHorizonMaximum',max(used), ...
                'policy',options.Policy,'clfViolationFraction',clfViolations/max(clfSteps,1), ...
                'domainExits',domainExits);
            for f=1:numel(families)
                member=any(safe(ismember(offsets,families(f).offsets),:),1);
                free=NaN(1,last);
                for k=1:last
                    j=find(member(k:end),1);
                    if ~isempty(j),free(k)=j-1;end
                end
                row.(families(f).name+"Median")=median(free,'omitnan');
                row.(families(f).name+"Maximum")=max(free);
                row.(families(f).name+"Unreached")=nnz(isnan(free));
            end
            summary=[summary,row]; %#ok<AGROW>
            fprintf(['HORIZON %2d %-19s frames=%4d separating(med/max)=%g/%g  pathOnly(med/max/unreached)=%g/%g/%d  ' ...
                'pathAndLanes(med/max/unreached)=%g/%g/%d%s\n'],speed,name,last,row.separatingHorizonMedian, ...
                row.separatingHorizonMaximum,row.pathOnlyMedian,row.pathOnlyMaximum,row.pathOnlyUnreached, ...
                row.pathAndLanesMedian,row.pathAndLanesMaximum,row.pathAndLanesUnreached, ...
                string(repmat(sprintf(' [curved: %g-s window]',options.WindowSeconds),1,curved)));
            fprintf('POLICY %s CLF-decrease violations %.4f of continuation steps, domain exits %d\n', ...
                options.Policy,row.clfViolationFraction,domainExits);
        end
    end
    study=struct('scope',"Offline study on recorded exact-observation trajectories; recorded states proxy the plans", ...
        'campaignDirectory',campaignDirectory,'offsets',options.Offsets,'windowSeconds',options.WindowSeconds, ...
        'summary',summary);
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

function [safe,steps,violations,domain]=localSafe(x,input,time,q0,lane,reference,cfg,offset,maximum,curved,window,road,policy)
    % Guidance toward a parallel path: evaluate the path guidance on the
    % state shifted by -offset along the local path normal (exact on a
    % straight road, a first-order approximation on the curved one).
    h=cfg.controller.sampleTime;nominal=nonlinearBicycleModel.nominalGuidanceParameters(cfg,reference.curvature);
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    safe=false;steps=0;violations=0;domain=0;
    scales=[cfg.clf.lateralPositionErrorScale;cfg.clf.headingErrorScale;cfg.clf.speedErrorScale; ...
        cfg.clf.lateralVelocityErrorScale;cfg.clf.yawRateErrorScale];
    if curved,maximum=round(window/h);end
    for step=0:maximum
        q=predictiveSafetyGeometry.predictTarget(q0,time+step*h);
        % The continuation keeps the planning clearance and stays on the road.
        if predictiveSafetyGeometry.rectangle(x(1:3),shape,q(1:3),q(8:11))<cfg.collision.safetyMarginMeters,return;end
        rotation=[cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
        corners=laneGeometry.project(x(1:2)+rotation*(shape(3:4)+shape(1:2).*[1,1,-1,-1;1,-1,1,-1]),lane);
        if any(corners.lateralPosition(:)>road.lateralClearance(2) | corners.lateralPosition(:)<-road.lateralClearance(1)),return;end
        if ~curved && localCertificate(x,q,lane,reference,cfg,shape,offset),safe=true;return;end
        if step==maximum,break;end
        projection=laneGeometry.project(x(1:2),lane);
        shifted=x;shifted(1:2)=x(1:2)-offset*[-sin(projection.heading);cos(projection.heading)];
        input=nonlinearBicycleModel.nominalFeedback(shifted,input,lane,reference,cfg,nominal);
        try
            next=nonlinearBicycleModel.sample(x,input,cfg);
        catch
            domain=1;return;
        end
        steps=steps+1;
        value=nonlinearBicycleModel.nominalValue(x,lane,reference);
        successor=nonlinearBicycleModel.nominalValue(next,lane,reference);
        e=nonlinearBicycleModel.error(x,lane,reference);
        violations=violations+(successor>reference.contraction*value+1e-9);
        x=next;
    end
    safe=curved;
end

function certified=localCertificate(x,q,lane,reference,cfg,shape,offset)
    certified=false;
    projection=laneGeometry.project(x(1:2),lane);
    shifted=x;shifted(1:2)=x(1:2)-offset*[-sin(projection.heading);cos(projection.heading)];
    e=nonlinearBicycleModel.error(shifted,lane,reference);
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
