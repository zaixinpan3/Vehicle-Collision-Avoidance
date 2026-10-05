function study = studyClosedLoopTerminalFeasibility(campaignDirectory,options)
%studyClosedLoopTerminalFeasibility Offline real-time study of a closed-loop terminal set.
% The candidate terminal set is the set of states from which the nominal path
% guidance alone (nonlinearBicycleModel.nominalFeedback) never collides with a
% target that keeps constant tangential acceleration and sideslip. This study
% does not change the controller. For recorded ego states of a campaign it
% measures
%   A. how many nominal-guidance steps a rollout needs before a model-based
%      certificate proves that no later collision occurs, and whether the
%      rollout collides first;
%   B. the per-node cost of rolling out, linearizing and differentiating the
%      nominal guidance;
%   C. the conic solve time of the present formulation versus horizon length.
% The certificate requires the ego to have converged to the path (the guidance
% then holds cruise on the path) and one of the following, with the circumscribed
% clearance dSafe = R_E + R_C + safetyMarginMeters:
%   straight road, beta~=0: the target stays on a circle with centre c and
%     radius rho; (p-c)'tau>=0 and |p-c|-rho>=dSafe;
%   straight road, beta==0: separating, |p-q|>=dSafe and the sign condition
%     that keeps the separating rate nondecreasing under the target's constant
%     acceleration until it stops;
%   curved road: the ego path circle and the target's whole future path (its
%     circle, or the remaining ray or segment) are at least dSafe apart, or
%     the target is outside the road circle by dSafe and moving outward.

    arguments
        campaignDirectory (1,1) string
        options.FrameStride (1,1) double {mustBePositive,mustBeInteger} = 5
        options.MaximumTailSteps (1,1) double {mustBePositive,mustBeInteger} = 4000
        options.HorizonLengths (1,:) double = [64 128 256 512 1024]
        options.OutputFile (1,1) string = ""
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'));
    names=["headOn","acceleratingHeadOn","brakingLead","crossing","turningCrossing","curvedHeadOn","curvedCrossing"];
    rows=struct('speed',{},'scenario',{},'frame',{},'time',{},'tailSteps',{},'certificate',{}, ...
        'collides',{},'minimumDistance',{},'rolloutSeconds',{});
    summary=struct('speed',{},'scenario',{},'frames',{},'certifiedFrames',{},'collidingFrames',{}, ...
        'uncertifiedFrames',{},'tailMedian',{},'tailP95',{},'tailMaximum',{},'requiredTotalMaximum',{}, ...
        'rolloutMillisecondsP95',{});
    for speed=[8,15]
        for name=names
            file=fullfile(campaignDirectory,sprintf('speed%g-%s.json',speed,name));
            result=jsondecode(fileread(file)).results;trace=result.trace;
            configuration=struct('referenceSpeed',speed,'controller',struct('horizonSteps',8+8*(speed==15)));
            [~,q0,road,cfg]=collisionThreatScenario(name,configuration);
            [lane,reference]=localGeometry(road,cfg);
            frames=1:options.FrameStride:numel(trace);
            local=rows([]);
            for k=frames
                x=trace(k).state(:);input=reference.input;
                if k>1,input=trace(k-1).input(:);end
                wall=tic;r=localTail(x,input,trace(k).time,q0,lane,reference,cfg,options.MaximumTailSteps);
                r.rolloutSeconds=toc(wall);
                local(end+1)=struct('speed',speed,'scenario',name,'frame',k,'time',trace(k).time, ...
                    'tailSteps',r.steps,'certificate',r.certificate,'collides',r.collides, ...
                    'minimumDistance',r.minimumDistance,'rolloutSeconds',r.rolloutSeconds); %#ok<AGROW>
            end
            rows=[rows,local]; %#ok<AGROW>
            certified=~[local.collides] & [local.certificate]~="none";
            tails=[local(certified).tailSteps];
            % Total nodes if the free part had to reach the first later
            % sampled state whose nominal rollout is certified.
            total=NaN(1,numel(local));
            for i=1:numel(local)
                j=find(certified(i:end),1);
                if ~isempty(j),j=i+j-1;total(i)=local(j).frame-local(i).frame+local(j).tailSteps;end
            end
            summary(end+1)=struct('speed',speed,'scenario',name,'frames',numel(local), ...
                'certifiedFrames',nnz(certified),'collidingFrames',nnz([local.collides]), ...
                'uncertifiedFrames',nnz(~[local.collides] & [local.certificate]=="none"), ...
                'tailMedian',localQuantile(tails,.5),'tailP95',localQuantile(tails,.95), ...
                'tailMaximum',localQuantile(tails,1),'requiredTotalMaximum',max([total,-Inf]), ...
                'rolloutMillisecondsP95',1e3*localQuantile([local.rolloutSeconds],.95)); %#ok<AGROW>
            fprintf('TAIL %2d %-19s frames=%3d certified=%3d colliding=%3d uncertified=%3d tail(med/p95/max)=%g/%g/%g total<=%g rollout p95=%.1f ms\n', ...
                speed,name,summary(end).frames,summary(end).certifiedFrames,summary(end).collidingFrames, ...
                summary(end).uncertifiedFrames,summary(end).tailMedian,summary(end).tailP95,summary(end).tailMaximum, ...
                summary(end).requiredTotalMaximum,summary(end).rolloutMillisecondsP95);
        end
    end
    study=struct('scope',"Offline study on recorded exact-observation trajectories; no controller change", ...
        'campaignDirectory',campaignDirectory,'frames',rows,'summary',summary, ...
        'nodeCost',localNodeCost(),'solveScaling',localSolveScaling(options.HorizonLengths));
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

function r=localTail(x,input,time,q0,lane,reference,cfg,maximum)
    h=cfg.controller.sampleTime;nominal=nonlinearBicycleModel.nominalGuidanceParameters(cfg,reference.curvature);
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    r=struct('steps',NaN,'certificate',"none",'collides',false,'minimumDistance',Inf);
    for step=0:maximum
        q=predictiveSafetyGeometry.predictTarget(q0,time+step*h);
        distance=predictiveSafetyGeometry.rectangle(x(1:3),shape,q(1:3),q(8:11));
        r.minimumDistance=min(r.minimumDistance,distance);
        if distance<=0,r.collides=true;r.steps=step;return;end
        certificate=localCertificate(x,q,lane,reference,cfg,shape);
        if certificate~="none",r.steps=step;r.certificate=certificate;return;end
        if step==maximum,break;end
        input=nonlinearBicycleModel.nominalFeedback(x,input,lane,reference,cfg,nominal);
        x=nonlinearBicycleModel.sample(x,input,cfg);
    end
end

function certificate=localCertificate(x,q,lane,reference,cfg,shape)
    certificate="none";
    e=nonlinearBicycleModel.error(x,lane,reference);
    if any(abs(e)>[.2;.02;.2;.1;.02]),return;end
    safe=norm(shape(1:2))+norm(shape(3:4))+norm(q(8:9))+norm(q(10:11))+cfg.collision.safetyMarginMeters;
    p=x(1:2);c=cos(x(3));s=sin(x(3));v=[c,-s;s,c]*x(4:5);
    heading=q(3)+q(6);direction=[cos(heading);sin(heading)];speed=max(q(4),0);
    curvature=sin(q(6))/q(7);
    stopped=speed<=0;stop=Inf;
    if q(5)<0,stop=speed^2/(2*-q(5));end
    if reference.curvature==0
        tau=v/max(norm(v),eps);
        if ~stopped && abs(curvature)>1e-9
            centre=q(1:2)+[-sin(heading);cos(heading)]/curvature;
            if (p-centre).'*tau>=0 && norm(p-centre)-1/abs(curvature)>=safe,certificate="targetCircle";end
            return;
        end
        delta=q(1:2)-p;w=speed*direction;
        if norm(delta)<safe || delta.'*(w-v)<0,return;end
        if stopped || q(5)==0,certificate="separatingConstantVelocity";
        elseif q(5)>0 && delta.'*direction>=0 && speed>=v.'*direction,certificate="separatingAcceleratingAhead";
        elseif q(5)<0 && delta.'*direction<=0 && v.'*direction>=speed,certificate="separatingBrakingBehind";
        end
        return;
    end
    curve=lane.referenceCurve;
    roadCentre=curve.origin+[-sin(curve.heading);cos(curve.heading)]/curve.curvature;
    roadRadius=1/abs(curve.curvature);
    if ~stopped && abs(curvature)>1e-9
        centre=q(1:2)+[-sin(heading);cos(heading)]/curvature;rho=1/abs(curvature);
        gap=norm(centre-roadCentre);
        if gap>=roadRadius+rho,clearance=gap-roadRadius-rho;
        elseif gap<=abs(roadRadius-rho),clearance=abs(roadRadius-rho)-gap;
        else,clearance=0;
        end
        if clearance>=safe,certificate="disjointCircles";end
        return;
    end
    radial=q(1:2)-roadCentre;
    if norm(radial)>=roadRadius+safe && (stopped || radial.'*direction>=0),certificate="outsideRoadCircle";return;end
    if isfinite(stop)
        points=q(1:2)+direction*linspace(0,stop,200);
        if all(abs(vecnorm(points-roadCentre)-roadRadius)>=safe),certificate="stopSegmentClear";end
    end
end

function cost=localNodeCost()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',15));
    reference=nonlinearBicycleModel.cruise(cfg,0);
    lane=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',0,'length',200));
    nominal=nonlinearBicycleModel.nominalGuidanceParameters(cfg,0);
    x=[0;1;.05;reference.state(4:6)];u=reference.input;count=400;
    wall=tic;for i=1:count,u=nonlinearBicycleModel.nominalFeedback(x,u,lane,reference,cfg,nominal);end;feedback=toc(wall)/count;
    wall=tic;for i=1:count,nonlinearBicycleModel.sample(x,u,cfg);end;sample=toc(wall)/count;
    wall=tic;for i=1:count,[~,~,~,~,~,~]=nonlinearBicycleModel.hold(x,u,cfg);end;hold=toc(wall)/count;
    % Central differences of the guidance in the six states and the held
    % braking ratio: fourteen evaluations per node.
    wall=tic;
    for i=1:count/10
        step=1e-6;
        for j=1:6
            d=zeros(6,1);d(j)=step;
            nonlinearBicycleModel.nominalFeedback(x+d,u,lane,reference,cfg,nominal);
            nonlinearBicycleModel.nominalFeedback(x-d,u,lane,reference,cfg,nominal);
        end
        nonlinearBicycleModel.nominalFeedback(x,u+[0;step],lane,reference,cfg,nominal);
        nonlinearBicycleModel.nominalFeedback(x,u-[0;step],lane,reference,cfg,nominal);
    end
    jacobian=toc(wall)/(count/10);
    cost=struct('feedbackMicroseconds',1e6*feedback,'sampleMicroseconds',1e6*sample, ...
        'holdMicroseconds',1e6*hold,'guidanceJacobianMicroseconds',1e6*jacobian, ...
        'perTailNodeMicroseconds',1e6*(feedback+sample+hold+jacobian));
    fprintf('NODE feedback %.1f us, sample %.1f us, hold %.1f us, guidance Jacobian %.1f us, per tail node %.1f us\n', ...
        cost.feedbackMicroseconds,cost.sampleMicroseconds,cost.holdMicroseconds, ...
        cost.guidanceJacobianMicroseconds,cost.perTailNodeMicroseconds);
end

function scaling=localSolveScaling(lengths)
    % Present formulation, fixed horizon N: a receding target inside the
    % perception radius (collision rows on the early nodes) and road rows.
    scaling=struct('horizon',{},'startupSeconds',{},'startupConicSeconds',{}, ...
        'shiftedSeconds',{},'shiftedConicSeconds',{});
    road=struct('centerline',[-100,0;1000,0],'lateralClearance',[8.5344;12.192]);
    target=struct('targetPositionInertial',[-15;6],'targetVelocityInertial',[-8;0],'targetYawInertial',pi);
    for n=lengths
        cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
            'controller',struct('horizonSteps',n,'maximumHorizonSteps',n)));
        ego=struct('position',[0;0],'yaw',0,'speed',8,'lateralVelocity',0,'yawRate',0);
        collisionAvoidanceController("resetNominalTrajectory");
        wall=tic;[~,~,first,prior]=collisionAvoidanceController(ego,target,road,cfg,[]);startup=toc(wall);
        x=prior.stateTrajectory(:,2);ego.position=x(1:2);ego.yaw=x(3);ego.speed=x(4);
        ego.lateralVelocity=x(5);ego.yawRate=x(6);ego.heldActuatorInput=prior.appliedInput;
        wall=tic;[~,~,second]=collisionAvoidanceController(ego,target,road,cfg,prior);shifted=toc(wall);
        conic=@(p)sum([p.metadata.search.stages([p.metadata.search.stages.numericalSolve]).seconds]);
        scaling(end+1)=struct('horizon',n,'startupSeconds',startup,'startupConicSeconds',conic(first), ...
            'shiftedSeconds',shifted,'shiftedConicSeconds',conic(second)); %#ok<AGROW>
        fprintf('SOLVE N=%4d startup %.1f ms (conic %.1f ms), shifted %.1f ms (conic %.1f ms)\n', ...
            n,1e3*startup,1e3*conic(first),1e3*shifted,1e3*conic(second));
    end
end

function value=localQuantile(values,p)
    values=sort(values(isfinite(values)));
    if isempty(values),value=NaN;return;end
    value=values(max(1,ceil(p*numel(values))));
end
