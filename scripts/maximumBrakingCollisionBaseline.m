function baseline = maximumBrakingCollisionBaseline(initialState,target,cfg)
%maximumBrakingCollisionBaseline Audit immediate full braking without steering.
% The independent ode45 counterfactual continues to rest or the first target
% exit. After rest, analytic rectangle-track geometry checks the remainder of
% this encounter. There is no preset simulation-duration cutoff.
% For a collinear straight target ahead, longitudinal order also rules out
% every zero-steering input history: no weaker brake can stay behind this
% minimum-progress trajectory. This is a nominal-model comparison argument.
% For crossing traffic, failure of this one policy is not such a certificate.
    arguments
        initialState (6,1) double {mustBeReal,mustBeFinite}
        target double
        cfg (1,1) struct
    end
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    input=[0;cfg.actuation.brakingRatioMinimum];
    assert(input(2)<0,'maximumBrakingCollisionBaseline:noBrakingAuthority', ...
        'The maximum-braking counterfactual requires a negative admissible ratio.');
    x=initialState;time=0;distance=0;firstCollision=NaN;minimum=Inf;initialGap=Inf;
    phase="";ending="noTarget";
    if ~isempty(target)
        entered=norm(x(1:2)-target(1:2))<=cfg.collision.encounterRangeMeters;
        initialGap=predictiveSafetyGeometry.rectangle(x(1:3),shape,target(1:3),target(8:11));
        minimum=initialGap;
        if initialGap<=0,firstCollision=0;phase="initial";end
        h=cfg.controller.sampleTime;ending="stopped";
        while any(x(4:6)~=0)
            [t,z]=ode45(@(~,state)nonlinearBicycleModel.derivative(state,input,cfg), ...
                linspace(0,h,31),x,odeset('RelTol',1e-9,'AbsTol',1e-11));
            q=predictiveSafetyGeometry.predictTarget(target,time+t.');
            for node=1:numel(t)
                inside=norm(z(node,1:2).'-q(1:2,node))<=cfg.collision.encounterRangeMeters;
                if inside,entered=true;end
                if ~inside && entered
                    ending="exit";break;
                end
                gap=predictiveSafetyGeometry.rectangle(z(node,1:3).',shape,q(1:3,node),q(8:11,node));
                minimum=min(minimum,gap);
                if gap<=0 && strlength(phase)==0
                    firstCollision=time+t(node);phase="braking";
                end
            end
            if ending=="exit",time=time+t(node);x=z(node,:).';break;end
            distance=distance+sum(vecnorm(diff(z(:,1:2),1,1),2,2));
            x=nonlinearBicycleModel.sampleEndState(z(end,:).',input,cfg);time=time+h;
        end
        if ending=="stopped"
            q=predictiveSafetyGeometry.predictTarget(target,time);
            context=struct('shape',shape,'range',cfg.collision.encounterRangeMeters);
            remaining=terminalSafeSet.parkedClearance(context,x(1:3),q);
            minimum=min(minimum,remaining);
            if remaining<=0 && strlength(phase)==0,phase="afterStop";end
        end
    end
    longitudinalOrder=false;
    if ~isempty(target) && all(initialState(5:6)==0) && target(6)==0
        forward=predictiveSafetyGeometry.direction(initialState(3));normal=[-forward(2);forward(1)];
        targetForward=predictiveSafetyGeometry.direction(target(3));
        egoCenter=initialState(1:2)+[forward,normal]*shape(3:4);
        targetCenter=target(1:2)+[targetForward,[-targetForward(2);targetForward(1)]]*target(10:11);
        longitudinalOrder=abs(normal.'*targetForward)<1e-12 ...
            && abs(normal.'*(targetCenter-egoCenter))<shape(2)+target(9) ...
            && forward.'*(targetCenter-egoCenter)>shape(1)+target(8);
    end
    stopSeconds=NaN;stopDistance=NaN;
    if ending=="stopped",stopSeconds=time;stopDistance=distance;end
    baseline=struct('policy',"immediate maximum admissible braking with zero steering", ...
        'actuatorInput',input,'initialClearanceMeters',initialGap, ...
        'collisionDetected',strlength(phase)>0,'collisionPhase',phase, ...
        'firstCollisionSeconds',firstCollision,'minimumClearanceMeters',minimum, ...
        'stopSeconds',stopSeconds,'stoppingDistanceMeters',stopDistance, ...
        'finalState',x,'ending',ending,'longitudinalOrderApplicable',longitudinalOrder, ...
        'allLongitudinalPoliciesCertified',longitudinalOrder && strlength(phase)>0, ...
        'certificateScope',"nominal forward-only collinear model; numerical trajectory audit");
end
