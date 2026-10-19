function baseline = bypassFeasibilityBaseline(initialState,target,road,cfg,options)
%bypassFeasibilityBaseline Find an independent, physically feasible bypass.
% Offline scenario qualification only: truth is used to try prescribed offset
% maneuvers. No avoidance field, RTI solution, or terminal-set test is called.
% A retained held-input sequence is independently replayed with ode45. Its
% complete ego and target rectangles must stay inside the scenario road and
% separated at sample nodes until the first perception-range exit. The ego
% subsequently returns to nominal cruise. This nominal-model witness makes
% no claim about robustness to every observer/model uncertainty realization.
% Failure of this finite maneuver family means unqualified, not impossible.
    arguments
        initialState (6,1) double {mustBeReal,mustBeFinite}
        target (11,1) double {mustBeReal,mustBeFinite}
        road (1,1) struct
        cfg (1,1) struct
        options.HalfWidthMeters (1,1) double {mustBePositive,mustBeFinite} = 9
    end
    curve=struct('origin',[0;0],'heading',0,'curvature',0,'length',200);
    if isfield(road,'referenceCurve'),curve=road.referenceCurve;
    else
        direction=road.centerline(end,:)-road.centerline(1,:);
        curve.origin=road.centerline(1,:).';curve.heading=atan2(direction(2),direction(1));
    end
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    baseline=struct('qualified',false,'reason',"noWitnessFound", ...
        'scope',"sampled nominal nonlinear vehicle; truth-based offline scenario qualification", ...
        'robustFeasibilityClaimed',false,'halfWidthMeters',options.HalfWidthMeters, ...
        'minimumClearanceMeters',NaN,'minimumEgoRoadMarginMeters',NaN, ...
        'minimumTargetRoadMarginMeters',NaN,'maximumLateralDeviationMeters',NaN, ...
        'encounterExitSeconds',NaN,'recoverySeconds',NaN,'attempts',0, ...
        'initialState',initialState,'targetInitialState',target,'inputs',zeros(2,0), ...
        'states',zeros(6,0),'times',zeros(1,0),'offsetMeters',NaN,'riseSeconds',NaN);
    if scenarioRoadFootprintMargin(target(1:3),target(8:11),curve,options.HalfWidthMeters)<0
        baseline.reason="targetOutsideRoad";return;
    end
    if scenarioRoadFootprintMargin(initialState(1:3),shape,curve,options.HalfWidthMeters)<0
        baseline.reason="egoOutsideRoad";return;
    end
    if predictiveSafetyGeometry.rectangle(initialState(1:3),shape,target(1:3),target(8:11))<=0
        baseline.reason="initialCollision";return;
    end
    if norm(initialState(1:2)-target(1:2))>cfg.collision.encounterRangeMeters
        baseline.reason="targetInitiallyOutsidePerception";return;
    end
    reference=nonlinearBicycleModel.cruise(cfg,curve.curvature);
    % A fixed offline family supplies existence evidence, not an online
    % candidate selector. Retain the first successful independent replay.
    for offset=[3,4.5,6]
        for rise=[1.2,1.6,2,2.5,3]
            for side=[1,-1]
                baseline.attempts=baseline.attempts+1;
                candidate=localRollout(initialState,target,curve,shape,cfg,reference, ...
                    side*offset,rise,options.HalfWidthMeters,[]);
                if ~candidate.qualified,continue;end
                verified=localRollout(initialState,target,curve,shape,cfg,reference, ...
                    side*offset,rise,options.HalfWidthMeters,candidate.inputs);
                if ~verified.qualified,continue;end
                fields=fieldnames(verified);
                for k=1:numel(fields),baseline.(fields{k})=verified.(fields{k});end
                baseline.offsetMeters=side*offset;baseline.riseSeconds=rise;return;
            end
        end
    end
end

function result=localRollout(x,target,curve,shape,cfg,reference,offset,rise,width,replayInputs)
    h=cfg.controller.sampleTime;
    % Explicit finite witness-search coverage, not a controller horizon or
    % an assumption that an unresolved encounter becomes safe at this time.
    duration=2*cfg.collision.encounterRangeMeters/max(cfg.referenceSpeed,1)+4*rise+8;
    count=ceil(duration/h);replay=~isempty(replayInputs);
    if replay,count=size(replayInputs,2);end
    inputs=zeros(2,count);states=zeros(6,count+1);states(:,1)=x;
    q=predictiveSafetyGeometry.predictTarget(target,(0:count)*h);
    previous=zeros(2,1);exitTime=NaN;minimum=Inf;egoMargin=Inf;targetMargin=Inf;maximum=0;
    qualified=false;reason="searchCoverageExhausted";
    [lower,upper]=nonlinearBicycleModel.inputBounds(cfg,false);
    for node=1:count+1
        time=(node-1)*h;x=states(:,node);
        egoMargin=min(egoMargin,scenarioRoadFootprintMargin(x(1:3),shape,curve,width));
        targetMargin=min(targetMargin,scenarioRoadFootprintMargin(q(1:3,node),q(8:11,node),curve,width));
        projection=laneGeometry.projectReferenceCurve(x(1:2),curve);
        maximum=max(maximum,abs(projection.lateralPosition));
        if min(egoMargin,targetMargin)<0,reason="footprintOutsideRoad";break;end
        if localStateViolation(x,cfg)>1e-9,reason="stateLimit";break;end
        if isnan(exitTime)
            gap=predictiveSafetyGeometry.rectangle(x(1:3),shape,q(1:3,node),q(8:11,node));
            minimum=min(minimum,gap);
            if gap<=0,reason="collision";break;end
            if norm(x(1:2)-q(1:2,node))>cfg.collision.encounterRangeMeters,exitTime=time;end
        end
        heading=atan2(sin(x(3)-projection.heading-reference.state(3)),cos(x(3)-projection.heading-reference.state(3)));
        recovered=~isnan(exitTime) && time>exitTime+rise && abs(projection.lateralPosition)<.1 ...
            && abs(heading)<pi/180 && abs(hypot(x(4),x(5))-cfg.referenceSpeed)<.1 ...
            && abs(x(5)-reference.state(5))<.05 && abs(x(6)-reference.state(6))<.01;
        if recovered && (~replay || node==count+1),qualified=true;reason="verifiedBypass";break;end
        if node>count,break;end
        if replay
            u=replayInputs(:,node);
        else
            [b,db,ddb]=localShape(time/rise);
            if ~isnan(exitTime)
                [s,ds,dds]=localShape((time-exitTime)/rise);b=1-s;db=-ds;ddb=-dds;
            end
            tangent=[cos(projection.heading);sin(projection.heading)];normal=[-tangent(2);tangent(1)];
            rotation=[cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];velocity=rotation*x(4:5);
            along=tangent.'*velocity;across=normal.'*velocity;kappa=curve.curvature;
            stationRate=along/max(1-kappa*projection.lateralPosition,.1);
            transverse=offset*ddb/rise^2+2*(offset*db/rise-across)+offset*b-projection.lateralPosition;
            acceleration=(.8*(cfg.referenceSpeed-along)-kappa*stationRate*across)*tangent ...
                +(kappa*stationRate*along+transverse)*normal;
            u=nonlinearBicycleModel.accelerationGuidance(x,acceleration,previous,cfg);
        end
        if any(u<lower-1e-10 | u>upper+1e-10) ...
                || abs(u(2)-previous(2))>cfg.model.brakingRatioRateMaximum*h+1e-10
            reason="inputLimit";break;
        end
        if replay
            [~,z]=ode45(@(~,z)nonlinearBicycleModel.derivative(z,u,cfg),[0,h],x, ...
                odeset('RelTol',1e-10,'AbsTol',1e-12));
            next=nonlinearBicycleModel.sampleEndState(z(end,:).',u,cfg);
        else
            next=nonlinearBicycleModel.sample(x,u,cfg);
        end
        inputs(:,node)=u;states(:,node+1)=next;previous=u;
    end
    result=struct('qualified',qualified,'reason',reason,'minimumClearanceMeters',minimum, ...
        'minimumEgoRoadMarginMeters',egoMargin,'minimumTargetRoadMarginMeters',targetMargin, ...
        'maximumLateralDeviationMeters',maximum,'encounterExitSeconds',exitTime, ...
        'recoverySeconds',(node-1)*h,'inputs',inputs(:,1:node-1), ...
        'states',states(:,1:node),'times',(0:node-1)*h);
end

function violation=localStateViolation(x,cfg)
    violation=max([0;cfg.model.speedMinimum-x(4);x(4)-cfg.model.speedMaximum; ...
        abs(x(5))-cfg.model.lateralVelocityMaximum;abs(x(6))-cfg.model.yawRateMaximum]);
    if x(4)>cfg.model.brakeHoldSpeed
        violation=max(violation,abs(x(5))-tan(cfg.model.sideslipMaximum)*x(4));
    end
    if any(~isfinite(x)),violation=Inf;end
end

function [s,ds,dds]=localShape(t)
    t=min(1,max(0,t));s=t^3*(10+t*(-15+6*t));
    ds=30*t^2*(1-t)^2;dds=60*t*(1-t)*(1-2*t);
end
