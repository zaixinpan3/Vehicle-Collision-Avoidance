classdef terminalSafeSet
%terminalSafeSet Safe-exit terminal set of a family of lane-hold CLF backups.
% After the horizon a backup controller completes the encounter. A backup is
% the CLF of the given path's trim held at a lane centre d: the nominal path
% (d = 0) or another lane centre of model.road.laneOffsets. It is any
% controller whose CLF row holds without slack, V_d(t) <= exp(-2t/T) V_d(0),
% about the trim of the path offset by d. Such a controller keeps the
% transverse error in
%   |e_i(t)| <= a_i sqrt(V0) exp(-t/T),    a_i = sqrt((inv(P))_ii),
% about the offset path, and the path station within s0 + vPath_d t +/- G(V0,t),
% G the integral of a bound on the station-rate error. The ego rectangle
% therefore stays in a box in path coordinates (the CLF tube of the backup).
% For one backup the terminal set is
%   S_d = { x : V_d(x) <= levelMaximum_d, the tube of x stays on the road,
%           and the tube does not meet the target's forecast rectangle until
%           the encounter ends },
% and the terminal set is the union of the S_d. The encounter ends at the
% earlier of the target leaving encounterRangeMeters of every point of the
% tube and the end of the encounter window, which is fixed in time:
% terminal.horizonSeconds after the encounter's start (model.encounterWindow
% Seconds remain from the present sample). The end is an absorbing mode: no
% property of the target after it is used, and a target still present at the
% window end starts a new encounter. The tube shrinks along any
% CLF-satisfying trajectory (V1 <= rho V0 nests the tube of x1 in the tube of
% x0 shifted by one hold) and the window end does not move, so S_d is
% invariant under its backup until the encounter ends. See
% TERMINAL_SAFE_SET.md.
% The nominal backup is tried first; the others only matter where it fails.
% levelMaximum_d is the smaller of terminal.levelMaximum (the CLF's
% certified region) and the largest level whose ellipsoid lies inside the
% state rows of the problem (speed, lateral velocity and yaw-rate limits,
% sideslip cone, rear adhesion at the certification braking ratio).
% On a curve the backup of lane d uses the trim of the offset path
% (curvature kappa/(1-kappa d)) with the CLF matrix of the given path; that
% pairing is not certified offline, and every hold the terminal controller
% appends is checked for V(x+) <= rho V(x).
% "Does not meet" is checked on a time grid of step dt: at each grid point
% the two boxes must be farther apart than the distance their bodies can
% move in dt/2 (a continuous-time no-collision bound), and at least
% safetyMarginMeters, so that the continuation also satisfies the
% problem's own sampled collision rows at its nodes and hold midpoints,
% which lie on the grid. The ego estimation enclosure, when present,
% inflates the tube.
    methods (Static)
        function references = modeReferences(model)
            % The backups' references: the nominal path first, then the other
            % lane centres by distance from it.
            base=model.nominalReference;base.lateralOffset=0;
            offsets=0;
            if isfield(model,'road') && isfield(model.road,'laneOffsets') && ~isempty(model.road.laneOffsets)
                offsets=unique([0,reshape(model.road.laneOffsets,1,[])]);
            end
            [~,order]=sort(abs(offsets));offsets=offsets(order);
            references=repmat(base,1,numel(offsets));
            for index=2:numel(offsets)
                references(index).lateralOffset=offsets(index);
                if base.curvature~=0
                    point=terminalSafeSet.offsetTrim(model.cfg,base.curvature,offsets(index));
                    references(index).state=point.state;references(index).input=point.input;
                    references(index).continuousA=point.continuousA;references(index).continuousB=point.continuousB;
                end
            end
        end

        function point = offsetTrim(cfg,curvature,offset)
            % Trim of the path offset by 'offset' (curvature kappa/(1-kappa d)).
            persistent keys points
            if isempty(keys),keys={};points={};end
            key=[char(nonlinearBicycleModel.clfKey(cfg,curvature)),sprintf('|%.17g',offset)];
            index=find(strcmp(keys,key),1);
            if ~isempty(index),point=points{index};return;end
            point=nonlinearBicycleModel.operatingPoint(cfg,curvature/(1-curvature*offset));
            keys=[{key},keys(1:min(end,31))];points=[{point},points(1:min(end,31))];
        end

        function context = context(model)
            % Constants of one frame, one entry of 'modes' per backup, and a
            % lazily extended target table on the absolute grid of this frame
            % (step dt, index 0 = now) shared by all backups.
            cfg=model.cfg;terminal=cfg.terminal;
            references=terminalSafeSet.modeReferences(model);
            positionRadius=0;yawRadius=0;
            if isfield(model,'uncertainty') && isfield(model.uncertainty,'egoGenerator')
                generator=model.uncertainty.egoGenerator;
                positionRadius=sum(vecnorm(generator(1:2,:),2,1));yawRadius=sum(abs(generator(3,:)));
            end
            h=cfg.controller.sampleTime;dt=terminal.timeStepSeconds;
            ratio=round(h/(2*dt))*2;
            if abs(ratio*dt-h)>1e-9*h
                error('collisionAvoidanceController:invalidTerminalGrid', ...
                    'terminal.timeStepSeconds must divide half the sample time.');
            end
            here=laneGeometry.project(model.initialState(1:2),model.lane);
            clearance=[Inf;Inf];
            if ~isempty(model.road.lateralClearance),clearance=model.road.lateralClearance;end
            modes=struct('reference',cell(1,numel(references)),'scale',[],'yawOffset',[], ...
                'velocity',[],'yawRate',[],'tangentSpeed',[],'pathSpeed',[],'lateralOffset',[],'levelMaximum',[]);
            for index=1:numel(references)
                reference=references(index);offset=reference.state(3);velocity=reference.state(4:5);
                tangentSpeed=velocity(1)*cos(offset)-velocity(2)*sin(offset);
                modes(index).reference=reference;modes(index).scale=sqrt(diag(inv(reference.matrix)));
                modes(index).yawOffset=offset;modes(index).velocity=velocity;
                modes(index).yawRate=reference.state(6);modes(index).tangentSpeed=tangentSpeed;
                % Station rate of the offset path, on the given path's stations.
                modes(index).pathSpeed=tangentSpeed/(1-reference.curvature*reference.lateralOffset);
                modes(index).lateralOffset=reference.lateralOffset;
                modes(index).levelMaximum=min(terminal.levelMaximum,terminalSafeSet.stateLevel(reference,cfg));
            end
            context=struct('modes',modes,'timeConstant',cfg.clf.convergenceTimeConstantSeconds, ...
                'curvature',references(1).curvature,'pathSpeed',modes(1).pathSpeed, ...
                'halfLength',cfg.vehicle.length/2,'halfWidth',cfg.vehicle.width/2, ...
                'offset',norm(cfg.vehicle.rectangleOffset), ...
                'reach',norm([cfg.vehicle.length;cfg.vehicle.width]/2)+norm(cfg.vehicle.rectangleOffset), ...
                'buffer',cfg.collision.safetyMarginMeters,'range',cfg.collision.encounterRangeMeters, ...
                'clearance',clearance,'levelMaximum',modes(1).levelMaximum, ...
                'dt',dt,'ratio',ratio,'windowSteps',localWindowSteps(model,terminal,dt), ...
                'positionRadius',positionRadius,'yawRadius',yawRadius, ...
                'hasTarget',~isempty(model.target),'stationNow',here.station, ...
                'covered',-1,'targetStation',zeros(2,0),'targetLateral',zeros(2,0), ...
                'targetSpeed',zeros(1,0),'pointStation',zeros(1,0),'pointLateral',zeros(1,0));
            context.model=struct('targetEpoch',model.targetEpoch,'sampleIndex',model.sampleIndex, ...
                'sampleTime',h,'lane',model.lane,'reference',references(1));
        end

        function [member,context,info] = member(context,x,node,mode)
            % x at plan node 'node' (0 = now). Membership in the terminal set:
            % the first backup (nominal first) whose set contains x, or only
            % backup 'mode' when given. info.mode is its index (NaN if none);
            % the other fields are those of the nominal (or given) backup when
            % x is not a member.
            if nargin>3,candidates=mode;else,candidates=1:numel(context.modes);end
            member=false;info=[];
            for m=candidates
                [value,station]=terminalSafeSet.coordinates(context,x,m);
                % A backup whose region does not contain x is skipped cheaply.
                if m~=candidates(1) && ~(value<=context.modes(m).levelMaximum*(1+1e-12)),continue;end
                [ok,context,attempt]=terminalSafeSet.clear(context,value,station,node,m);
                attempt.value=value;attempt.mode=m;attempt.lateralOffset=context.modes(m).lateralOffset;
                if ok,member=true;info=attempt;return;end
                if isempty(info),info=attempt;end
            end
            info.mode=NaN;
        end

        function [mode,context] = select(context,x,node)
            % The backup of a plan endpoint: the one whose set contains it, or
            % else the one whose CLF region it is closest to.
            [member,context,info]=terminalSafeSet.member(context,x,node);
            if member,mode=info.mode;return;end
            ratios=inf(1,numel(context.modes));
            for m=1:numel(context.modes)
                ratios(m)=terminalSafeSet.coordinates(context,x,m)/context.modes(m).levelMaximum;
            end
            [~,mode]=min(ratios);
        end

        function [level,context,info] = level(context,x,node,mode)
            % Largest CLF level of backup 'mode' whose tube at x's station is
            % clear (-Inf if none). Clearance is monotone in the level: tubes
            % are nested. Without 'mode' the backup is selected (select).
            if nargin<4,[mode,context]=terminalSafeSet.select(context,x,node);end
            maximum=context.modes(mode).levelMaximum;
            [value,station]=terminalSafeSet.coordinates(context,x,mode);
            [top,context,info]=terminalSafeSet.clear(context,maximum,station,node,mode);
            info.value=value;info.mode=mode;
            if top,level=maximum;return;end
            [bottom,context]=terminalSafeSet.clear(context,0,station,node,mode);
            if ~bottom,level=-Inf;return;end
            low=0;high=maximum;
            for iteration=1:30
                middle=(low+high)/2;
                [ok,context]=terminalSafeSet.clear(context,middle,station,node,mode);
                if ok,low=middle;else,high=middle;end
                if high-low<=1e-4*maximum,break;end
            end
            level=low;[~,context,info]=terminalSafeSet.clear(context,level,station,node,mode);
            info.value=value;info.mode=mode;
        end

        function [value,station] = coordinates(context,x,mode)
            if nargin<3,mode=1;end
            lane=context.model.lane;reference=context.modes(mode).reference;
            projection=laneGeometry.project(x(1:2),lane,context.stationNow);
            value=nonlinearBicycleModel.nominalValue(x,lane,reference);station=projection.station;
        end

        function [ok,context,info] = clear(context,value,station,node,mode)
            % Tube of backup 'mode' (default nominal) of level 'value' starting
            % at 'station' at plan node 'node'.
            if nargin<5,mode=1;end
            info=struct('exitSeconds',NaN,'margin',Inf,'reason',"");ok=false;
            if ~(value<=context.modes(mode).levelMaximum*(1+1e-12))
                info.reason="levelAboveCertifiedRegion";return;
            end
            % The lateral extent is largest at the start and only shrinks.
            ego=terminalSafeSet.tube(context,value,station,0,0,mode);
            if ego.lateral(2)>context.clearance(2) || -ego.lateral(1)>context.clearance(1)
                info.reason="tubeLeavesRoad";return;
            end
            if ~context.hasTarget,ok=true;info.exitSeconds=0;return;end
            start=node*context.ratio;chunk=round(2/context.dt);
            if start>=context.windowSteps
                % The encounter window has ended by this node (absorbing).
                ok=true;info.exitSeconds=0;info.reason="encounterWindowEnded";return;
            end
            steps=0;shift=0;
            while true
                last=min(start+steps+chunk,context.windowSteps);
                context=terminalSafeSet.extend(context,last);
                tau=(steps:last-start)*context.dt;
                [ego,box,guard]=terminalSafeSet.tube(context,value,station,tau,shift,mode);
                index=start+(steps:last-start)+1;
                pointGap=terminalSafeSet.separation(context,box.station,box.lateral, ...
                    context.pointStation(:,index),context.pointLateral(:,index));
                exitAt=find(pointGap>context.range,1);
                bodyGap=terminalSafeSet.separation(context,ego.station,ego.lateral, ...
                    context.targetStation(:,index),context.targetLateral(:,index));
                required=max(context.buffer,guard+context.dt/2*context.targetSpeed(index));
                limit=numel(tau);if ~isempty(exitAt),limit=exitAt-1;end
                if limit>0
                    margin=bodyGap(1:limit)-required(1:limit);info.margin=min(info.margin,min(margin));
                    if any(margin<0),info.reason="tubeMeetsTarget";return;end
                end
                if ~isempty(exitAt),ok=true;info.exitSeconds=tau(exitAt);return;end
                % Clear until the encounter window ends: the encounter ends
                % there (absorbing); the window end is fixed in time.
                if last>=context.windowSteps
                    ok=true;info.exitSeconds=tau(end);info.reason="clearToWindowEnd";return;
                end
                shift=box.shift;steps=last-start+1;
            end
        end

        function [ego,box,guard] = tube(context,value,station,tau,shift,mode)
            % Path-coordinate boxes of the ego rectangle (ego) and of its
            % reference point (box) at times tau after the node under backup
            % 'mode' (default nominal), and the distance (guard) an ego body
            % point can move in dt/2 there.
            if nargin<6,mode=1;end
            p=context.modes(mode);
            radius=sqrt(max(value,0))*exp(-tau/context.timeConstant);
            a=p.scale;kappa=abs(context.curvature);d=p.lateralOffset;
            lateral=a(1)*radius+context.positionRadius;
            heading=min(pi/2,abs(p.yawOffset)+a(2)*radius+context.yawRadius);
            % Station rate about the offset path (TERMINAL_SAFE_SET.md, (3.2)):
            % |ds/dt - v_t*/(1-kappa d)| <= dv/q + |v_t*| |kappa| a1 r/(q (1-kappa d)),
            % q = 1 - kappa d - |kappa| a1 r, with the signed product kappa d.
            base=max(1-context.curvature*d,.5);
            q=max(base-kappa*a(1)*radius,.5);
            dv=(abs(p.velocity(1))+abs(p.velocity(2)))*a(2)*radius+(a(3)+a(4))*radius;
            speedError=dv./q+abs(p.tangentSpeed)*kappa*a(1)*radius./(q*base);
            % Left Riemann sum of a nonincreasing integrand bounds its integral.
            drift=shift+[0,cumsum(speedError(1:end-1))]*context.dt;
            centre=station+p.pathSpeed*tau;
            along=context.halfLength*cos(heading)+context.halfWidth*sin(heading)+context.offset;
            across=context.halfLength*sin(heading)+context.halfWidth*cos(heading)+context.offset;
            bulge=kappa*(2*context.halfLength)^2/8;
            outer=lateral+across+bulge;
            stretch=along./max(1-kappa*(abs(d)+outer),.5);
            ego.station=[centre-drift-stretch-context.positionRadius;centre+drift+stretch+context.positionRadius];
            ego.lateral=[d-outer;d+outer];
            box.station=[centre-drift;centre+drift];box.lateral=[d-lateral;d+lateral];
            box.shift=drift(end)+speedError(end)*context.dt;
            % Body-point speed over [tau-dt/2,tau+dt/2]: |v| + |r| reach.
            early=radius*exp(context.dt/(2*context.timeConstant));
            speed=hypot(abs(p.velocity(1))+a(3)*early,abs(p.velocity(2))+a(4)*early) ...
                +(abs(p.yawRate)+a(5)*early)*context.reach;
            guard=context.dt/2*speed;
        end

        function gap = separation(context,station,lateral,otherStation,otherLateral)
            % Lower bound of the Euclidean distance between two path boxes.
            alongGap=max(0,max(otherStation(1,:)-station(2,:),station(1,:)-otherStation(2,:)));
            acrossGap=max(0,max(otherLateral(1,:)-lateral(2,:),lateral(1,:)-otherLateral(2,:)));
            kappa=context.curvature;
            if kappa==0
                gap=hypot(alongGap,acrossGap);return;
            end
            radius=1/abs(kappa);
            if kappa>0,inner=radius-max(lateral(2,:),otherLateral(2,:));
            else,inner=radius+min(lateral(1,:),otherLateral(1,:));
            end
            chord=2*max(inner,0).*sin(min(pi/2,alongGap/(2*radius)));
            gap=max(acrossGap,chord);
        end

        function context = extend(context,last)
            % Target rectangle and reference point in path coordinates on the
            % absolute grid up to index 'last' (inclusive), and the largest
            % speed of a target body point within dt/2 of each grid time.
            if last<=context.covered,return;end
            index=context.covered+1:last;time=index*context.dt;
            m=context.model;
            q=predictiveSafetyGeometry.predictTarget(m.targetEpoch,m.sampleIndex*m.sampleTime+time);
            hint=context.stationNow+context.pathSpeed*time;
            c=cos(q(3,:));s=sin(q(3,:));signs=[-1,1,1,-1;-1,-1,1,1];
            station=zeros(4,numel(index));lateral=zeros(4,numel(index));
            for corner=1:4
                lx=q(10,:)+q(8,:)*signs(1,corner);ly=q(11,:)+q(9,:)*signs(2,corner);
                points=[q(1,:)+c.*lx-s.*ly;q(2,:)+s.*lx+c.*ly];
                projection=laneGeometry.project(points,m.lane,hint);
                station(corner,:)=projection.station;lateral(corner,:)=projection.lateralPosition;
            end
            bulge=abs(context.curvature)*(2*max(q(8:9,:),[],1)).^2/8;
            reference=laneGeometry.project(q(1:2,:),m.lane,hint);
            speed=abs(q(4,:))+abs(q(5,:))*context.dt/2;
            reach=vecnorm(q(8:9,:),2,1)+vecnorm(q(10:11,:),2,1);
            speed=speed.*(1+abs(sin(q(6,:))./q(7,:)).*reach);
            context.targetStation=[context.targetStation,[min(station,[],1);max(station,[],1)]];
            context.targetLateral=[context.targetLateral,[min(lateral,[],1)-bulge;max(lateral,[],1)+bulge]];
            context.targetSpeed=[context.targetSpeed,speed];
            context.pointStation=[context.pointStation,[reference.station;reference.station]];
            context.pointLateral=[context.pointLateral,[reference.lateralPosition;reference.lateralPosition]];
            context.covered=last;
        end

        function level = stateLevel(reference,cfg)
            % Largest c with {e : e'Pe <= c} inside the linear state rows g'e <= b:
            % c = min b^2/(g' inv(P) g).
            x=reference.state;tire=modifiedFialaTire.parameters(cfg);
            k=3*tire.longitudinalForceScale(2)/tire.corneringStiffness(2);
            t=tan(cfg.model.sideslipMaximum);lr=cfg.vehicle.lr;
            low=max(cfg.model.speedMinimum,cfg.model.scheduleSpeedFloor+1e-4);
            braking=min(1,abs(reference.input(2))+cfg.clf.certificationBrakingRatio);
            adhesion=k*sqrt(1-braking^2);
            % e = [lateral; heading; vx-vx*; vy-vy*; r-r*]
            g=[0,0,1,0,0;0,0,-1,0,0;0,0,0,1,0;0,0,0,-1,0;0,0,0,0,1;0,0,0,0,-1; ...
                0,0,-t,1,0;0,0,-t,-1,0;0,0,-adhesion,1,-lr;0,0,-adhesion,-1,lr];
            w=x(5)-lr*x(6);
            b=[cfg.model.speedMaximum-x(4);x(4)-low;cfg.model.lateralVelocityMaximum-x(5); ...
                cfg.model.lateralVelocityMaximum+x(5);cfg.model.yawRateMaximum-x(6);cfg.model.yawRateMaximum+x(6); ...
                t*x(4)-x(5);t*x(4)+x(5);adhesion*x(4)-w;adhesion*x(4)+w];
            spread=sum((g/reference.matrix).*g,2);
            level=min(max(b,0).^2./spread);
        end

        function [middle,next] = holdStates(x,u,cfg)
            % The hold's midpoint and endpoint exactly as nonlinearBicycleModel.hold
            % forms them, without tangents.
            half=cfg.controller.sampleTime/2;
            if nonlinearBicycleModel.meshCount(cfg)==1
                next=nonlinearBicycleModel.sample(x,u,cfg);middle=(x+next)/2;
            else
                middle=nonlinearBicycleModel.sample(x,u,cfg,[],half);
                next=nonlinearBicycleModel.sample(middle,u,cfg,[],half);
            end
        end

        function [input,next,ok,value] = terminalInput(x,previous,model,reference)
            % One hold of a backup (default nominal): an admissible input whose
            % successor meets its CLF without slack, V(next) <= rho V(x), the
            % road, the stable-handling envelope and the state limits.
            if nargin<4,reference=model.nominalReference;end
            cfg=model.cfg;lane=model.lane;
            current=nonlinearBicycleModel.nominalValue(x,lane,reference);
            bound=reference.contraction*current*(1+1e-9)+1e-12;
            candidates=terminalSafeSet.candidateInputs(x,previous,model,reference);
            input=[];next=[];ok=false;value=Inf;
            for k=1:size(candidates,2)
                u=candidates(:,k);
                try
                    [middle,successor]=terminalSafeSet.holdStates(x,u,cfg);
                catch exception
                    if startsWith(string(exception.identifier),"collisionAvoidanceController:"),continue;end
                    rethrow(exception);
                end
                v=nonlinearBicycleModel.nominalValue(successor,lane,reference);
                if v<value && terminalSafeSet.admissible(x,middle,successor,u,model)
                    input=u;next=successor;value=v;
                end
                if k<=2 && value<=bound,break;end
            end
            ok=value<=bound;
        end

        function candidates = candidateInputs(x,previous,model,reference)
            % The path guidance, the input minimizing the linear sampled CLF
            % successor, and a grid about the latter, for a backup's reference.
            if nargin<4,reference=model.nominalReference;end
            cfg=model.cfg;lane=model.lane;
            guidance=nonlinearBicycleModel.nominalGuidanceParameters(cfg,reference.curvature);
            nominal=nonlinearBicycleModel.nominalFeedback(x,previous,lane,reference,cfg,guidance);
            h=cfg.controller.sampleTime;
            transition=expm([reference.continuousA,reference.continuousB;zeros(2,7)]*h);
            a=transition(1:5,1:5);b=transition(1:5,6:7);
            e=nonlinearBicycleModel.error(x,lane,reference);
            least=reference.input-(reference.factor*b)\(reference.factor*a*e);
            [steer,brake]=ndgrid(-.15:.03:.15,-.3:.075:.3);
            grid=least+[steer(:).';brake(:).'];
            candidates=[nominal,least,grid];
            low=max(-1+1e-8,cfg.actuation.brakingRatioMinimum);high=min(1-1e-8,cfg.actuation.brakingRatioMaximum);
            candidates(2,:)=min(high,max(low,candidates(2,:)));
        end

        function ok = admissible(x,middle,next,u,model)
            % Hard rows of one hold on the nonlinear model.
            cfg=model.cfg;
            ok=terminalSafeSet.stateRows(next,cfg) && terminalSafeSet.stateRows(middle,cfg) ...
                && terminalSafeSet.handlingRows(x,u,cfg) && terminalSafeSet.handlingRows(next,u,cfg) ...
                && terminalSafeSet.roadMargin(next,model)>=-1e-6 && terminalSafeSet.roadMargin(middle,model)>=-1e-6;
        end

        function ok = stateRows(x,cfg)
            low=[max(cfg.model.speedMinimum,cfg.model.scheduleSpeedFloor+1e-4);-cfg.model.lateralVelocityMaximum;-cfg.model.yawRateMaximum];
            high=[cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum];
            ok=all(x(4:6)>=low-1e-9 & x(4:6)<=high+1e-9);
        end

        function ok = handlingRows(x,u,cfg)
            % Rear adhesion ||[w/k; v_x b]|| <= v_x and the sideslip cone.
            tire=modifiedFialaTire.parameters(cfg);
            k=3*tire.longitudinalForceScale(2)/tire.corneringStiffness(2);
            adhesion=hypot((x(5)-cfg.vehicle.lr*x(6))/k,x(4)*u(2));
            ok=adhesion<=x(4)*(1+1e-9) && abs(x(5))<=tan(cfg.model.sideslipMaximum)*x(4)*(1+1e-9);
        end

        function margin = roadMargin(x,model)
            % Smallest distance of an ego corner inside the road (negative outside).
            margin=Inf;
            if isempty(model.road.lateralClearance),return;end
            cfg=model.cfg;clearance=model.road.lateralClearance;
            corners=cfg.vehicle.rectangleOffset+[cfg.vehicle.length;cfg.vehicle.width]/2.*[1,1,-1,-1;1,-1,1,-1];
            rotation=[cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
            projection=laneGeometry.project(x(1:2)+rotation*corners,model.lane);
            lateral=projection.lateralPosition;
            margin=min([clearance(2)-lateral(:);clearance(1)+lateral(:)]);
        end
    end
end

function steps = localWindowSteps(model,terminal,dt)
    % Grid index (from the present sample) at which the encounter window ends.
    seconds=terminal.horizonSeconds;
    if isfield(model,'encounterWindowSeconds') && isfinite(model.encounterWindowSeconds)
        seconds=max(0,model.encounterWindowSeconds);
    end
    steps=floor(seconds/dt+1e-9);
end
