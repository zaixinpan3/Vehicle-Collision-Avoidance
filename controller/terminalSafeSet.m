classdef terminalSafeSet
%terminalSafeSet Encounter-safe terminal set of the CLF terminal controller.
% After the horizon the controller is the same problem without PCBF rows: a
% controller whose CLF row holds without slack, V(t) <= exp(-2t/T) V(0), on
% the given path's trim. Any such controller keeps the transverse error in
%   |e_i(t)| <= a_i sqrt(V0) exp(-t/T),    a_i = sqrt((inv(P))_ii),
% and the path station within s0 + vPath t +/- G(V0,t), G the integral of a
% bound on the path-speed error. The ego rectangle therefore stays in a box
% in path coordinates (the CLF tube). The terminal set is
%   S = { x : V(x) <= levelMaximum, the tube of x stays on the road, and it
%         does not meet the target's forecast rectangle until the target
%         leaves encounterRangeMeters of every point of the tube }.
% A target outside the perception range no longer exists for the
% controller, so the encounter ends there; a target that stays inside it is
% checked for terminal.horizonSeconds, within which the encounter is taken
% to end. levelMaximum is the smaller of
% terminal.levelMaximum (the CLF's certified region) and the largest level
% whose ellipsoid lies inside the state rows of the problem (speed, lateral
% velocity and yaw-rate limits, sideslip cone, rear adhesion at the
% certification braking ratio).
% The tube shrinks along any CLF-satisfying trajectory (V1 <= rho V0 nests
% the tube of x1 in the tube of x0 shifted by one hold), so S is forward
% invariant under the terminal controller; see TERMINAL_SAFE_SET.md.
% "Does not meet" is checked on a time grid of step dt: at each grid point
% the two boxes must be farther apart than the distance their bodies can
% move in dt/2 (a continuous-time no-collision bound), and at least
% safetyMarginMeters, so that the continuation also satisfies the
% problem's own sampled collision rows at its nodes and hold midpoints,
% which lie on the grid. The ego estimation enclosure, when present,
% inflates the tube.
    methods (Static)
        function context = context(model)
            % Constants of one frame and a lazily extended target table on
            % the absolute grid of this frame (step dt, index 0 = now).
            cfg=model.cfg;reference=model.nominalReference;terminal=cfg.terminal;
            scale=sqrt(diag(inv(reference.matrix)));
            offset=reference.state(3);velocity=reference.state(4:5);
            pathSpeed=velocity(1)*cos(offset)-velocity(2)*sin(offset);
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
            context=struct('scale',scale,'timeConstant',cfg.clf.convergenceTimeConstantSeconds, ...
                'contraction',reference.contraction,'yawOffset',offset,'velocity',velocity, ...
                'yawRate',reference.state(6),'pathSpeed',pathSpeed,'curvature',reference.curvature, ...
                'halfLength',cfg.vehicle.length/2,'halfWidth',cfg.vehicle.width/2, ...
                'offset',norm(cfg.vehicle.rectangleOffset), ...
                'reach',norm([cfg.vehicle.length;cfg.vehicle.width]/2)+norm(cfg.vehicle.rectangleOffset), ...
                'buffer',cfg.collision.safetyMarginMeters,'range',cfg.collision.encounterRangeMeters, ...
                'clearance',clearance, ...
                'levelMaximum',min(terminal.levelMaximum,terminalSafeSet.stateLevel(reference,cfg)), ...
                'dt',dt,'ratio',ratio,'horizonSteps',ceil(terminal.horizonSeconds/dt), ...
                'positionRadius',positionRadius,'yawRadius',yawRadius, ...
                'hasTarget',~isempty(model.target),'stationNow',here.station, ...
                'covered',-1,'targetStation',zeros(2,0),'targetLateral',zeros(2,0), ...
                'targetSpeed',zeros(1,0),'pointStation',zeros(1,0),'pointLateral',zeros(1,0));
            context.model=struct('targetEpoch',model.targetEpoch,'sampleIndex',model.sampleIndex, ...
                'sampleTime',h,'lane',model.lane,'reference',reference);
        end

        function [member,context,info] = member(context,x,node)
            % x at plan node 'node' (0 = now). V(x) is the tube level.
            [value,station]=terminalSafeSet.coordinates(context,x);
            [member,context,info]=terminalSafeSet.clear(context,value,station,node);
            info.value=value;
        end

        function [level,context,info] = level(context,x,node)
            % Largest CLF level whose tube at x's station is clear (-Inf if
            % none). Clearance is monotone in the level: tubes are nested.
            [value,station]=terminalSafeSet.coordinates(context,x);
            [top,context,info]=terminalSafeSet.clear(context,context.levelMaximum,station,node);
            if top,level=context.levelMaximum;info.value=value;return;end
            [bottom,context]=terminalSafeSet.clear(context,0,station,node);
            if ~bottom,level=-Inf;info.value=value;return;end
            low=0;high=context.levelMaximum;
            for iteration=1:30
                middle=(low+high)/2;
                [ok,context]=terminalSafeSet.clear(context,middle,station,node);
                if ok,low=middle;else,high=middle;end
                if high-low<=1e-4*context.levelMaximum,break;end
            end
            level=low;[~,context,info]=terminalSafeSet.clear(context,level,station,node);info.value=value;
        end

        function [value,station] = coordinates(context,x)
            lane=context.model.lane;reference=context.model.reference;
            projection=laneGeometry.project(x(1:2),lane,context.stationNow);
            value=nonlinearBicycleModel.nominalValue(x,lane,reference);station=projection.station;
        end

        function [ok,context,info] = clear(context,value,station,node)
            % Tube of level 'value' starting at 'station' at plan node 'node'.
            info=struct('exitSeconds',NaN,'margin',Inf,'reason',"");ok=false;
            if ~(value<=context.levelMaximum*(1+1e-12))
                info.reason="levelAboveCertifiedRegion";return;
            end
            % The lateral extent is largest at the start and only shrinks.
            ego=terminalSafeSet.tube(context,value,station,0,0);
            if ego.lateral(2)>context.clearance(2) || -ego.lateral(1)>context.clearance(1)
                info.reason="tubeLeavesRoad";return;
            end
            if ~context.hasTarget,ok=true;info.exitSeconds=0;return;end
            start=node*context.ratio;chunk=round(2/context.dt);
            steps=0;shift=0;
            while true
                last=min(start+steps+chunk,start+context.horizonSteps);
                context=terminalSafeSet.extend(context,last);
                tau=(steps:last-start)*context.dt;
                [ego,box,guard]=terminalSafeSet.tube(context,value,station,tau,shift);
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
                % Clear for the whole window: the encounter is taken to end
                % within terminal.horizonSeconds (TERMINAL_SAFE_SET.md, H4).
                if last>=start+context.horizonSteps
                    ok=true;info.exitSeconds=Inf;info.reason="clearThroughHorizon";return;
                end
                shift=box.shift;steps=last-start+1;
            end
        end

        function [ego,box,guard] = tube(context,value,station,tau,shift)
            % Path-coordinate boxes of the ego rectangle (ego) and of its
            % reference point (box) at times tau after the node, and the
            % distance (guard) an ego body point can move in dt/2 there.
            radius=sqrt(max(value,0))*exp(-tau/context.timeConstant);
            a=context.scale;kappa=abs(context.curvature);
            lateral=a(1)*radius+context.positionRadius;
            heading=min(pi/2,abs(context.yawOffset)+a(2)*radius+context.yawRadius);
            speedError=((abs(context.velocity(1))+abs(context.velocity(2)))*a(2)*radius ...
                +(a(3)+a(4))*radius+context.pathSpeed*kappa*a(1)*radius)./max(1-kappa*a(1)*radius,.5);
            % Left Riemann sum of a nonincreasing integrand bounds its integral.
            drift=shift+[0,cumsum(speedError(1:end-1))]*context.dt;
            centre=station+context.pathSpeed*tau;
            along=context.halfLength*cos(heading)+context.halfWidth*sin(heading)+context.offset;
            across=context.halfLength*sin(heading)+context.halfWidth*cos(heading)+context.offset;
            bulge=kappa*(2*context.halfLength)^2/8;
            outer=lateral+across+bulge;
            stretch=along./max(1-kappa*outer,.5);
            ego.station=[centre-drift-stretch-context.positionRadius;centre+drift+stretch+context.positionRadius];
            ego.lateral=[-outer;outer];
            box.station=[centre-drift;centre+drift];box.lateral=[-lateral;lateral];
            box.shift=drift(end)+speedError(end)*context.dt;
            % Body-point speed over [tau-dt/2,tau+dt/2]: |v| + |r| reach.
            early=radius*exp(context.dt/(2*context.timeConstant));
            speed=hypot(abs(context.velocity(1))+a(3)*early,abs(context.velocity(2))+a(4)*early) ...
                +(abs(context.yawRate)+a(5)*early)*context.reach;
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

        function [input,next,ok,value] = terminalInput(x,previous,model)
            % One hold of the terminal controller: an admissible input whose
            % successor meets the CLF without slack, V(next) <= rho V(x), the
            % road, the stable-handling envelope and the state limits.
            cfg=model.cfg;reference=model.nominalReference;lane=model.lane;
            current=nonlinearBicycleModel.nominalValue(x,lane,reference);
            bound=reference.contraction*current*(1+1e-9)+1e-12;
            candidates=terminalSafeSet.candidateInputs(x,previous,model);
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

        function candidates = candidateInputs(x,previous,model)
            % The path guidance, the input minimizing the linear sampled CLF
            % successor, and a grid about the latter.
            cfg=model.cfg;reference=model.nominalReference;lane=model.lane;
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
