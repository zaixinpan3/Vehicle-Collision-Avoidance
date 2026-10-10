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
%         does not meet the target's forecast rectangle until the encounter
%         ends }.
% The encounter ends at the first of two events, both of which can be
% computed, so no encounter duration is preset:
%   exit        the target leaves encounterRangeMeters of every point of the
%               tube;
%   separation  from that time on the target's forecast motion relative to
%               the tube's box lies outside their collision cone, so the two
%               provably never come within the collision buffer again
%               (straight road; see collisionCone).
% The end is an absorbing state: no property of the target after it is used.
% The check reads the forecast at the node's own time first (closed-form
% tests) and follows the grid only when needed, for a span the forecast's own
% geometry bounds (clear, scanBound); a tube that reaches the end of that span
% with neither event is not terminal. levelMaximum is the smaller
% of terminal.levelMaximum (the CLF's certified region) and the largest level
% whose ellipsoid lies inside the state rows of the problem (speed, lateral
% velocity and yaw-rate limits, sideslip cone, rear adhesion at the
% certification braking ratio).
% The tube shrinks along any CLF-satisfying trajectory (V1 <= rho V0 nests
% the tube of x1 in the tube of x0 shifted by one hold), so both events only
% come earlier and S is forward invariant under the terminal controller
% until the encounter ends; see TERMINAL_SAFE_SET.md.
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
                'dt',dt,'ratio',ratio, ...
                'pathHeading',here.heading,'targetMotion',localTargetMotion(model), ...
                'positionRadius',positionRadius,'yawRadius',yawRadius, ...
                'hasTarget',~isempty(model.target),'stationNow',here.station, ...
                'covered',-1,'targetStation',zeros(2,0),'targetLateral',zeros(2,0), ...
                'targetSpeed',zeros(1,0),'pointStation',zeros(1,0),'pointLateral',zeros(1,0), ...
                'targetCourse',zeros(1,0),'targetVelocity',zeros(1,0));
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
            % The node's own grid point is checked first: the tube and the
            % forecast at one time, so a relative motion outside the cone (or
            % a target already out of range) is certified in closed form. Only
            % otherwise does the check follow the grid, in 2-s chunks, until
            % the first exit or cone certificate, over a span that scanBound
            % derives from the forecast itself (on a straight road in closed
            % form: a forecast that meets the settled tube before the target
            % can leave the range is rejected at once). A tube that reaches
            % the end of the span with neither event is not terminal. Nothing
            % about the encounter is preset.
            info=struct('exitSeconds',NaN,'margin',Inf,'reason',"");ok=false;
            if ~(value<=context.levelMaximum*(1+1e-12))
                info.reason="levelAboveCertifiedRegion";return;
            end
            % The lateral extent is largest at the start and only shrinks.
            [ego,~,~,rate]=terminalSafeSet.tube(context,value,station,0,0);
            if ego.lateral(2)>context.clearance(2) || -ego.lateral(1)>context.clearance(1)
                info.reason="tubeLeavesRoad";return;
            end
            if ~context.hasTarget,ok=true;info.exitSeconds=0;info.reason="noTarget";return;end
            start=node*context.ratio;chunk=round(2/context.dt);
            steps=0;shift=0;cap=0;ending="";
            while true
                if steps==0,last=start;else,last=min(start+steps+chunk,start+cap);end
                context=terminalSafeSet.extend(context,last);
                tau=(steps:last-start)*context.dt;
                [ego,box,guard,rate]=terminalSafeSet.tube(context,value,station,tau,shift);
                index=start+(steps:last-start)+1;
                pointGap=terminalSafeSet.separation(context,box.station,box.lateral, ...
                    context.pointStation(:,index),context.pointLateral(:,index));
                exitAt=find(pointGap>context.range,1);
                apart=terminalSafeSet.collisionCone(context,ego,rate,index);
                bodyGap=terminalSafeSet.separation(context,ego.station,ego.lateral, ...
                    context.targetStation(:,index),context.targetLateral(:,index));
                required=max(context.buffer,guard+context.dt/2*context.targetSpeed(index));
                % Grid points before an exit must be clear; the collision-cone
                % certificate covers the time from its own grid point on, so
                % that grid point is checked as well.
                limit=numel(tau);reason="exit";stop=exitAt;
                if ~isempty(apart) && (isempty(exitAt) || apart<=exitAt),stop=apart;reason="outsideCollisionCone";end
                if ~isempty(stop),limit=stop-(reason=="exit");end
                if limit>0
                    margin=bodyGap(1:limit)-required(1:limit);info.margin=min(info.margin,min(margin));
                    if any(margin<0),info.reason="tubeMeetsTarget";return;end
                end
                if ~isempty(stop),ok=true;info.exitSeconds=tau(stop);info.reason=reason;return;end
                if steps==0
                    % Neither event at the node itself: the span to follow.
                    [cap,ending]=terminalSafeSet.scanBound(context,value,station,ego,box,rate,index);
                end
                % Neither event within the span: not terminal.
                if last>=start+cap,info.reason=ending;return;end
                shift=box.shift;steps=last-start+1;
            end
        end

        function [cap,ending] = scanBound(context,value,station,ego,box,rate,index)
            % Grid steps the time-resolved check may follow after a node whose
            % own point certified nothing, and the reason reported when the
            % span ends with neither event. cap = 0 rejects the node at once.
            %   Straight road, straight-line target: the relative motion is a
            %   parabola (collisionCone). If it enters the settled tube's box
            %   (level 0) at u_hit, the tube meets the target then unless the
            %   target has left the range before: the span is the first time
            %   the reference point is surely out of range, or none when that
            %   time is not before u_hit. Otherwise the parabola leaves the
            %   whole tube's box (widened by the total drift) for good at
            %   u_out, where the cone certifies at the latest: the span is
            %   u_out, or the tube's settling time when the parabola never
            %   leaves (a co-moving target).
            %   Circling target: the span is the time the tube's box has passed
            %   the target's disk, where the disk test certifies.
            %   Curved road: no cone; the span is the settling time plus the
            %   time the ego needs for the range diameter, 2R/v. A target still
            %   in range then has neither passed nor been passed.
            T=context.timeConstant;dt=context.dt;b=context.buffer;R=context.range;v=context.pathSpeed;
            r0=sqrt(max(value,0));remaining=T*rate;
            settle=max(remaining,context.scale(1)*r0);tolerance=max(b,1e-2);
            tSettle=0;if settle>tolerance,tSettle=T*log(settle/tolerance);end
            motion=context.targetMotion;ending="noExitWithinSpan";
            if context.curvature~=0 || ~motion.available
                cap=max(1,ceil((tSettle+2*R/max(v,.5))/dt));return;
            end
            if ~motion.straight
                behind=motion.centre(1)+motion.diskRadius+b-(ego.station(1)-remaining);
                cap=max(1,ceil(behind/max(v,.5)/dt));return;
            end
            ending="insideCollisionCone";
            angle=context.targetCourse(index)-context.pathHeading;c=cos(angle);s=sin(angle);
            V=context.targetVelocity(index);A=motion.acceleration;
            aS=c*A/2;bS=c*V-v;aD=s*A/2;bD=s*V;
            targetS=context.targetStation(:,index);targetD=context.targetLateral(:,index);
            gS=mean(targetS)-mean(ego.station);gD=mean(targetD)-mean(ego.lateral);
            settled=terminalSafeSet.tube(context,0,station,0,0);
            halfS0=diff(settled.station)/2+diff(targetS)/2+b;halfD0=diff(settled.lateral)/2+diff(targetD)/2+b;
            excess=rate*dt;
            halfS=diff(ego.station)/2+remaining+excess+diff(targetS)/2+b;halfD=diff(ego.lateral)/2+diff(targetD)/2+b;
            [entry,~]=localParabolaBoxSpan(aS,bS,gS,halfS0,aD,bD,gD,halfD0);
            if isfinite(entry)
                % The forecast meets the settled tube at 'entry'.
                radius=hypot(diff(box.station)/2+remaining+excess,diff(box.lateral)/2);
                gRefS=context.pointStation(1,index)-mean(box.station);gRefD=context.pointLateral(1,index)-mean(box.lateral);
                leave=localParabolaDiskExit(aS,bS,gRefS,aD,bD,gRefD,R+radius);
                if ~(leave<entry),cap=0;return;end
                cap=max(1,ceil(leave/dt)+1);return;
            end
            [~,out]=localParabolaBoxSpan(aS,bS,gS,halfS,aD,bD,gD,halfD);
            cap=max(1,ceil(min(out,tSettle)/dt)+1);
        end

        function first = collisionCone(context,ego,rate,index)
            % First grid point from which the target provably never meets the
            % tube's rectangle box again, or []: the target's forecast motion
            % relative to the box is outside the collision cone of the two
            % boxes. Only on a straight road.
            %   From a grid time the ego band across the road only shrinks; along
            %   the road the box moves at the trim's station rate v and drifts
            %   by at most T*sigma(r) more (sigma(lambda r) <= lambda sigma(r),
            %   r decays as exp(-t/T)). Widened by that drift, the ego box is
            %   fixed in a frame moving at v.
            %   A straight-line target (beta = 0) moves its box by c s(u) along
            %   and s s(u) across the road, s(u) = V u + A u^2/2, c and s the
            %   cosine and sine of its course to the road, also through a stop.
            %   In the moving frame the difference of the box centres is a
            %   parabola in u, and the boxes meet at some u >= 0 iff the
            %   parabola enters the sum box (half-widths H_s, H_d, with the
            %   buffer): the relative motion lies inside the collision cone.
            %   Equivalently, the least normalized box distance
            %   max(|q_s|/H_s, |q_d|/H_d) over u >= 0 is at most 1; it is
            %   attained at u = 0, at a vertex or zero of q_s or q_d, or where
            %   |q_s|/H_s = |q_d|/H_d, and is evaluated at all of these, so the
            %   test is exact for the forecast (localParabolaMeetsBox).
            %   A circling target (beta ~= 0) stays in the disk of its circle
            %   widened by its body reach: a disk beside the ego band, or behind
            %   it while the ego moves forward, stays apart.
            first=[];
            if context.curvature~=0,return;end
            motion=context.targetMotion;
            if ~motion.available,return;end
            v=context.pathSpeed;b=context.buffer;
            remaining=context.timeConstant*rate;
            lateral=ego.lateral;along=ego.station;
            if motion.straight
                angle=context.targetCourse(index)-context.pathHeading;
                c=cos(angle);s=sin(angle);speed=context.targetVelocity(index);A=motion.acceleration;
                target=context.targetLateral(:,index);station=context.targetStation(:,index);
                gapS=mean(station,1)-mean(along,1);halfS=diff(along,1,1)/2+remaining+diff(station,1,1)/2+b;
                gapD=mean(target,1)-mean(lateral,1);halfD=diff(lateral,1,1)/2+diff(target,1,1)/2+b;
                apart=~localParabolaMeetsBox(c*A/2,c.*speed-v,gapS,halfS,s*A/2,s.*speed,gapD,halfD);
            else
                radius=motion.diskRadius;centre=motion.centre;
                apart=(centre(2)-radius-lateral(2,:)>=b) | (lateral(1,:)-centre(2)-radius>=b) ...
                    | (along(1,:)-remaining-centre(1)-radius>=b & v>=0);
            end
            first=find(apart,1);
        end

        function [ego,box,guard,rate] = tube(context,value,station,tau,shift)
            % Path-coordinate boxes of the ego rectangle (ego) and of its
            % reference point (box) at times tau after the node, the distance
            % (guard) an ego body point can move in dt/2 there, and the bound
            % on the station-rate error (rate).
            radius=sqrt(max(value,0))*exp(-tau/context.timeConstant);
            a=context.scale;kappa=abs(context.curvature);
            lateral=a(1)*radius+context.positionRadius;
            heading=min(pi/2,abs(context.yawOffset)+a(2)*radius+context.yawRadius);
            % |v_t - v_t*| for theta = psi* + e_psi, |e_psi| <= a2 r (TERMINAL_SAFE_SET.md, (3.2)):
            % |dvx| + |dvy| (|sin psi*| + a2 r) + |vx*| (|sin psi*| a2 r + (a2 r)^2/2) + |vy*| a2 r;
            % the heading enters a straight road's station rate only at second order.
            yaw=a(2)*radius;tilt=abs(sin(context.yawOffset));
            tangentError=a(3)*radius+a(4)*radius.*(tilt+yaw)+abs(context.velocity(1))*(tilt*yaw+yaw.^2/2) ...
                +abs(context.velocity(2))*yaw;
            speedError=(tangentError+context.pathSpeed*kappa*a(1)*radius)./max(1-kappa*a(1)*radius,.5);
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
            box.shift=drift(end)+speedError(end)*context.dt;rate=speedError;
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
            context.targetCourse=[context.targetCourse,q(3,:)+q(6,:)];
            context.targetVelocity=[context.targetVelocity,q(4,:)];
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

function hit = localParabolaMeetsBox(aS,bS,gS,hS,aD,bD,gD,hD)
    % Per column: does the parabola (aS u^2 + bS u + gS, aD u^2 + bD u + gD),
    % u >= 0, enter the box |q_s| <= hS, |q_d| <= hD? Exact: the least
    % normalized box distance over u >= 0 is attained at u = 0, at a vertex or
    % zero of either coordinate, or where the two normalized distances are
    % equal, and it is evaluated at all of them.
    n=numel(gS);aS=aS+zeros(1,n);aD=aD+zeros(1,n);
    u=[zeros(1,n);localVertex(aS,bS);localVertex(aD,bD);localRoots(aS,bS,gS);localRoots(aD,bD,gD); ...
        localRoots(aS./hS-aD./hD,bS./hS-bD./hD,gS./hS-gD./hD); ...
        localRoots(aS./hS+aD./hD,bS./hS+bD./hD,gS./hS+gD./hD)];
    u(u<0)=NaN;
    qS=aS.*u.^2+bS.*u+gS;qD=aD.*u.^2+bD.*u+gD;
    distance=max(abs(qS)./hS,abs(qD)./hD);
    hit=min(distance,[],1,'omitnan')<=1+1e-9;
end

function [entry,leave] = localParabolaBoxSpan(aS,bS,gS,hS,aD,bD,gD,hD)
    % First and last time u >= 0 at which the parabola (aS u^2 + bS u + gS,
    % aD u^2 + bD u + gD) is inside the box |q_s| <= hS, |q_d| <= hD; Inf and
    % Inf when never, entry and Inf when it stays inside.
    bandS=localBand(aS,bS,gS,hS);bandD=localBand(aD,bD,gD,hD);
    entry=Inf;leave=Inf;hit=false;
    for i=1:size(bandS,1)
        for j=1:size(bandD,1)
            lo=max(bandS(i,1),bandD(j,1));hi=min(bandS(i,2),bandD(j,2));
            if lo<=hi
                if ~hit,leave=-Inf;hit=true;end
                entry=min(entry,lo);leave=max(leave,hi);
            end
        end
    end
end

function band = localBand(a,b,g,h)
    % {u >= 0 : |a u^2 + b u + g| <= h} as rows [lo, hi] (hi may be Inf).
    f=@(u)a*u.^2+b*u+g;
    points=[localScalarRoots(a,b,g-h),localScalarRoots(a,b,g+h)];points=sort(points(points>0));
    edges=[0,points,Inf];band=zeros(0,2);
    for k=1:numel(edges)-1
        lo=edges(k);hi=edges(k+1);
        if isinf(hi),probe=lo+1;else,probe=(lo+hi)/2;end
        if abs(f(probe))<=h*(1+1e-9)+1e-12
            if ~isempty(band) && band(end,2)==lo,band(end,2)=hi;else,band(end+1,:)=[lo,hi];end %#ok<AGROW>
        end
    end
end

function r = localScalarRoots(a,b,c)
    % Real roots of a u^2 + b u + c = 0 (scalar coefficients).
    if a==0
        if b==0,r=zeros(1,0);else,r=-c/b;end
        return;
    end
    disc=b^2-4*a*c;
    if disc<0,r=zeros(1,0);return;end
    sgn=sign(b);if sgn==0,sgn=1;end
    q=-(b+sgn*sqrt(disc))/2;
    if q==0,r=0;else,r=[q/a,c/q];end
end

function first = localParabolaDiskExit(aS,bS,gS,aD,bD,gD,rho)
    % First time u >= 0 at which the parabola is farther than rho from the
    % origin, Inf when never.
    p=conv([aS,bS,gS],[aS,bS,gS])+conv([aD,bD,gD],[aD,bD,gD]);p(end)=p(end)-rho^2;
    if polyval(p,0)>0,first=0;return;end
    r=roots(p);r=sort(real(r(abs(imag(r))<1e-9*max(1,abs(r)) & real(r)>0))).';
    first=Inf;
    for k=1:numel(r)
        if k<numel(r),probe=(r(k)+r(k+1))/2;else,probe=r(k)+1;end
        if polyval(p,probe)>0,first=r(k);return;end
    end
end

function u = localVertex(a,b)
    u=nan(size(b));at=a~=0;u(at)=-b(at)./(2*a(at));
end

function r = localRoots(a,b,c)
    % Real roots of a u^2 + b u + c = 0 per column, two rows, NaN where absent.
    % Stable for small a: q = -(b + sign(b) sqrt(disc))/2, roots q/a and c/q.
    n=numel(c);r=nan(2,n);
    linear=a==0;at=linear & b~=0;r(1,at)=-c(at)./b(at);
    disc=b.^2-4*a.*c;ok=~linear & disc>=0;
    sgn=sign(b);sgn(sgn==0)=1;q=-(b+sgn.*sqrt(max(disc,0)))/2;
    r(1,ok)=q(ok)./a(ok);
    at=ok & q~=0;r(2,at)=c(at)./q(at);
end

function motion = localTargetMotion(model)
    % Constants of the target's forecast used by the separation certificate:
    % its tangential acceleration, and for beta ~= 0 the centre of its circle
    % in path coordinates and the circle's radius widened by the body reach.
    motion=struct('available',false,'straight',true,'acceleration',0,'diskRadius',Inf,'centre',[NaN;NaN]);
    q=model.targetEpoch;
    if isempty(q),return;end
    motion.available=true;motion.acceleration=q(5);
    curvature=sin(q(6))/q(7);
    if curvature==0,return;end
    motion.straight=false;
    course=q(3)+q(6);
    centre=q(1:2)+[-sin(course);cos(course)]/curvature;
    projection=laneGeometry.project(centre,model.lane);
    motion.centre=[projection.station;projection.lateralPosition];
    motion.diskRadius=1/abs(curvature)+norm(q(8:9))+norm(q(10:11));
end
