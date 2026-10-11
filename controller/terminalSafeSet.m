classdef terminalSafeSet
%terminalSafeSet Terminal set of the braking continuation, and the anchor's holds.
% Past the plan's endpoint both vehicles continue in the constant tangential
% acceleration and sideslip family of the target contract (Sharma et al.,
% 2026, Eq. (2)). The target keeps its forecast A and beta. The ego follows
% the terminal law
%   kappa_f(z) = ( A = max(-a_b, -V/h),  beta held ),
% a = terminal.brakingDecelerationMetersPerSecondSquared, h the sample time:
% it brakes at a_b with its sideslip angle beta = atan2(v_y, v_x) and its
% course curvature r/V held, and stops exactly at the end of a hold, after
% J = ceil(V/(a_b h)) holds. phi, the joint one-step map under this law, is
% exact and in closed form: both vehicles move on arcs of constant curvature
% (continuation, successor). The terminal set is
%   X_f = { z : the law is admissible at z (sideslip cone; braking and the
%           held arc's lateral acceleration r V inside the friction circle),
%           and along phi^j(z), j >= 0,
%           - every node, hold midpoint and the parked pose keeps the ego
%             rectangle on the road,
%           - every node and hold midpoint clears the target's forecast
%             rectangle by safetyMarginMeters until the target leaves
%             encounterRangeMeters,
%           - after the stop, the target's remaining track until it leaves
%             the range clears the parked rectangle by the same margin }.
% The track check is exact and in closed form (parkedClearance): the track is
% a stretch of the target's line or circle that ends where the target leaves
% the range or comes to rest, or the whole circle; braking ends at rest for
% both vehicles and never turns either back. Rectangle against
% track reduces to vertex paths (segments or arcs) against edges, the
% rectangle form of the point-to-arc distance. The target's first exit ends
% the encounter: no property of the target after it is used.
% X_f is forward invariant under the law by construction: the continuation of
% phi(z) is the remainder of the continuation of z, at the same absolute
% times, and the exit is the same. It is the largest set that this law keeps
% safe. The ego enclosure, when present, widens every clearance by its
% position radius and the chord of its yaw radius. See TERMINAL_SAFE_SET.md.
%
% The ego's continuation is a certificate evaluated at the declared model's
% state, not a hold of that model. Holds appended to the anchor are holds of
% the declared model, kept only when their endpoint is in X_f: the hold of the
% CLF's certified cruise controller u = u* + K e (cruiseInput), or else the
% hold nearest to it toward braking with the steering held, as hard as the
% rear adhesion admits or at the law's rate a_b (brakingInput). jacobians, rayJacobians, certificate and stateLevel belong
% to the offline certificate of the cruise controller and the CLF
% (NOMINAL_CLF.md).
    methods (Static)
        function context = context(model)
            % Constants of one frame.
            cfg=model.cfg;positionRadius=0;yawRadius=0;
            if isfield(model,'uncertainty') && isfield(model.uncertainty,'egoGenerator')
                generator=model.uncertainty.egoGenerator;
                positionRadius=sum(vecnorm(generator(1:2,:),2,1));yawRadius=sum(abs(generator(3,:)));
            end
            shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
            reach=norm(shape(1:2))+norm(shape(3:4));
            clearance=[Inf;Inf];
            if ~isempty(model.road.lateralClearance),clearance=model.road.lateralClearance;end
            here=laneGeometry.project(model.initialState(1:2),model.lane);
            context=struct('deceleration',cfg.terminal.brakingDecelerationMetersPerSecondSquared, ...
                'sampleTime',cfg.controller.sampleTime, ...
                'friction',min(cfg.tire.frictionCoefficient)*cfg.vehicle.gravity, ...
                'sideslipMaximum',cfg.model.sideslipMaximum,'shape',shape,'reach',reach, ...
                'margin',cfg.collision.safetyMarginMeters,'required',cfg.collision.safetyMarginMeters+positionRadius+2*reach*sin(min(pi,yawRadius)/2), ...
                'inflation',positionRadius+2*reach*sin(min(pi,yawRadius)/2), ...
                'range',cfg.collision.encounterRangeMeters,'clearance',clearance, ...
                'hasTarget',~isempty(model.target),'targetEpoch',model.targetEpoch, ...
                'targetEpochIndex',localEpochIndex(model), ...
                'sampleIndex',model.sampleIndex,'lane',model.lane,'stationNow',here.station, ...
                'reference',model.nominalReference);
        end

        function [member,context,info] = member(context,x,node)
            % x at plan node 'node' (0 = now). info records how the
            % continuation ends ('exit', 'parked' or 'noTarget'), the time to
            % the stop and to the target's exit after x, the least clearance
            % beyond the requirement, and the CLF value of x.
            [member,info]=localDecide(context,x,node);
            info.value=nonlinearBicycleModel.nominalValue(x,context.lane,context.reference);
        end

        function path = continuation(context,x,steps)
            % The ego's continuation from x under the terminal law, in closed
            % form: its poses (columns [px; py; yaw]) at times k h/2 after x,
            % k = 0..2J, nodes and hold midpoints until it stops at node J.
            % The speed falls by a_b h per hold, and by what is left in the
            % last one; the arc length is exact for that piecewise linear
            % speed. 'steps' fixes J (to differentiate with the schedule held).
            h=context.sampleTime;a=context.deceleration;V=hypot(x(4),x(5));
            beta=0;kappa=0;
            if V>0,beta=atan2(x(5),x(4));kappa=x(6)/V;end
            if nargin<3
                steps=0;if V>0,steps=ceil(V/(a*h));end
            end
            speed=[V-(0:steps-1)*a*h,0];
            if steps==0,speed=V;end
            node=[0,cumsum((speed(1:end-1)+speed(2:end))*h/2)];
            arc=zeros(1,2*steps+1);arc(1:2:end)=node;
            arc(2:2:end)=node(1:end-1)+(3*speed(1:end-1)+speed(2:end))*h/8;
            half=kappa*arc/2;scale=ones(size(half));
            bend=half~=0;scale(bend)=sin(half(bend))./half(bend);
            position=x(1:2)+(arc.*scale).*predictiveSafetyGeometry.direction(x(3)+beta+half);
            path=struct('pose',[position;x(3)+kappa*arc],'time',(0:2*steps)*h/2,'arc',arc, ...
                'steps',steps,'speed',V,'sideslip',beta,'curvature',kappa);
        end

        function clear = clears(context,x,node)
            % Does the ego rectangle at x, at plan node 'node' (a half node is
            % a hold midpoint), meet the problem's own sampled collision row:
            % safetyMarginMeters from the target's forecast, or the target
            % beyond the range there.
            clear=true;if ~context.hasTarget,return;end
            q=localTarget(context,node,0);
            if norm(x(1:2)-q(1:2))>context.range,return;end
            clear=predictiveSafetyGeometry.rectangle(x(1:3),context.shape,q(1:3),q(8:11))>=context.margin-1e-6;
        end

        function next = successor(context,x)
            % The ego part of phi: one hold of the terminal law from x, as a
            % state with the same sideslip and course curvature (zero speed
            % once stopped). Its continuation is that of x shifted by a hold.
            path=terminalSafeSet.continuation(context,x);
            if path.steps==0,next=[x(1:3);0;0;0];return;end
            V=max(path.speed-context.deceleration*context.sampleTime,0);
            next=[path.pose(:,3);V*cos(path.sideslip);V*sin(path.sideslip);path.curvature*V];
        end

        function [rows,bounds,context,info] = rows(context,y,node)
            % The terminal set linearized at the anchor endpoint y on the
            % endpoint correction dz, rows*dz <= bounds:
            % - the friction circle of the law, |r V| <= sqrt((mu g)^2 - a_b^2),
            %   only where y is outside it (inside it the acceptance check
            %   keeps it: as a hard row on the boundary it left the CLF
            %   stage's cone program without a numerical result in a hard
            %   recovery turn); the sideslip cone is a row of the problem at
            %   every node;
            % - fixed-dual collision rows at the continuation's nodes and
            %   midpoints before the exit, and at the parked pose against the
            %   target's closest pose on its track (the minimizer's own rows);
            % - corner rows on the road at the continuation's poses.
            % The continuation's poses are differentiated with the schedule
            % held. Conditions whose clearance exceeds the requirement by more
            % than localBand are left out; the exact set decides acceptance.
            % Each row keeps up to localReserve of the margin it has at y, so
            % that an endpoint off the boundary stays off it by that much and
            % the next sample's shift survives a small departure (the plant
            % or the appended hold's departure from the law); a row is never
            % tighter than y itself, so when y is in X_f the zero correction
            % meets every row.
            [~,info]=localDecide(context,y,node);
            info.value=nonlinearBicycleModel.nominalValue(y,context.lane,context.reference);
            band=localBand();required=context.required;
            path=terminalSafeSet.continuation(context,y);
            jacobian=localPoseJacobian(context,y,path.steps);
            V=hypot(y(4),y(5));limit=sqrt(max(context.friction^2-context.deceleration^2,0));
            rows=zeros(0,6);bounds=zeros(0,1);
            if abs(y(6)*V)>limit
                gradient=sign(y(6))*[0,0,0,y(6)*y(4)/V,y(6)*y(5)/V,V];
                rows=gradient;bounds=limit-abs(y(6)*V);
            end
            [margin,lateral,normal,local]=localRoad(context,path.pose);
            for k=find(margin<context.inflation+band)
                c=cos(path.pose(3,k));s=sin(path.pose(3,k));
                for corner=1:4
                    derivative=normal(:,corner,k).'*[eye(2),[-s,-c;c,-s]*local(:,corner)]*jacobian(:,:,k);
                    rows=[rows;derivative;-derivative]; %#ok<AGROW>
                    bounds=[bounds;localKeep(context.clearance(2)-lateral(corner,k)-context.inflation); ...
                        localKeep(context.clearance(1)+lateral(corner,k)-context.inflation)]; %#ok<AGROW>
                end
            end
            if ~context.hasTarget,return;end
            q=localTarget(context,node,path.time);
            [gap,out]=localClearances(context,path.pose,q);
            last=numel(gap);if ~isempty(out),last=out-1;end
            for k=find(gap(1:last)<required+band)
                dual=localDual(path.pose(:,k),context.shape,q(:,k),[]);
                if isempty(dual),continue;end
                rows=[rows;-dual.jacobian*jacobian(:,:,k)];bounds=[bounds;localKeep(dual.value-required)]; %#ok<AGROW>
            end
            if isempty(out)
                [distance,witness]=terminalSafeSet.parkedClearance(context,path.pose(:,end),q(:,end));
                if distance<required+band
                    % A witness at exact contact (a vertex path crossing an
                    % edge) has no well-defined normal; a pose 1 mm or 1 cm
                    % along the track stands for it.
                    dual=localDual(path.pose(:,end),context.shape,witness,[1e-3,-1e-3,1e-2,-1e-2]);
                    if ~isempty(dual)
                        rows=[rows;-dual.jacobian*jacobian(:,:,end)];bounds=[bounds;localKeep(dual.value-required)];
                    end
                end
            end
        end

        function [distance,witness,track] = parkedClearance(context,pose,q)
            % Least distance between the ego rectangle parked at 'pose' and the
            % target rectangle over the target's remaining track from its
            % state q until it first leaves the range about the parked
            % reference point or comes to rest, and the target state
            % attaining it (witness).
            % Exact when the two are apart at q: the distance is continuous
            % along the track, and while the rectangles are apart it is the
            % least distance from a vertex of one to an edge of the other, so
            % the least distance over the track is the least distance from a
            % vertex path (segment or arc) to an edge (localTrackDistance).
            [lower,upper,full,exitSeconds]=localTrack(q,pose(1:2),context.range);
            [distance,sigma]=localTrackDistance(pose,context.shape,q,lower,upper,full);
            witness=localAlong(q,sigma);
            track=struct('lower',lower,'upper',upper,'fullCircle',full,'exitSeconds',exitSeconds);
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

        function [input,next,ok,value,middle] = cruiseInput(x,model)
            % One hold of the CLF's certified cruise controller u = u* + K e(x),
            % the gain certified offline together with P (NOMINAL_CLF.md). On
            % the certified level set the hold satisfies V(next) <= rho V(x)
            % and the input bound. ok evaluates the decrease and the hard rows
            % on this hold. The anchor appends it while its endpoint stays in
            % the terminal set.
            cfg=model.cfg;reference=model.nominalReference;lane=model.lane;
            e=nonlinearBicycleModel.error(x,lane,reference);
            input=reference.input+reference.gain*e;
            low=max(-1+1e-8,cfg.actuation.brakingRatioMinimum);high=min(1-1e-8,cfg.actuation.brakingRatioMaximum);
            input(2)=min(high,max(low,input(2)));
            bound=reference.contraction*sum((reference.factor*e).^2)*(1+1e-9)+1e-12;
            next=[];middle=[];ok=false;value=Inf;
            try
                [middle,next]=terminalSafeSet.holdStates(x,input,cfg);
            catch exception
                if startsWith(string(exception.identifier),"collisionAvoidanceController:"),return;end
                rethrow(exception);
            end
            value=nonlinearBicycleModel.nominalValue(next,lane,reference);
            ok=value<=bound && terminalSafeSet.admissible(x,middle,next,input,model);
        end

        function [input,next,ok,middle,fraction] = brakingInput(x,previous,cruise,model,context,node)
            % One hold of the declared model from x at plan node 'node' that
            % keeps the endpoint in X_f, meets the hard rows and clears the
            % target at its midpoint, as close as found to the cruise input.
            % The far end of the search holds the previous steering and
            % brakes with the ratio of actuator deceleration a_b, the law's
            % realization (road load neglected), or else as hard as the rear
            % adhesion at x admits (with 5 percent to spare); a far end that
            % keeps the endpoint in X_f is bisected (five halvings) toward the
            % cruise input along u = cruise + lambda (far - cruise), lambda in
            % (0, 1], and the least lambda found is returned. ok is false when
            % neither far end keeps the endpoint in X_f. The realization
            % follows the law's continuation only approximately; braking
            % harder in the first hold leaves the continuation behind it
            % slower. Every candidate is checked.
            cfg=model.cfg;
            low=max(-1+1e-8,cfg.actuation.brakingRatioMinimum);
            tire=modifiedFialaTire.parameters(cfg);
            k=3*tire.longitudinalForceScale(2)/tire.corneringStiffness(2);
            adhesion=sqrt(max(0,1-((x(5)-cfg.vehicle.lr*x(6))/(k*x(4)))^2));
            law=-cfg.terminal.brakingDecelerationMetersPerSecondSquared/modifiedFialaTire.accelerationGain(cfg);
            fraction=1;ok=false;
            for braking=[max(low,law),max(low,-.95*adhesion)]
                input=[previous(1);braking];
                [ok,next,middle]=localHoldStays(x,input,model,context,node);
                if ok,break;end
            end
            if ~ok,return;end
            far=input;lower=0;upper=1;
            for halving=1:5
                lambda=(lower+upper)/2;candidate=cruise+lambda*(far-cruise);
                [stays,candidateNext,candidateMiddle]=localHoldStays(x,candidate,model,context,node);
                if stays
                    upper=lambda;input=candidate;next=candidateNext;middle=candidateMiddle;fraction=lambda;
                else
                    lower=lambda;
                end
            end
        end

        function level = stateLevel(reference,cfg)
            % Largest c with {e : e'Pe <= c} inside the linear state rows g'e <= b:
            % c = min b^2/(g' inv(P) g). Part of the cruise controller's certificate.
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

        function [a,b,plus] = jacobians(reference,cfg,errors,inputs,duration)
            % Jacobians of the sampled error map at the states x(e) on the
            % trim's own lane: for the columns e of errors and du of inputs,
            % e(s) = g(e, du) over the first 'duration' seconds of a hold with
            % the input u* + du, a = dg/de (5x5xn) and b = dg/ddu (5x2xn), from
            % the RK4 variational equations and the error chart, and plus =
            % g(e, du) itself (5xn); NaN where the model's domain is left.
            % scripts/synthesizeClfMatrices encloses their ray averages
            % (rayJacobians) over the certified set (NOMINAL_CLF.md).
            if nargin<5,duration=cfg.controller.sampleTime;end
            curve=struct('origin',[0;0],'heading',0,'curvature',reference.curvature,'length',200);
            lane=struct('referenceCurve',curve);
            [~,heading]=laneGeometry.referencePose(50,0,curve);
            chart=[[-sin(heading);cos(heading)],zeros(2,4);zeros(4,1),eye(4)];
            n=size(errors,2);a=nan(5,5,n);b=nan(5,2,n);plus=nan(5,n);
            for j=1:n
                e=errors(:,j);
                [position,yaw]=laneGeometry.referencePose(50,e(1),curve);
                x=[position;yaw+reference.state(3)+e(2);reference.state(4:6)+e(3:5)];
                try
                    [next,ax,bx]=nonlinearBicycleModel.sample(x,reference.input+inputs(:,j),cfg,[],duration);
                catch exception
                    if startsWith(string(exception.identifier),"collisionAvoidanceController:"),continue;end
                    rethrow(exception);
                end
                [plus(:,j),jacobian]=nonlinearBicycleModel.errorLinearization(next,lane,reference);
                a(:,:,j)=jacobian*ax*chart;b(:,:,j)=jacobian*bx;
            end
        end

        function [a,b,gap] = rayJacobians(reference,cfg,errors,inputs,duration,nodes)
            % Ray-averaged Jacobians of the sampled error map,
            %   Gbar(e, du) = integral_0^1 G(t e, t du) dt,
            % by Gauss-Legendre quadrature with 'nodes' nodes (a = 5x5xn,
            % b = 5x2xn), and gap = g(e, du) - Gbar [e; du] (5xn), the
            % residual of the exact identity g(e, du) = Gbar [e; du] (the trim
            % is a fixed point of the map) that the quadrature leaves. For
            % the saturating tire the ray averages spread like the secants of
            % its curve, not like its tangents, which is why the certificate
            % encloses them (NOMINAL_CLF.md). NaN where the model's domain is
            % left.
            if nargin<5 || isempty(duration),duration=cfg.controller.sampleTime;end
            if nargin<6,nodes=8;end
            beta=.5./sqrt(1-(2*(1:nodes-1)).^(-2));tridiagonal=diag(beta,1)+diag(beta,-1);
            [vectors,values]=eig(tridiagonal);[points,order]=sort(diag(values));
            weights=vectors(1,order).^2;points=(points.'+1)/2;
            n=size(errors,2);a=zeros(5,5,n);b=zeros(5,2,n);
            for q=1:nodes
                [aq,bq]=terminalSafeSet.jacobians(reference,cfg,points(q)*errors,points(q)*inputs,duration);
                a=a+weights(q)*aq;b=b+weights(q)*bq;
            end
            [~,~,plus]=terminalSafeSet.jacobians(reference,cfg,errors,inputs,duration);
            gap=nan(5,n);
            for j=1:n,gap(:,j)=plus(:,j)-a(:,:,j)*errors(:,j)-b(:,:,j)*inputs(:,j);end
        end

        function result = certificate(reference,cfg,errors,subdivisions)
            % Sampled check of the cruise controller on the trim's own lane.
            % For each column e of errors (transverse errors), one hold from
            % x(e) under u = u* + K e gives
            %   contraction  V(x+) / V(x);
            %   remainder    ||F (e(x+) - (A0 + B0 K) e)|| / ||F e||, F = chol(P),
            %                the departure from the trim's Jacobian in CLF units;
            %   holdFactor   max over s in (0, h] of exp(s/T) sqrt(V(x(s)) / V(x)),
            %                the excess of the CLF radius inside the hold;
            %   rows         state and handling rows at the hold's midpoint and
            %                endpoint;
            % NaN where the model's domain is left. The certificate itself is
            % the robust LMI of scripts/synthesizeClfMatrices over the enclosure
            % of the Jacobians (NOMINAL_CLF.md); these samples test it
            % independently.
            if nargin<4,subdivisions=10;end
            curve=struct('origin',[0;0],'heading',0,'curvature',reference.curvature,'length',200);
            lane=struct('referenceCurve',curve);
            f=reference.factor;closed=reference.nominalA+reference.nominalB*reference.gain;
            h=cfg.controller.sampleTime;T=cfg.clf.convergenceTimeConstantSeconds;
            n=size(errors,2);
            result=struct('contraction',nan(1,n),'remainder',nan(1,n),'holdFactor',nan(1,n), ...
                'rows',false(1,n),'input',nan(2,n));
            for j=1:n
                e=errors(:,j);value=sum((f*e).^2);
                [position,heading]=laneGeometry.referencePose(50,e(1),curve);
                x=[position;heading+reference.state(3)+e(2);reference.state(4:6)+e(3:5)];
                u=reference.input+reference.gain*e;result.input(:,j)=u;
                try
                    [middle,next]=terminalSafeSet.holdStates(x,u,cfg);
                    factor=0;
                    for s=h*(1:subdivisions)/subdivisions
                        y=nonlinearBicycleModel.sample(x,u,cfg,[],s);
                        factor=max(factor,exp(s/T)*sqrt(nonlinearBicycleModel.nominalValue(y,lane,reference)/value));
                    end
                catch exception
                    if startsWith(string(exception.identifier),"collisionAvoidanceController:"),continue;end
                    rethrow(exception);
                end
                plus=nonlinearBicycleModel.error(next,lane,reference);
                result.contraction(j)=sum((f*plus).^2)/value;
                result.remainder(j)=norm(f*(plus-closed*e))/sqrt(value);
                result.holdFactor(j)=factor;
                result.rows(j)=terminalSafeSet.stateRows(next,cfg) && terminalSafeSet.stateRows(middle,cfg) ...
                    && terminalSafeSet.handlingRows(x,u,cfg) && terminalSafeSet.handlingRows(next,u,cfg);
            end
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

function [stays,next,middle] = localHoldStays(x,u,model,context,node)
    % Does one hold of the declared model with input u from x at plan node
    % 'node' meet the hard rows, clear the target at its midpoint and end in
    % X_f?
    stays=false;next=[];middle=[];
    try
        [middle,next]=terminalSafeSet.holdStates(x,u,model.cfg);
    catch exception
        if startsWith(string(exception.identifier),"collisionAvoidanceController:"),return;end
        rethrow(exception);
    end
    stays=terminalSafeSet.admissible(x,middle,next,u,model) && terminalSafeSet.clears(context,middle,node+.5) ...
        && terminalSafeSet.member(context,next,node+1);
end

function dual = localDual(pose,shape,q,shifts)
    % The fixed-dual rows of the ego rectangle at 'pose' against the target
    % state q, or, where the closest features give no consistent dual (an
    % exact contact), against q moved along its track by each of 'shifts' in
    % turn; [] when none has one, and the row is left out (the exact set
    % decides acceptance).
    dual=[];
    for shift=[0,shifts]
        target=q;if shift~=0,target=localAlong(q,shift);end
        try
            dual=predictiveSafetyGeometry.supportLinearization(pose,shape,target(1:3),target(8:11));return;
        catch exception
            if exception.identifier~="MATLAB:assertion:failed",rethrow(exception);end
        end
    end
end

function bound = localKeep(margin)
    % A row's bound that keeps up to localReserve of the margin it has.
    bound=margin-min(localReserve(),max(margin,0));
end

function reserve = localReserve()
    % Margin [m] that the terminal rows keep where the anchor has it.
    reserve=.05;
end

function band = localBand()
    % Clearance and road margin beyond the requirement [m] up to which a
    % continuation condition enters the convex problem's terminal rows.
    band=5;
end

function [member,info] = localDecide(context,x,node)
    % Membership of x at plan node 'node' in X_f, evaluated in this order:
    % the law's admissibility, the road along the continuation, the target at
    % its nodes and midpoints until the exit, and the parked pose against the
    % target's remaining track.
    info=struct('reason',"",'end',"",'stopSeconds',NaN,'exitSeconds',NaN, ...
        'margin',Inf,'roadMargin',Inf);
    member=false;V=hypot(x(4),x(5));
    if V>0 && (abs(x(5))>tan(context.sideslipMaximum)*x(4)*(1+1e-9) ...
            || hypot(context.deceleration,x(6)*V)>context.friction)
        info.reason="terminalLawInadmissible";return;
    end
    path=terminalSafeSet.continuation(context,x);
    info.stopSeconds=path.steps*context.sampleTime;
    margin=localRoad(context,path.pose);
    info.roadMargin=min(margin)-context.inflation;
    if info.roadMargin<-1e-6,info.reason="continuationLeavesRoad";return;end
    if ~context.hasTarget,member=true;info.reason="noTarget";info.end="noTarget";return;end
    q=localTarget(context,node,path.time);
    [gap,out]=localClearances(context,path.pose,q);
    last=numel(gap);if ~isempty(out),last=out-1;end
    if last>0,info.margin=min(gap(1:last))-context.required;end
    if info.margin<-1e-6,info.reason="continuationMeetsTarget";return;end
    if ~isempty(out)
        member=true;info.reason="exit";info.end="exit";info.exitSeconds=path.time(out);return;
    end
    [distance,~,track]=terminalSafeSet.parkedClearance(context,path.pose(:,end),q(:,end));
    info.margin=min(info.margin,distance-context.required);
    info.exitSeconds=info.stopSeconds+track.exitSeconds;
    if distance-context.required<-1e-6,info.reason="parkedEgoMeetsTarget";return;end
    member=true;info.reason="parked";info.end="parked";
end

function q = localTarget(context,node,time)
    % The target's forecast at times 'time' after plan node 'node'.
    q=predictiveSafetyGeometry.predictTarget(context.targetEpoch, ...
        (context.sampleIndex-context.targetEpochIndex+node)*context.sampleTime+time);
end

function index = localEpochIndex(model)
    % Sample index of the target epoch (0 when a model carries none).
    index=0;if isfield(model,'targetEpochIndex'),index=model.targetEpochIndex;end
end

function [gap,out] = localClearances(context,pose,q)
    % Rectangle clearance at each continuation pose until the first pose
    % whose reference point is farther than the range from the target's (out,
    % [] when none; gap is NaN from there). Where the reference distance less
    % both reaches already exceeds the requirement by the band, that lower
    % bound stands for the clearance.
    n=size(pose,2);gap=nan(1,n);
    distance=vecnorm(pose(1:2,:)-q(1:2,:),2,1);
    out=find(distance>context.range,1);
    last=n;if ~isempty(out),last=out-1;end
    bound=distance-context.reach-vecnorm(q(8:9,:),2,1)-vecnorm(q(10:11,:),2,1);
    for k=1:last
        if bound(k)>=context.required+localBand()
            gap(k)=bound(k);
        else
            gap(k)=predictiveSafetyGeometry.rectangle(pose(:,k),context.shape,q(1:3,k),q(8:11,k));
        end
    end
end

function [margin,lateral,normal,local] = localRoad(context,pose)
    % Least distance of an ego corner inside the road at each pose (Inf
    % without road clearance), with each corner's lateral position, the path
    % normal at its projection and the corners in the body frame.
    shape=context.shape;n=size(pose,2);
    local=shape(3:4)+shape(1:2).*[1,1,-1,-1;1,-1,1,-1];
    margin=Inf(1,n);lateral=zeros(4,n);normal=zeros(2,4,n);
    if all(isinf(context.clearance)),return;end
    c=cos(pose(3,:));s=sin(pose(3,:));points=zeros(2,4,n);
    for corner=1:4
        points(:,corner,:)=reshape(pose(1:2,:)+[c*local(1,corner)-s*local(2,corner); ...
            s*local(1,corner)+c*local(2,corner)],2,1,n);
    end
    projection=laneGeometry.project(reshape(points,2,[]),context.lane,context.stationNow);
    lateral=reshape(projection.lateralPosition,4,n);
    heading=reshape(projection.heading,1,4,n);
    normal=[-sin(heading);cos(heading)];
    margin=min([context.clearance(2)-lateral;context.clearance(1)+lateral],[],1);
end

function jacobian = localPoseJacobian(context,y,steps)
    % d pose / d y at every continuation pose (3x6xK), central differences of
    % the closed-form continuation with the schedule held.
    n=2*steps+1;jacobian=zeros(3,6,n);delta=[1e-6;1e-6;1e-7;1e-6;1e-6;1e-7];
    for i=1:6
        e=zeros(6,1);e(i)=delta(i);
        plus=terminalSafeSet.continuation(context,y+e,steps);
        minus=terminalSafeSet.continuation(context,y-e,steps);
        jacobian(:,i,:)=reshape((plus.pose-minus.pose)/(2*delta(i)),3,1,n);
    end
end

function [lower,upper,full,exitSeconds] = localTrack(q,centre,range)
    % The stretch [lower, upper] of arc length along the target's track
    % (relative to its position in q) that its reference point sweeps from q
    % on until it first leaves the disk of radius 'range' about 'centre';
    % full when it sweeps its whole circle; and the time of that exit (Inf
    % when it never leaves). The speed is V + A t until it reaches zero:
    % braking ends at rest at the arc length -V^2/(2A), and a target at rest
    % moves off only forward (predictiveSafetyGeometry.targetArc).
    V=q(4);A=q(5);k=sin(q(6))/q(7);course=q(3)+q(6);full=false;exitSeconds=Inf;
    % The arc-length interval [a, b] (a <= 0 <= b) within range.
    if k==0
        u=predictiveSafetyGeometry.direction(course);w=q(1:2)-centre;
        p=u.'*w;c=w.'*w-range^2;root=sqrt(max(p^2-c,0));
        a=min(0,-p-root);b=max(0,-p+root);
    else
        middle=q(1:2)+[-sin(course);cos(course)]/k;rho=1/abs(k);
        d=middle-centre;w=q(1:2)-middle;dd=norm(d);C=Inf;
        if dd>0,C=(range^2-dd^2-rho^2)/(2*rho*dd);end
        if C>=1
            a=-Inf;b=Inf;
        else
            % Out of range where |y| < alpha, y the angle of the target about
            % its centre measured from the side facing away from 'centre'.
            alpha=acos(max(-1,C));angle=atan2(w(2),w(1))-atan2(d(2),d(1));
            y=atan2(sin(angle),cos(angle));
            if y>=0,up=2*pi-alpha-y;down=y-alpha;else,up=-alpha-y;down=2*pi-alpha+y;end
            up=max(up,0);down=max(down,0);
            if k>0,a=-down/k;b=up/k;else,a=up/k;b=-down/k;end
        end
    end
    if V==0 && A<=0,lower=0;upper=0;return;end
    direction=sign(V);if direction==0,direction=1;end
    leave=Inf;
    if V*A>=0
        if direction>0,lower=0;upper=b;leave=b;else,lower=a;upper=0;leave=a;end
    else
        rest=-V^2/(2*A);
        if direction>0 && b<rest,lower=0;upper=b;leave=b;
        elseif direction<0 && a>rest,lower=a;upper=0;leave=a;
        elseif direction>0,lower=0;upper=rest;
        else,lower=rest;upper=0;
        end
    end
    full=k~=0 && (upper-lower)*abs(k)>=2*pi;
    if isfinite(leave),exitSeconds=localFirstTime(V,A,leave);end
end

function t = localFirstTime(V,A,s)
    % Least t >= 0 with V t + A t^2/2 = s.
    if A==0,t=s/V;return;end
    r=roots([A/2,V,-s]);r=real(r(abs(imag(r))<=1e-9*max(1,abs(r)) & real(r)>=0));
    t=min([r(:);Inf]);
end

function [distance,sigma] = localTrackDistance(pose,shape,q,lower,upper,full)
    % Least vertex-to-edge distance between the fixed rectangle at 'pose' and
    % the target rectangle over arc lengths [lower, upper] of its track (the
    % whole circle when full), and the arc length attaining it. A straight
    % track translates the target: its vertices sweep segments against the
    % fixed edges, and the fixed vertices sweep the opposite segments in the
    % target's frame. A circular track rotates the target about the circle's
    % centre: the paths are arcs about that centre (localArcSegment).
    E=localCorners(pose,shape);B=localCorners(q(1:3),q(8:11));next=[2,3,4,1];
    if lower==upper
        distance=predictiveSafetyGeometry.rectangle(pose,shape,q(1:3),q(8:11));sigma=0;return;
    end
    distance=Inf;sigma=0;k=sin(q(6))/q(7);course=q(3)+q(6);
    if k==0
        u=predictiveSafetyGeometry.direction(course);
        for i=1:4
            for j=1:4
                [d,t]=localSegmentSegment(B(:,i)+lower*u,B(:,i)+upper*u,E(:,j),E(:,next(j)));
                if d<distance,distance=d;sigma=lower+t*(upper-lower);end
                [d,t]=localSegmentSegment(E(:,i)-upper*u,E(:,i)-lower*u,B(:,j),B(:,next(j)));
                if d<distance,distance=d;sigma=upper-t*(upper-lower);end
            end
        end
        return;
    end
    centre=q(1:2)+[-sin(course);cos(course)]/k;
    if full,span=[0,2*pi];else,span=sort(k*[lower,upper]);end
    for i=1:4
        b=B(:,i)-centre;e=E(:,i)-centre;
        angleB=atan2(b(2),b(1));angleE=atan2(e(2),e(1));
        for j=1:4
            [d,phi]=localArcSegment(centre,norm(b),angleB+span(1),angleB+span(2),E(:,j),E(:,next(j)));
            if d<distance,distance=d;sigma=(phi-angleB)/k;end
            [d,phi]=localArcSegment(centre,norm(e),angleE-span(2),angleE-span(1),B(:,j),B(:,next(j)));
            if d<distance,distance=d;sigma=(angleE-phi)/k;end
        end
    end
end

function q = localAlong(q,sigma)
    % The target state at arc length sigma along its track (pose only).
    k=sin(q(6))/q(7);half=k*sigma/2;scale=1;if half~=0,scale=sin(half)/half;end
    q(1:2)=q(1:2)+sigma*scale*predictiveSafetyGeometry.direction(q(3)+q(6)+half);
    q(3)=q(3)+k*sigma;
end

function corners = localCorners(pose,shape)
    rotation=[cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];
    corners=pose(1:2)+rotation*(shape(3:4)+shape(1:2).*[-1,1,1,-1;-1,-1,1,1]);
end

function [distance,t] = localPointSegment(p,a,b)
    d=b-a;length2=d.'*d;t=0;
    if length2>0,t=min(1,max(0,((p-a).'*d)/length2));end
    distance=norm(p-a-t*d);
end

function [distance,t] = localSegmentSegment(p1,p2,q1,q2)
    % Least distance between the segments [p1,p2] and [q1,q2], and the
    % parameter t on the first where it is attained.
    d1=p2-p1;d2=q2-q1;cross=d1(1)*d2(2)-d1(2)*d2(1);
    if abs(cross)>1e-14*max(1,norm(d1)*norm(d2))
        tu=[d1,-d2]\(q1-p1);
        if all(tu>=0 & tu<=1),distance=0;t=tu(1);return;end
    end
    [distance,~]=localPointSegment(p1,q1,q2);t=0;
    [d,~]=localPointSegment(p2,q1,q2);if d<distance,distance=d;t=1;end
    [d,s]=localPointSegment(q1,p1,p2);if d<distance,distance=d;t=s;end
    [d,s]=localPointSegment(q2,p1,p2);if d<distance,distance=d;t=s;end
end

function [distance,angle] = localArcSegment(centre,radius,first,last,a,b)
    % Least distance between the arc of 'radius' about 'centre' over the
    % angles [first, last] (last - first <= 2 pi) and the segment [a, b], and
    % the arc angle (in [first, last]) where it is attained. Candidates: the
    % arc's ends against the segment, the segment's ends against the arc (the
    % point-to-arc distance: | |p - centre| - radius | when the arc passes
    % the point's angle), and the segment's line against the circle (the
    % circle point nearest the line, or an intersection).
    span=last-first;full=span>=2*pi-1e-12;
    p=centre+radius*[cos(first);sin(first)];distance=localPointSegment(p,a,b);angle=first;
    p=centre+radius*[cos(last);sin(last)];d=localPointSegment(p,a,b);
    if d<distance,distance=d;angle=last;end
    if radius==0,return;end
    for p=[a,b]
        w=p-centre;[inside,phi]=localWithin(atan2(w(2),w(1)),first,span,full);
        if inside
            d=abs(norm(w)-radius);if d<distance,distance=d;angle=phi;end
        end
    end
    direction=b-a;length2=direction.'*direction;
    if length2==0,return;end
    t=((centre-a).'*direction)/length2;foot=a+t*direction;w=foot-centre;height=norm(w);
    if height>=radius
        if t>=0 && t<=1
            [inside,phi]=localWithin(atan2(w(2),w(1)),first,span,full);
            if inside && height-radius<distance,distance=height-radius;angle=phi;end
        end
    else
        offset=sqrt(radius^2-height^2)/sqrt(length2);
        for s=[t-offset,t+offset]
            if s>=0 && s<=1
                w=a+s*direction-centre;[inside,phi]=localWithin(atan2(w(2),w(1)),first,span,full);
                if inside,distance=0;angle=phi;return;end
            end
        end
    end
end

function [inside,angle] = localWithin(angle,first,span,full)
    % Is 'angle' on the arc [first, first + span]; the same angle unwrapped
    % into it.
    offset=mod(angle-first,2*pi);inside=full || offset<=span*(1+1e-12)+1e-12;
    angle=first+offset;
end
