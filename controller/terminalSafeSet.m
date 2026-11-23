classdef terminalSafeSet
%terminalSafeSet Moving range-exit continuation and shared CLF and geometry utilities.
    methods (Static)
        function input=movingControl(cfg)
            gain=nonlinearBicycleModel.tireAccelerationGain(cfg);
            trim=nonlinearBicycleModel.roadLoad(cfg.referenceSpeed,cfg)/(cfg.vehicle.m*gain);
            input=[0;min(cfg.actuation.brakingRatioMaximum,max(cfg.actuation.brakingRatioMinimum,trim))];
        end

        function [value,normal,gradient,exitSteps]=movingValue(x,q,cfg)
            % Constant-velocity handoff for one known target.
            R=cfg.collision.encounterRangeMeters;
            value=-R;normal=zeros(2,1);gradient=zeros(1,6);exitSteps=0;
            if isempty(q) || norm(q(1:2)-x(1:2))>R,return;end
            % Relative angles retain equivalence under full revolutions.
            direction=predictiveSafetyGeometry.direction(x(3));
            basis=[direction,[-direction(2);direction(1)]];
            targetVelocity=basis*(q(4)*predictiveSafetyGeometry.direction(rem(q(3)-x(3)+q(6),2*pi)));
            [value,normal,gradient,exitSteps]=terminalSafeSet.separatingMotion(x,q,targetVelocity,0,0,cfg);
            if isfinite(exitSteps)
                % Exact first outside node on the frozen relative line.
                offset=q(1:2)-x(1:2);relative=targetVelocity-x(4)*direction;
                speed=norm(relative);along=offset.'*(relative/speed);
                residual=(R-norm(offset))*(R+norm(offset));
                root=sqrt(along^2+residual);travel=root-along;
                if along>0,travel=residual/(root+along);end
                exitSteps=floor(travel/(speed*cfg.controller.sampleTime))+1;
            end
        end

        function [value,normal,gradient,exitSteps]=separatingMotion(x,boxes,velocity,radius,positionRadius,cfg)
            % For each occupied box choose a supporting plane whose gap can
            % never decrease under the handoff velocity disk.
            h=cfg.controller.sampleTime;R=cfg.collision.encounterRangeMeters;
            shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
            e=predictiveSafetyGeometry.direction(x(3));J=[0,-1;1,0];
            w=x(4)*e-velocity;speed=vecnorm(w);
            normal=zeros(2,1);gradient=zeros(1,6);exitSteps=Inf;
            drift=speed-radius;
            if x(4)<=0
                value=max(2*cfg.outputFeedback.nlpTolerance,-x(4));gradient(4)=-1;return
            end
            [slow,k]=min(drift);
            if slow<=0
                value=2*cfg.outputFeedback.nlpTolerance-h*slow;
                if speed(k)>0
                    gradient(3:4)=-h*w(:,k).'/speed(k)*[x(4)*J*e,e];
                else
                    gradient(4)=-h;
                end
                return
            end
            pose=repmat(x(1:3),1,size(boxes,2));
            [gap,g]=predictiveSafetyGeometry.rectangleClearance(pose,boxes,shape);
            % A penetrating handoff is infeasible for every separating plane.
            % Its rectangle dual still supplies a local improvement direction.
            adjust=gap>0 & sum(g(1:2,:).*w,1)<radius;
            gradients=[g;zeros(1,size(g,2))];normals=g(1:2,:);
            if any(adjust)
                axis=w(:,adjust)./speed(adjust);
                cosine=radius(adjust)./speed(adjust);
                sine=sqrt((drift(adjust)./speed(adjust)).*(1+cosine));
                right=axis.*cosine-J*axis.*sine;left=axis.*cosine+J*axis.*sine;
                target=boxes(:,adjust);
                c=cos(target(3,:));s=sin(target(3,:));
                lateral=J*e;
                targetCenter=target(1:2,:)+[c.*target(10,:)-s.*target(11,:);s.*target(10,:)+c.*target(11,:)];
                delta=x(1:2)+[e,lateral]*shape(3:4)-targetCenter;
                [first,firstGradient]=support(right,-1);
                [second,secondGradient]=support(left,1);
                use=second>first;first(use)=second(use);
                firstGradient(:,use)=secondGradient(:,use);right(:,use)=left(:,use);
                gap(adjust)=first;gradients(:,adjust)=firstGradient;normals(:,adjust)=right;
            end
            [least,k]=min(gap);value=cfg.outputFeedback.clearanceMeters-least;
            gradient(1:4)=-gradients(:,k).';normal=normals(:,k);
            % A finite upper bound on the exit index, without time marching.
            offset=x(1:2)-boxes(1:2,:);
            exitSteps=1+floor(max((R+vecnorm(offset)+positionRadius)./(h*drift)));

            function [projection,derivative]=support(n,side)
                ne=e.'*n;nl=lateral.'*n;
                nt=c.*n(1,:)+s.*n(2,:);nb=-s.*n(1,:)+c.*n(2,:);
                projection=sum(n.*delta,1)-shape(1)*abs(ne)-shape(2)*abs(nl) ...
                    -target(8,:).*abs(nt)-target(9,:).*abs(nb);
                egoArm=e.*(shape(3)-shape(1)*sign(ne))+lateral.*(shape(4)-shape(2)*sign(nl));
                tLong=target(8,:).*sign(nt);tLat=target(9,:).*sign(nb);
                witness=x(1:2)+egoArm-targetCenter-[c.*tLong-s.*tLat;s.*tLong+c.*tLat];
                % Angle of n is angle(w) +/- acos(radius/|w|).
                phi=(J*axis+side*axis.*(cosine./sine))./speed(adjust);
                nTerm=sum((J*n).*witness,1);
                derivative=[n;sum(n.*(J*egoArm),1)+nTerm.*(x(4)*lateral.'*phi);nTerm.*(e.'*phi)];
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
            if isinf(upper) && sin(q(6))==0
                direction=predictiveSafetyGeometry.direction(q(3)+q(6));
                ego=localCorners(pose,context.shape);body=localCorners(q(1:3),q(8:11));
                upper=max(0,max(direction.'*ego)-min(direction.'*body));
            end
            [distance,sigma]=localTrackDistance(pose,context.shape,q,lower,upper,full);
            witness=localAlong(q,sigma);
            track=struct('lower',lower,'upper',upper,'fullCircle',full,'exitSeconds',exitSeconds,'witnessArcLength',sigma);
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
            % the gain certified offline together with P (CONTROLLER_THEORY.md, Section 4). On
            % the certified level set the hold satisfies V(next) <= rho V(x)
            % and the input bound. ok evaluates the decrease and the hard rows
            % on this hold. The anchor appends it while its endpoint stays in
            % the terminal set.
            cfg=model.cfg;reference=model.nominalReference;lane=model.lane;
            e=nonlinearBicycleModel.error(x,lane,reference);
            input=reference.input+reference.gain*e;
            [lower,upper]=nonlinearBicycleModel.inputBounds(cfg,true);low=lower(2);high=upper(2);
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

        function level = stateLevel(reference,cfg)
            % Largest c with {e : e'Pe <= c} inside the linear state rows g'e <= b:
            % c = min b^2/(g' inv(P) g). Part of the cruise controller's certificate.
            x=reference.state;tire=nonlinearBicycleModel.tireParameters(cfg);
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
            % (rayJacobians) over the certified set (CONTROLLER_THEORY.md, Section 4).
            if nargin<5,duration=cfg.controller.sampleTime;end
            curve=struct('origin',[0;0],'heading',0,'curvature',reference.curvature,'length',200);
            lane=struct('referenceCurve',curve);
            [~,heading]=predictiveSafetyGeometry.referencePose(50,0,curve);
            chart=[[-sin(heading);cos(heading)],zeros(2,4);zeros(4,1),eye(4)];
            n=size(errors,2);a=nan(5,5,n);b=nan(5,2,n);plus=nan(5,n);
            for j=1:n
                e=errors(:,j);
                [position,yaw]=predictiveSafetyGeometry.referencePose(50,e(1),curve);
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
            % encloses them (CONTROLLER_THEORY.md, Section 4). NaN where the model's domain is
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
            % of the Jacobians (CONTROLLER_THEORY.md, Section 4); these samples test it
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
                [position,heading]=predictiveSafetyGeometry.referencePose(50,e(1),curve);
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
            % State limits of each column of x.
            low=[cfg.model.speedMinimum;-cfg.model.lateralVelocityMaximum;-cfg.model.yawRateMaximum];
            high=[cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum];
            ok=all(x(4:6,:)>=low-1e-9 & x(4:6,:)<=high+1e-9,1);
        end

        function ok = handlingRows(x,u,cfg)
            % Shared observer sideslip domain and physical input magnitude.
            % Fiala dynamics already account for saturation at either axle.
            ok=abs(u(2,:))<=1+1e-9 ...
                & (x(4,:)<=cfg.model.brakeHoldSpeed | abs(x(5,:))<=tan(cfg.model.sideslipMaximum)*x(4,:)*(1+1e-9));
        end

        function margin = roadMargin(x,model)
            % Smallest distance of an ego corner inside the road (negative
            % outside), for each column of x, in one projection.
            margin=Inf(1,size(x,2));
            if isempty(model.road.lateralClearance),return;end
            cfg=model.cfg;clearance=model.road.lateralClearance;
            corners=cfg.vehicle.rectangleOffset+[cfg.vehicle.length;cfg.vehicle.width]/2.*[1,1,-1,-1;1,-1,1,-1];
            n=size(x,2);c=cos(x(3,:));s=sin(x(3,:));points=zeros(2,4,n);
            for corner=1:4
                points(:,corner,:)=reshape(x(1:2,:)+[c*corners(1,corner)-s*corners(2,corner); ...
                    s*corners(1,corner)+c*corners(2,corner)],2,1,n);
            end
            projection=predictiveSafetyGeometry.project(reshape(points,2,[]),model.lane);
            lateral=reshape(projection.lateralPosition,4,n);
            margin=min([clearance(2)-lateral;clearance(1)+lateral],[],1);
        end
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
    % The physical stop can lie between checked samples. Explicitly reject
    % initial overlap, including containment with no boundary crossing.
    distance=predictiveSafetyGeometry.rectangle(pose,shape,q(1:3),q(8:11));sigma=0;
    if distance<=0 || lower==upper,return;end
    E=localCorners(pose,shape);B=localCorners(q(1:3),q(8:11));next=[2,3,4,1];
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
