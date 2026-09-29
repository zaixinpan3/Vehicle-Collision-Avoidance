classdef predictiveSafetyGeometry
    %predictiveSafetyGeometry Target flow, polygon duals and nominal MPC terminal geometry.
    methods (Static)
        function q = target(observation,raw,~)
            % q = [X;Y;psi;V;A;beta;lr;halfLength;halfWidth;offsetX;offsetY].
            % V is signed tangential velocity; A and beta remain constant.
            q=zeros(11,0);if isempty(observation),return;end
            headingKnown=false;
            for name=["targetYawInertial","targetHeadingInertial","targetYawRelative", ...
                    "targetYawAngle","targetYaw","yawAngle","yaw","heading","relativeYaw","relativeHeading"]
                headingKnown=headingKnown || (isfield(raw,name) && ~isempty(raw.(name)));
            end
            if ~headingKnown
                error('collisionAvoidanceController:missingBodyHeading', ...
                    'A rotating target rectangle requires an explicit body heading, separate from its velocity course.');
            end
            speed=norm(observation.velocity);beta=0;
            if speed>0,beta=atan2(sin(atan2(observation.velocity(2),observation.velocity(1))-observation.yaw), ...
                    cos(atan2(observation.velocity(2),observation.velocity(1))-observation.yaw));end
            if abs(beta)>pi/2,beta=beta-sign(beta)*pi;end
            offset=zeros(2,1);
            if isfield(raw,'targetRectangleOffset'),offset=raw.targetRectangleOffset;end
            validateattributes(offset,{'double'},{'size',[2,1],'real','finite'});
            if isfield(raw,'targetSideslip') && ~isempty(raw.targetSideslip)
                beta=raw.targetSideslip;
            end
            validateattributes(beta,{'double'},{'scalar','real','finite','>',-pi/2,'<',pi/2});
            tangent=[cos(observation.yaw+beta);sin(observation.yaw+beta)];
            speed=tangent.'*observation.velocity;
            if norm(observation.velocity-speed*tangent)>1e-10*max(1,abs(speed))
                error('collisionAvoidanceController:inconsistentTargetMotion','Velocity direction and supplied sideslip disagree.');
            end
            acceleration=observation.tangentialAcceleration;
            if isempty(acceleration),acceleration=tangent.'*observation.acceleration;end
            q=[observation.position;observation.yaw;speed;acceleration;beta;observation.rearAxleDistance; ...
                observation.length/2;observation.width/2;offset];
        end

        function next = targetFlow(q,time)
            if isempty(q),next=q;return;end
            arc=q(4)*time+.5*q(5)*time^2;
            curvature=sin(q(6))/q(7);a=curvature*arc/2;scale=1;
            if a~=0,scale=sin(a)/a;end
            next=q;next(1:2)=q(1:2)+arc*scale*[cos(q(3)+q(6)+a);sin(q(3)+q(6)+a)];
            next(3)=q(3)+curvature*arc;next(4)=q(4)+q(5)*time;
        end

        function [distance,certificate] = rectangle(poseE,shapeE,poseT,shapeT)
            re=[cos(poseE(3)),-sin(poseE(3));sin(poseE(3)),cos(poseE(3))];
            rt=[cos(poseT(3)),-sin(poseT(3));sin(poseT(3)),cos(poseT(3))];
            signs=[-1,1,1,-1;-1,-1,1,1];
            e=poseE(1:2)+re*(shapeE(3:4)+shapeE(1:2).*signs);
            t=poseT(1:2)+rt*(shapeT(3:4)+shapeT(1:2).*signs);
            axes=[re,rt];separated=false;
            for j=1:4
                pe=axes(:,j).'*e;pt=axes(:,j).'*t;
                separated=separated || min(pe)>max(pt) || min(pt)>max(pe);
            end
            distance=0;normal=zeros(2,1);
            if separated
                distance=Inf;
                for j=1:4
                    for k=1:4
                        [d,p]=localPointSegment(e(:,j),t(:,k),t(:,mod(k,4)+1));
                        if d<distance,distance=d;normal=(e(:,j)-p)/d;end
                        [d,p]=localPointSegment(t(:,j),e(:,k),e(:,mod(k,4)+1));
                        if d<distance,distance=d;normal=(p-t(:,j))/d;end
                    end
                end
            end
            sE=-re.'*normal;sT=rt.'*normal;
            mu=[max(sE,0);max(-sE,0)];lambda=[max(sT,0);max(-sT,0)];
            hE=[shapeE(1:2)+shapeE(3:4);shapeE(1:2)-shapeE(3:4)];
            hT=[shapeT(1:2)+shapeT(3:4);shapeT(1:2)-shapeT(3:4)];
            certificate=struct('normal',normal,'mu',mu,'lambda',lambda, ...
                'value',normal.'*(poseE(1:2)-poseT(1:2))-hE.'*mu-hT.'*lambda, ...
                'normalConvention',"targetToEgo");
        end

        function rows = dualLinearization(poseE,shapeE,poseT,shapeT,preferred)
            % Li-style fixed-normal distance dual, extended to both bodies.
            % The four ego vertex rows retain yaw dependence in each SCA step.
            % At overlap, a signed support certificate supplies a nonzero
            % restoration direction; it never certifies positive clearance.
            re=[cos(poseE(3)),-sin(poseE(3));sin(poseE(3)),cos(poseE(3))];
            rt=[cos(poseT(3)),-sin(poseT(3));sin(poseT(3)),cos(poseT(3))];
            body=shapeE(3:4)+shapeE(1:2).*[-1,1,1,-1;-1,-1,1,1];
            ego=poseE(1:2)+re*body;
            target=poseT(1:2)+rt*(shapeT(3:4)+shapeT(1:2).*[-1,1,1,-1;-1,-1,1,1]);
            [distance,witness]=predictiveSafetyGeometry.rectangle(poseE,shapeE,poseT,shapeT);
            normal=witness.normal;
            if distance==0
                directions=[preferred/norm(preferred),-preferred/norm(preferred),re,-re,rt,-rt];
                gaps=min(directions.'*ego,[],2)-max(directions.'*target,[],2);
                best=max(gaps);index=find(gaps>=best-1e-10,1);normal=directions(:,index);
            end
            se=-re.'*normal;st=rt.'*normal;
            mu=[max(se,0);max(-se,0)];lambda=[max(st,0);max(-st,0)];
            ht=[shapeT(1:2)+shapeT(3:4);shapeT(1:2)-shapeT(3:4)];
            targetSupport=normal.'*poseT(1:2)+ht.'*lambda;
            values=(normal.'*ego-targetSupport).';
            yaw=(normal.'*re*[0,-1;1,0]*body).';
            rows=struct('value',values,'jacobian',[repmat(normal.',4,1),yaw], ...
                'normal',normal,'mu',mu,'lambda',lambda,'signedDistance',min(values));
        end

        function frame = roadFrame(lane,road)
            if isfield(lane,'referenceCurve')
                curve=lane.referenceCurve;origin=curve.origin;heading=curve.heading;k=curve.curvature;
            else
                if any(abs(lane.segmentCurvature)>1e-12)
                    error('collisionAvoidanceController:unsupportedReference','Polyline corners cannot define a smooth terminal lane reference.');
                end
                origin=lane.segmentStart(1,:).';heading=atan2(lane.tangent(1,2),lane.tangent(1,1));k=0;
            end
            clearance=road.lateralClearance;
            if isempty(clearance),clearance=[1e100;1e100];end
            frame=[origin;heading;k;clearance];
        end

        function terminal = terminalSet(cfg,frame)
            % LQR ellipsoid and analytic actuator/state limits. The nonlinear
            % local invariance assumption is stated in PCBF_CLF_ARCHITECTURE.md.
            reference=nonlinearBicycleModel.cruise(cfg,frame(4));
            unitError=vecnorm(reference.factor\eye(5),2,2);
            unitInput=vecnorm(reference.gain/reference.factor,2,2);
            lower=[-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum];
            upper=[cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum];
            rate=[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;
            available=min(reference.input-lower,upper-reference.input);
            stateRoom=[Inf;pi/4;min(reference.state(4)-cfg.model.scheduleSpeedFloor, ...
                cfg.model.speedMaximum-reference.state(4)); ...
                cfg.model.lateralVelocityMaximum-abs(reference.state(5)); ...
                cfg.model.yawRateMaximum-abs(reference.state(6))];
            radius=min([cfg.nonlinear.terminalRadius;available./max(unitInput,eps); ...
                rate./max(2*unitInput,eps);stateRoom./max(unitError,eps)]);
            if radius<=0 || ~isfinite(radius)
                error('collisionAvoidanceController:invalidTerminalSet','The lane trim must lie inside the model and input limits.');
            end
            terminal=struct('reference',reference,'radius',radius, ...
                'errorBound',radius*unitError,'inputRadius',radius*unitInput);
        end

        function margin = terminalMargin(x,q,frame,terminal,shape,clearance)
            % Closed-form separation of the terminal lane tube from the
            % target ray/orbit. Evaluated directly as an MPC terminal row.
            t=[cos(frame(3));sin(frame(3))];n=[-t(2);t(1)];origin=frame(1:2);
            bound=terminal.errorBound;ref=terminal.reference;
            body=shape(3:4)+shape(1:2).*[-1,1,1,-1;-1,-1,1,1];
            rotation=[cos(ref.state(3)),-sin(ref.state(3));sin(ref.state(3)),cos(ref.state(3))];
            corners=rotation*body;reach=max(vecnorm(body));
            lateral=[min(corners(2,:))-bound(1)-reach*bound(2); ...
                max(corners(2,:))+bound(1)+reach*bound(2)];
            road=min(lateral(1)+frame(5),frame(6)-lateral(2));
            if frame(4)~=0
                radius=abs(1/frame(4));center=origin+n/frame(4);width=bound(1)+reach;
                road=min(frame(5:6))-width;
                if isempty(q),margin=road;return;end
                if q(6)~=0 && any(q(4:5)~=0)
                    [targetCenter,inner,outer]=localOrbit(q);d=norm(targetCenter-center);
                    minimum=max([0,d-outer,inner-d]);maximum=d+outer;
                else
                    d=q(1:2)-center;direction=[cos(q(3)+q(6));sin(q(3)+q(6))];
                    [first,last]=localProgressRange(q(4),q(5));
                    arc=min(last,max(first,-d.'*direction));targetReach=norm(q(8:9)+abs(q(10:11)));
                    minimum=norm(d+arc*direction)-targetReach;maximum=norm(d)+targetReach;
                    if isinf(first) || isinf(last),maximum=Inf;end
                end
                collision=max(minimum-radius-width,radius-width-maximum)-clearance;
                margin=min(road,collision);return;
            end
            if isempty(q),margin=road;return;end
            angle=abs(ref.state(3))+bound(2);
            progress=(ref.state(4)-bound(3))*cos(angle)-(abs(ref.state(5))+bound(4))*sin(angle);
            if q(6)~=0 && any(q(4:5)~=0)
                [center,~,outer]=localOrbit(q);
                targetLateral=n.'*(center-origin)+[-outer;outer];
                targetForward=t.'*center+outer;
                advance=0;if progress<0,advance=Inf;end
            else
                r=[cos(q(3)),-sin(q(3));sin(q(3)),cos(q(3))];
                vertices=q(1:2)+r*(q(10:11)+q(8:9).*[-1,1,1,-1;-1,-1,1,1]);
                direction=[cos(q(3)+q(6));sin(q(3)+q(6))];
                lateralDirection=n.'*direction;forwardDirection=t.'*direction;
                if abs(lateralDirection)<1e-12,lateralDirection=0;end
                if abs(forwardDirection)<1e-12,forwardDirection=0;end
                [lower,upper]=localProgressRange(q(4)*lateralDirection,q(5)*lateralDirection);
                side=n.'*(vertices-origin);targetLateral=[min(side)+lower;max(side)+upper];
                targetForward=max(t.'*vertices);
                [~,advance]=localProgressRange(q(4)*forwardDirection-progress,q(5)*forwardDirection);
            end
            collision=max(lateral(1)-targetLateral(2),targetLateral(1)-lateral(2))-clearance;
            rear=min(corners(1,:))-reach*bound(2);
            collision=max(collision,t.'*x(1:2)+rear-targetForward-advance-clearance);
            margin=min(road,collision);
        end
    end
end

function [center,inner,outer]=localOrbit(q)
    radius=q(7)/sin(q(6));angle=q(3)+q(6);
    center=q(1:2)+radius*[-sin(angle);cos(angle)];
    offset=[-radius*sin(q(6));radius*cos(q(6))]-q(10:11);
    inner=norm(max(0,abs(offset)-q(8:9)));outer=norm(abs(offset)+q(8:9));
end

function [lower,upper]=localProgressRange(velocity,acceleration)
    % Range of velocity*t + acceleration*t^2/2 for every t >= 0.
    lower=0;upper=0;
    if acceleration>0
        lower=-min(velocity,0)^2/(2*acceleration);upper=Inf;
    elseif acceleration<0
        lower=-Inf;upper=-max(velocity,0)^2/(2*acceleration);
    elseif velocity>0
        upper=Inf;
    elseif velocity<0
        lower=-Inf;
    end
end

function [distance,point]=localPointSegment(vertex,first,last)
    edge=last-first;alpha=min(1,max(0,(vertex-first).'*edge/(edge.'*edge)));
    point=first+alpha*edge;distance=norm(vertex-point);
end
