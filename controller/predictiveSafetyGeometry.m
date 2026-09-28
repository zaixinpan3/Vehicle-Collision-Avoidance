classdef predictiveSafetyGeometry
    %predictiveSafetyGeometry Target flow, polygon duals and nominal MPC terminal geometry.
    methods (Static)
        function q = target(observation,raw,~)
            q=zeros(10,0);if isempty(observation),return;end
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
            offset=zeros(2,1);
            if isfield(raw,'targetRectangleOffset'),offset=raw.targetRectangleOffset;end
            validateattributes(offset,{'double'},{'size',[2,1],'real','finite'});
            if isfield(raw,'targetSideslip')
                validateattributes(raw.targetSideslip,{'double'},{'scalar','real','finite'});
                if speed>0 && abs(atan2(sin(beta-raw.targetSideslip),cos(beta-raw.targetSideslip)))>1e-10
                    error('collisionAvoidanceController:inconsistentTargetMotion','Velocity course and supplied sideslip disagree.');
                end
                beta=raw.targetSideslip;
            end
            % Tangential speed and body heading rate are independent constants.
            % A known constant course/body offset is retained when supplied.
            omega=observation.yawRate;
            q=[observation.position;observation.yaw;speed;beta;omega;observation.length/2;observation.width/2;offset];
        end

        function next = targetFlow(q,time)
            if isempty(q),next=q;return;end
            a=q(6)*time/2;scale=1;
            if a~=0,scale=sin(a)/a;end
            next=q;next(1:2)=q(1:2)+q(4)*time*scale*[cos(q(3)+q(5)+a);sin(q(3)+q(5)+a)];
            next(3)=q(3)+q(6)*time;
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
            if laneGeometry.isVaryingReference(lane)
                error('collisionAvoidanceController:unsupportedReference', ...
                    'The nominal terminal controller supports a straight or constant-curvature reference.');
            end
            if ~isempty(road.boundaries)
                error('collisionAvoidanceController:unsupportedRoad', ...
                    'Use a global lateralClearance corridor for the nominal terminal set.');
            end
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
            clearance=clearance-road.lateralClearanceErrorBound;
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
                if q(6)~=0
                    [targetCenter,inner,outer]=localOrbit(q);d=norm(targetCenter-center);
                    minimum=max([0,d-outer,inner-d]);maximum=d+outer;
                else
                    d=q(1:2)-center;v=q(4)*[cos(q(3)+q(5));sin(q(3)+q(5))];
                    tau=max(0,-d.'*v/max(q(4)^2,eps));targetReach=norm(q(7:8)+abs(q(9:10)));
                    minimum=norm(d+tau*v)-targetReach;maximum=norm(d)+targetReach;
                    if q(4)>0,maximum=Inf;end
                end
                collision=max(minimum-radius-width,radius-width-maximum)-clearance;
                margin=min(road,collision);return;
            end
            if isempty(q),margin=road;return;end
            velocity=q(4)*[cos(q(3)+q(5));sin(q(3)+q(5))];
            if q(6)~=0
                [center,~,outer]=localOrbit(q);
                targetLateral=n.'*(center-origin)+[-outer;outer];
                targetForward=t.'*center+outer;
                targetProgress=0;
            else
                r=[cos(q(3)),-sin(q(3));sin(q(3)),cos(q(3))];
                vertices=q(1:2)+r*(q(9:10)+q(7:8).*[-1,1,1,-1;-1,-1,1,1]);
                side=n.'*(vertices-origin);targetLateral=[min(side);max(side)];
                sideSpeed=n.'*velocity;
                if sideSpeed < -1e-12,targetLateral(1)=-Inf;end
                if sideSpeed > 1e-12,targetLateral(2)=Inf;end
                targetForward=max(t.'*vertices);targetProgress=t.'*velocity;
            end
            collision=max(lateral(1)-targetLateral(2),targetLateral(1)-lateral(2))-clearance;
            angle=abs(ref.state(3))+bound(2);
            progress=(ref.state(4)-bound(3))*cos(angle)-(abs(ref.state(5))+bound(4))*sin(angle);
            if progress>=targetProgress
                rear=min(corners(1,:))-reach*bound(2);
                collision=max(collision,t.'*x(1:2)+rear-targetForward-clearance);
            end
            margin=min(road,collision);
        end
    end
end

function [center,inner,outer]=localOrbit(q)
    radius=q(4)/q(6);angle=q(3)+q(5);
    center=q(1:2)+radius*[-sin(angle);cos(angle)];
    offset=[-radius*sin(q(5));radius*cos(q(5))]-q(9:10);
    inner=norm(max(0,abs(offset)-q(7:8)));outer=norm(abs(offset)+q(7:8));
end

function [distance,point]=localPointSegment(vertex,first,last)
    edge=last-first;alpha=min(1,max(0,(vertex-first).'*edge/(edge.'*edge)));
    point=first+alpha*edge;distance=norm(vertex-point);
end
