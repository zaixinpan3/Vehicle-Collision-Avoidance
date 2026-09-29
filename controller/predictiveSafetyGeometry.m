classdef predictiveSafetyGeometry
    %predictiveSafetyGeometry Target flow, polygon duals and road coordinates.
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
            tangent=predictiveSafetyGeometry.direction(observation.yaw+beta);
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
            next=q;next(1:2)=q(1:2)+arc*scale*predictiveSafetyGeometry.direction(q(3)+q(6)+a);
            next(3)=q(3)+curvature*arc;next(4)=q(4)+q(5)*time;
        end

        function tangent = direction(angle)
            % Exact cardinal directions avoid a fictitious transverse drift
            % over an infinite horizon. Nearby physical angles remain nonzero.
            tangent=[cospi(angle/pi);sinpi(angle/pi)];
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

        function frame = roadFrame(lane,~)
            if isfield(lane,'referenceCurve')
                curve=lane.referenceCurve;origin=curve.origin;heading=curve.heading;k=curve.curvature;
            else
                if any(abs(lane.segmentCurvature)>1e-12)
                    error('collisionAvoidanceController:unsupportedReference','Polyline corners cannot define a smooth terminal lane reference.');
                end
                origin=lane.segmentStart(1,:).';heading=atan2(lane.tangent(1,2),lane.tangent(1,1));k=0;
            end
            % Only the given path enters control; optional road widths are metadata.
            frame=[origin;heading;k];
        end

    end
end

function [distance,point]=localPointSegment(vertex,first,last)
    edge=last-first;alpha=min(1,max(0,(vertex-first).'*edge/(edge.'*edge)));
    point=first+alpha*edge;distance=norm(vertex-point);
end
