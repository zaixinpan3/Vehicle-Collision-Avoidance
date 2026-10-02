classdef predictiveSafetyGeometry
    %predictiveSafetyGeometry Target flow, polygon duals and road coordinates.
    methods (Static)
        function guide = movingGaussianGuide(state,lane,epoch,startTime,duration,cfg)
            % Cheng (2021), Eqs. (34)-(36), with a time-aligned target envelope.
            % The arrival map s(t)=s0+vRef*t is only a search-reference clock.
            % Fit one Gaussian to the known moving exclusion envelope at the
            % same times. No division by closing speed or safety admission.
            projection=laneGeometry.project(state(1:2),lane);
            guide=struct('station',projection.station,'amplitude',0,'centerTime',0, ...
                'width',cfg.nominalClf.lookaheadSeconds,'endTime',0,'radius',0);
            if isempty(epoch),return;end
            radius=norm([cfg.vehicle.length;cfg.vehicle.width]/2)+norm(cfg.vehicle.rectangleOffset) ...
                +norm(epoch(8:9))+norm(epoch(10:11))+cfg.collision.safetyMarginMeters;
            guide.radius=radius;
            times=0:cfg.controller.sampleTime/2:duration;
            station=projection.station+cfg.referenceSpeed*times;
            position=zeros(2,numel(times));
            for index=1:numel(times)
                q=predictiveSafetyGeometry.targetFlow(epoch,startTime+times(index));
                rotation=[cos(q(3)),-sin(q(3));sin(q(3)),cos(q(3))];
                position(:,index)=q(1:2)+rotation*q(10:11);
            end
            target=laneGeometry.project(position,lane,station);
            longitudinal=station-target.station;lateral=target.lateralPosition;
            active=abs(longitudinal)<radius;
            if ~any(active & hypot(longitudinal,lateral)<radius),return;end
            [~,closest]=min(hypot(longitudinal,lateral));guide.centerTime=times(closest);
            first=find(active,1);last=find(active,1,'last');guide.endTime=times(last);
            guide.width=max(guide.width,(times(last)-times(first))/2);
            crossSection=sqrt(max(0,radius^2-longitudinal(active).^2));
            kernel=exp(-.5*((times(active)-guide.centerTime)/guide.width).^2);
            left=max([0,(lateral(active)+crossSection)./kernel]);
            right=max([0,(-lateral(active)+crossSection)./kernel]);
            % The smaller scalar envelope selects a side, not a second plan.
            % Symmetric encounters retain the left preference under roundoff.
            if left<=right+1e-10*max(1,max(left,right)),guide.amplitude=left;
            else,guide.amplitude=-right;
            end
        end

        function velocity = movingFlowVelocity(position,nominal,center,translation,map,mapRate,circulation)
            % Transport a unit-cylinder flow through a moving ellipse frame.
            % Outside q'*q >= 1, exact first-order following preserves that
            % exterior. This is guidance, not a bicycle safety certificate.
            % The interior regularization only keeps restoration seeds finite.
            q=map\(position-center);radiusSquared=max(1,q.'*q);
            relative=map\(nominal-translation-mapRate*q);
            modulation=(1+1/radiusSquared)*eye(2)-2*(q*q.')/radiusSquared^2;
            tangent=[-q(2);q(1)];
            velocity=translation+mapRate*q+map*(modulation*relative+circulation*tangent/radiusSquared);
        end

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
            % Sharma NRMM (2026), Eq. (17), in inertial coordinates:
            % Vdot=A, betadot=0, psidot=V*sin(beta)/lr. Constant curvature
            % makes the position integral elementary even when A is nonzero.
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
            kernel=localKernel();
            if isempty(kernel),[distance,normal]=predictiveSafetyGeometry.rectangleNumeric(poseE,shapeE,poseT,shapeT);
            else,[distance,normal]=kernel.rectangle(poseE(1:3),shapeE(:),poseT(1:3),shapeT(:));
            end
            if nargout>1
                re=[cos(poseE(3)),-sin(poseE(3));sin(poseE(3)),cos(poseE(3))];
                rt=[cos(poseT(3)),-sin(poseT(3));sin(poseT(3)),cos(poseT(3))];
                sE=-re.'*normal;sT=rt.'*normal;
                mu=[max(sE,0);max(-sE,0)];lambda=[max(sT,0);max(-sT,0)];
                hE=[shapeE(1:2)+shapeE(3:4);shapeE(1:2)-shapeE(3:4)];
                hT=[shapeT(1:2)+shapeT(3:4);shapeT(1:2)-shapeT(3:4)];
                certificate=struct('normal',normal,'mu',mu,'lambda',lambda, ...
                    'value',normal.'*(poseE(1:2)-poseT(1:2))-hE.'*mu-hT.'*lambda, ...
                    'normalConvention',"targetToEgo");
            end
        end

        function [distance,normal] = rectangleNumeric(poseE,shapeE,poseT,shapeT)
            % Distance and target-to-ego witness normal. This numeric part
            % is what scripts/buildControllerKernels.m compiles.
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
        end

        function rows = dualLinearization(poseE,shapeE,poseT,shapeT)
            % Li et al. (2023), Eqs. (12)-(13): optimize the ordinary
            % distance dual, then hold its multipliers fixed in the trajectory
            % rows. The minimum over S(x) is explicit for a rectangular ego.
            % No signed-distance objective or overlap direction is substituted.
            re=[cos(poseE(3)),-sin(poseE(3));sin(poseE(3)),cos(poseE(3))];
            rt=[cos(poseT(3)),-sin(poseT(3));sin(poseT(3)),cos(poseT(3))];
            body=shapeE(3:4)+shapeE(1:2).*[-1,1,1,-1;-1,-1,1,1];
            relative=poseE(1:2)-poseT(1:2)+re*body;
            targetA=[eye(2);-eye(2)]*rt.';
            targetB=[shapeT(1:2)+shapeT(3:4);shapeT(1:2)-shapeT(3:4)];
            % max alpha-b'*lambda, alpha <= lambda'*A*(v_j-pT),
            % lambda >= 0, ||A'*lambda||_2 <= 1. The hypograph makes
            % the finite-body minimum a five-variable convex distance dual.
            cone=secondordercone([targetA.',zeros(2,1)],zeros(2,1),zeros(5,1),-1);
            inequalities=[-relative.'*targetA.',ones(4,1)];
            persistent options
            if isempty(options)
                options=optimoptions('coneprog','Display','none', ...
                    'ConstraintTolerance',1e-10,'OptimalityTolerance',1e-10);
            end
            [point,~,flag]=coneprog([targetB;-1],cone,inequalities,zeros(4,1), ...
                [],[],[zeros(4,1);-Inf],[],options);
            if isempty(point) || any(~isfinite(point))
                error('collisionAvoidanceController:distanceDualFailed', ...
                    'The ordinary distance dual returned no finite point (exit flag %d).',flag);
            end
            lambda=point(1:4);normal=targetA.'*lambda;
            [values,jacobian]=predictiveSafetyGeometry.fixedDualRows(poseE,shapeE,poseT,shapeT,lambda);
            % coneprog can report a stalled dual residual at an optimum.
            % Check the actual dual against an independent primal distance;
            % this verifies the same multipliers without changing them.
            primalDistance=predictiveSafetyGeometry.rectangle(poseE,shapeE,poseT,shapeT);
            gap=primalDistance-min(values);
            if min(lambda)<-1e-8 || norm(normal)>1+1e-8 || abs(gap)>1e-6
                error('collisionAvoidanceController:distanceDualFailed', ...
                    'The ordinary distance dual failed its primal-dual check (exit flag %d, gap %.3g m).',flag,gap);
            end
            rows=struct('value',values,'jacobian',jacobian,'normal',normal, ...
                'lambda',lambda,'distance',min(values),'exitFlag',flag,'dualityGap',gap);
        end

        function [values,jacobian] = fixedDualRows(poseE,shapeE,poseT,shapeT,lambda)
            % Evaluate Eq. (13) with the supplied multipliers held fixed.
            % Four rows implement the minimum over the rectangular S(x).
            re=[cos(poseE(3)),-sin(poseE(3));sin(poseE(3)),cos(poseE(3))];
            rt=[cos(poseT(3)),-sin(poseT(3));sin(poseT(3)),cos(poseT(3))];
            body=shapeE(3:4)+shapeE(1:2).*[-1,1,1,-1;-1,-1,1,1];
            relative=poseE(1:2)-poseT(1:2)+re*body;
            targetA=[eye(2);-eye(2)]*rt.';
            targetB=[shapeT(1:2)+shapeT(3:4);shapeT(1:2)-shapeT(3:4)];
            normal=targetA.'*lambda;
            values=(normal.'*relative-targetB.'*lambda).';
            yaw=(normal.'*re*[0,-1;1,0]*body).';
            jacobian=[repmat(normal.',4,1),yaw];
        end

        function check = intervalClearance(first,last,shape,targetEpoch,time,duration,margin)
            % Certify positive distance on a linear ego-pose interpolant.
            % The target retains its exact prescribed flow. Adaptive midpoint
            % cuts resolve a Lipschitz lower bound down to half the sampled
            % margin. The other half supplies a finite refinement reserve.
            % This concerns the nominal interpolant, not the continuous plant.
            start=predictiveSafetyGeometry.targetFlow(targetEpoch,time);
            finish=predictiveSafetyGeometry.targetFlow(targetEpoch,time+duration);
            middle=predictiveSafetyGeometry.targetFlow(targetEpoch,time+duration/2);
            speed=max(abs([start(4),finish(4)]));curvature=abs(sin(start(6))/start(7));
            relative=(last(1:2)-first(1:2))/duration ...
                -middle(4)*predictiveSafetyGeometry.direction(middle(3)+middle(6));
            translation=norm(relative)+(abs(start(5))+speed^2*curvature)*duration/2;
            reach=norm(shape(1:2))+norm(shape(3:4));
            targetReach=norm(start(8:9))+norm(start(10:11));
            lipschitz=translation+abs(last(3)-first(3))/duration*reach+speed*curvature*targetReach;
            lipschitz=lipschitz+64*eps(max(1,lipschitz));
            left=predictiveSafetyGeometry.rectangle(first,shape,start(1:3),start(8:11));
            right=predictiveSafetyGeometry.rectangle(last,shape,finish(1:3),finish(8:11));
            check=struct('certified',false,'fractions',zeros(1,0), ...
                'minimumSampledClearance',min(left,right),'lowerBound',Inf,'lipschitz',lipschitz);
            if min(left,right)<margin || min(left,right)<=0,check.lowerBound=0;return;end
            pending=[0,1,left,right];
            while ~isempty(pending)
                row=pending(end,:);pending(end,:)=[];
                bound=min([row(3:4),(row(3)+row(4)-lipschitz*duration*(row(2)-row(1)))/2]);
                bound=bound-64*eps(max([1,row(3:4)]));
                if bound>margin/2,check.lowerBound=min(check.lowerBound,bound);continue;end
                fraction=(row(1)+row(2))/2;
                if fraction==row(1) || fraction==row(2),check.lowerBound=0;return;end
                pose=(1-fraction)*first+fraction*last;
                target=predictiveSafetyGeometry.targetFlow(targetEpoch,time+fraction*duration);
                value=predictiveSafetyGeometry.rectangle(pose,shape,target(1:3),target(8:11));
                check.fractions(end+1)=fraction; %#ok<AGROW>
                check.minimumSampledClearance=min(check.minimumSampledClearance,value);
                if value<margin || value<=0,check.lowerBound=0;return;end
                pending=[pending;row(1),fraction,row(3),value;fraction,row(2),value,row(4)]; %#ok<AGROW>
            end
            check.certified=true;
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

function kernel=localKernel()
    % Use only the distance kernel. Collision duals are optimization
    % problems and never call the retired signed-direction MEX kernel.
    persistent handles checked
    if isempty(checked)
        checked=true;handles=[];
        native=fullfile(fileparts(fileparts(mfilename('fullpath'))),'solver','controller');
        if isfile(fullfile(native,['rectangleKernelMex.',mexext]))
            previous=addpath(native);candidate=struct('rectangle',@rectangleKernelMex);
            shape=[2.4;.95;.3;-.1];same=true;
            for poseT=[[9;4;2.1],[1.5;.8;.6],[-30;12;-1.3]]
                poseE=[.4;-.3;.25];
                [d,n]=predictiveSafetyGeometry.rectangleNumeric(poseE,shape,poseT,shape);
                [dk,nk]=candidate.rectangle(poseE,shape,poseT,shape);
                same=same && isequal({d,n},{dk,nk});
            end
            path(previous);
            if same,handles=candidate;
            else
                warning('collisionAvoidanceController:staleGeometryKernel', ...
                    'The compiled geometry kernels differ from the source and are not used. Rebuild them with buildControllerKernels.');
            end
        end
    end
    kernel=handles;
end

function [distance,point]=localPointSegment(vertex,first,last)
    edge=last-first;alpha=min(1,max(0,(vertex-first).'*edge/(edge.'*edge)));
    point=first+alpha*edge;distance=norm(vertex-point);
end
