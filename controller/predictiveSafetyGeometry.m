classdef predictiveSafetyGeometry
    %predictiveSafetyGeometry Target prediction, uncertainty tubes and polygon geometry.
    methods (Static)
        function uncertainty = observerUncertainty(ego,observation,q)
            % A common uncertain rigid pose cancels from pairwise distance.
            % Relative target balls are therefore used in the frame frozen at
            % this sample; path and CLF rows still use the full ego error box.
            uncertainty=struct('specified',ego.uncertaintySpecified, ...
                'egoGenerator',ego.errorGenerator,'collisionGenerator',ego.errorGenerator, ...
                'target',[],'relativeFrame',false);
            if isempty(observation),return;end
            set=observation.predictionErrorSet;
            if isempty(set),return;end
            required={'kind','time','available','referenceEgoPose','relativePosition', ...
                'positionRadius','courseCenter','courseRadius','speedInterval', ...
                'accelerationInterval','curvatureInterval','rearAxleDistance'};
            if ~isstruct(set) || ~isscalar(set) || ~all(isfield(set,required)) ...
                    || ~(isequal(set.kind,"nrmm-constant-parameter-set-v1") ...
                    || isequal(set.kind,'nrmm-constant-parameter-set-v1'))
                return;
            end
            if ~isequal(set.available,true)
                return;
            end
            if ~ego.uncertaintySpecified || ~observation.uncertaintySpecified
                return;
            end
            fields={'time','referenceEgoPose','relativePosition','positionRadius','courseCenter', ...
                'courseRadius','speedInterval','accelerationInterval','curvatureInterval','rearAxleDistance'};
            sizes=[1,3,2,1,1,1,2,2,2,1];
            for index=1:numel(fields)
                value=set.(fields{index});
                if ~isnumeric(value) || ~isreal(value) || numel(value)~=sizes(index) || any(~isfinite(value),'all')
                    return;
                end
                set.(fields{index})=value(:);
            end
            intervals=[set.speedInterval,set.accelerationInterval,set.curvatureInterval];
            if any(intervals(1,:)>intervals(2,:)) || min([set.positionRadius,set.courseRadius])<0 ...
                    || set.rearAxleDistance<=0 || any(abs(set.curvatureInterval*set.rearAxleDistance)>1)
                return;
            end
            rotation=[cos(ego.yaw),-sin(ego.yaw);sin(ego.yaw),cos(ego.yaw)];
            residual=[set.referenceEgoPose-ego.modelState(1:3); ...
                ego.position+rotation*set.relativePosition-q(1:2); ...
                atan2(sin(ego.yaw+set.courseCenter-q(3)-q(6)),cos(ego.yaw+set.courseCenter-q(3)-q(6))); ...
                set.rearAxleDistance-q(7)];
            residual(3)=atan2(sin(residual(3)),cos(residual(3)));
            if abs(set.time-ego.stateTime)>1e-9*max(1,abs(ego.stateTime)) || norm(residual,inf)>1e-8
                return;
            end
            curvature=sin(q(6))/q(7);
            uncertainty.target=struct('positionRadius',set.positionRadius,'courseRadius',min(pi,set.courseRadius), ...
                'speedRadius',max(abs(set.speedInterval-q(4))), ...
                'accelerationRadius',max(abs(set.accelerationInterval-q(5))), ...
                'curvatureRadius',max(abs(set.curvatureInterval-curvature)), ...
                'sideslipRadius',max(abs(asin(set.curvatureInterval*q(7))-q(6))));
            uncertainty.collisionGenerator(1:3,:)=0;
            uncertainty.relativeFrame=true;uncertainty.specified=true;
        end

        function tube = targetErrorTube(q,errorSet,time)
            % Exact analytic enclosure of constant-A, constant-beta paths.
            % Split the arc-length endpoint error from the course/curvature
            % error on the nominal arc. No future observer decay is assumed.
            % Braking that ends at rest keeps the arc-length error bound:
            % the speed max(0, V + A t) is 1-Lipschitz in (V, A t) for
            % targets that travel forward (targetArc).
            % q may hold one column per time (each time from its own start).
            time=reshape(time,1,[]);
            tube=struct('positionRadius',zeros(size(time)),'yawRadius',zeros(size(time)), ...
                'courseRadius',zeros(size(time)),'speedRadius',zeros(size(time)));
            if isempty(errorSet),return;end
            arc=predictiveSafetyGeometry.targetArc(q,time);
            arcError=errorSet.speedRadius*abs(time)+.5*errorSet.accelerationRadius*time.^2;
            curvature=abs(sin(q(6,:))./q(7,:));
            tube.positionRadius=errorSet.positionRadius+arcError+abs(arc)*errorSet.courseRadius ...
                +.5*arc.^2*errorSet.curvatureRadius;
            tube.courseRadius=min(pi,errorSet.courseRadius+abs(arc)*errorSet.curvatureRadius ...
                +(curvature+errorSet.curvatureRadius).*arcError);
            tube.yawRadius=min(pi,tube.courseRadius+errorSet.sideslipRadius);
            tube.speedRadius=errorSet.speedRadius+errorSet.accelerationRadius*abs(time);
        end

        function support = targetPositionSupport(q,errorSet,time,normal)
            % Directional enclosure of the same constant-parameter family.
            % A speed error moves principally along the target path; a ball
            % would incorrectly charge its full size in every normal.
            support=zeros(size(normal,1),1);
            if isempty(errorSet),return;end
            arc=predictiveSafetyGeometry.targetArc(q,time);kappa=sin(q(6))/q(7);
            ds=errorSet.speedRadius*abs(time)+.5*errorSet.accelerationRadius*time^2;
            dg=errorSet.courseRadius;dk=errorSet.curvatureRadius;
            course=q(3)+q(6);theta=course+kappa*arc;
            point=predictiveSafetyGeometry.predictTarget(q,time);
            displacement=point(1:2)-q(1:2);
            courseDerivative=[-displacement(2);displacement(1)];
            a=kappa*arc;
            if abs(a)<1e-3
                % Integrate tau*J*t(course+kappa*tau) without division by a
                % vanishing curvature. The retained series has O(a^5) error.
                realPart=-a/3+a^3/30-a^5/840;
                imaginaryPart=.5-a^2/8+a^4/144-a^6/5760;
                curvatureDerivative=arc^2*[cos(course),-sin(course);sin(course),cos(course)] ...
                    *[realPart;imaginaryPart];
                seriesPadding=abs(arc)^2*abs(a)^7/45360;
            else
                curvatureDerivative=(arc*predictiveSafetyGeometry.direction(theta)-displacement)/kappa;
                seriesPadding=0;
            end
            % Taylor remainder in (course, curvature), followed by the arc
            % endpoint perturbation. Bounds hold also for negative arc length.
            length=abs(arc);
            remainder=.5*length*dg^2+.5*length^2*dg*dk+length^3*dk^2/6 ...
                +ds*(dg+length*dk)+.5*(abs(kappa)+dk)*ds^2+dk*seriesPadding;
            support=errorSet.positionRadius*vecnorm(normal,2,2) ...
                +abs(normal*predictiveSafetyGeometry.direction(theta))*ds ...
                +abs(normal*courseDerivative)*dg+abs(normal*curvatureDerivative)*dk ...
                +vecnorm(normal,2,2)*remainder;
            ball=predictiveSafetyGeometry.targetErrorTube(q,errorSet,time);
            support=min(support,vecnorm(normal,2,2)*ball.positionRadius);
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

        function next = predictTarget(q,time)
            % Sharma NRMM (2026), Eq. (17), in inertial coordinates:
            % Vdot=A, betadot=0, psidot=V*sin(beta)/lr, with braking that
            % ends at rest (targetArc): a speed that reaches zero stays zero
            % and the state's A becomes 0 there. Constant curvature makes
            % the position integral elementary even when A is nonzero.
            if isempty(q),next=q;return;end
            time=reshape(time,1,[]);[arc,speed,rest]=predictiveSafetyGeometry.targetArc(q,time);
            curvature=sin(q(6))/q(7);a=curvature*arc/2;scale=ones(size(a));
            nonzero=a~=0;scale(nonzero)=sin(a(nonzero))./a(nonzero);
            next=repmat(q,1,numel(time));
            next(1:2,:)=q(1:2)+(arc.*scale).*predictiveSafetyGeometry.direction(q(3)+q(6)+a);
            next(3,:)=q(3)+curvature*arc;next(4,:)=speed;next(5,rest)=0;
        end

        function [arc,speed,rest] = targetArc(q,time)
            % Signed arc length and speed of the targets q (one column, or one
            % per time) after 'time'. The speed is V + A t until it reaches
            % zero and zero from then on: braking ends at rest and never turns
            % the target back. A target at rest moves off only forward
            % (A > 0). For V >= 0 the speed is max(0, V + A t), so the arc
            % length is 1-Lipschitz in (V, A) per unit of (t, t^2/2) among
            % forward targets. rest marks the times at rest after a stop.
            % Negative times run the motion backward and may not pass
            % through rest (a stop is not reversible).
            V=q(4,:);A=q(5,:);
            speed=V+A.*time;arc=V.*time+.5*A.*time.^2;
            if any(time<0 & ((V>=0 & speed<0) | (V<0 & speed>0)))
                error('collisionAvoidanceController:irreversibleTargetStop', ...
                    'A target forecast cannot be run backward through rest.');
            end
            braking=V.*A<0;stopped=V==0 & A<=0;
            rest=time>0 & ((braking & time>=-V./A) | stopped);
            restArc=zeros(size(V));restArc(braking)=-V(braking).^2./(2*A(braking));
            restArc=restArc+zeros(size(time));
            arc(rest)=restArc(rest);speed(rest)=0;
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
            % The same ordinary-distance dual optimum
            % constructed from the exact closest-feature normal of rectangles.
            [primal,certificate]=predictiveSafetyGeometry.rectangle(poseE,shapeE,poseT,shapeT);
            [values,jacobian]=predictiveSafetyGeometry.fixedDualRows(poseE,shapeE,poseT,shapeT,certificate.lambda);
            gap=primal-min(values);
            assert(min(certificate.lambda)>=0 && norm(certificate.normal)<=1+1e-8 && abs(gap)<=1e-6);
            rows=struct('value',values,'jacobian',jacobian,'normal',certificate.normal, ...
                'lambda',certificate.lambda,'distance',min(values),'exitFlag',1,'dualityGap',gap);
        end

        function rows = supportLinearization(poseE,shapeE,poseT,shapeT)
            rows=predictiveSafetyGeometry.dualLinearization(poseE,shapeE,poseT,shapeT);
            if norm(rows.normal)>0,return;end
            % Ordinary distance has a zero optimal dual at overlap. Select
            % another feasible dual on the least-penetrated rectangle axis.
            % Its negative support gap retains a useful escape derivative;
            % requiring that gap to be positive is still sufficient separation.
            re=[cos(poseE(3)),-sin(poseE(3));sin(poseE(3)),cos(poseE(3))];
            rt=[cos(poseT(3)),-sin(poseT(3));sin(poseT(3)),cos(poseT(3))];
            centers=poseE(1:2)+re*shapeE(3:4)-poseT(1:2)-rt*shapeT(3:4);
            axes=[re,rt];offset=centers.'*axes;
            gap=abs(offset)-shapeE(1:2).'*abs(re.'*axes)-shapeT(1:2).'*abs(rt.'*axes);
            [~,index]=max(gap);orientation=1;
            if abs(offset(index))>1e-8
                if offset(index)<0,orientation=-1;end
            elseif axes(:,index).'*re(:,2)<0
                % At a symmetric overlap, roundoff must not select opposite
                % escape sides at adjacent prediction nodes. Break the tie
                % in the ego lateral frame, invariant to world rotation.
                orientation=-1;
            end
            normal=orientation*axes(:,index);body=rt.'*normal;
            lambda=[max(body,0);max(-body,0)];
            [values,jacobian]=predictiveSafetyGeometry.fixedDualRows(poseE,shapeE,poseT,shapeT,lambda);
            rows.value=values;rows.jacobian=jacobian;rows.normal=normal;rows.lambda=lambda;
            rows.dualityGap=rows.distance-min(values);
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
            % The target retains its exact prescribed motion. Adaptive midpoint
            % cuts resolve a Lipschitz lower bound down to half the sampled
            % margin. The other half supplies a finite refinement reserve.
            % This concerns the nominal interpolant, not the continuous plant.
            start=predictiveSafetyGeometry.predictTarget(targetEpoch,time);
            finish=predictiveSafetyGeometry.predictTarget(targetEpoch,time+duration);
            middle=predictiveSafetyGeometry.predictTarget(targetEpoch,time+duration/2);
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
                target=predictiveSafetyGeometry.predictTarget(targetEpoch,time+fraction*duration);
                value=predictiveSafetyGeometry.rectangle(pose,shape,target(1:3),target(8:11));
                check.fractions(end+1)=fraction;
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
                    error('collisionAvoidanceController:unsupportedReference','Polyline corners cannot define a smooth lane reference.');
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
