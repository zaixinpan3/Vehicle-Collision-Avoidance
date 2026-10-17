classdef predictiveSafetyGeometry
    %predictiveSafetyGeometry Target prediction, uncertainty tubes and polygon geometry.
    methods (Static)
        function guide = avoidanceGuide(model,count,side)
            % Bounded candidate bank, actual vehicle rollouts, no trajectory
            % optimizer. The field is guidance, not a robust safety filter.
            cfg=model.cfg;time=(0:count)*cfg.controller.sampleTime;
            target=predictiveSafetyGeometry.predictTarget(model.target,time);
            shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
            generator=model.uncertainty.collisionGenerator;
            reserve=sum(vecnorm(generator(1:2,:),2,1)) ...
                +2*(norm(shape(1:2))+norm(shape(3:4)))*sin(min(pi,sum(abs(generator(3,:))))/2);
            % Match the runtime controller's per-hold target enclosure. A
            % separate frozen-observer, full-horizon tube would rank seeds
            % against a different problem and can swamp the actual geometry.
            holdTime=[0,repmat(cfg.controller.sampleTime,1,count)];
            starts=predictiveSafetyGeometry.predictTarget(model.target,max(0,time-holdTime));
            support=predictiveSafetyGeometry.targetSupportData(starts,model.uncertainty.target,holdTime);
            tube=predictiveSafetyGeometry.targetErrorTube(starts,model.uncertainty.target,holdTime);
            targetRadius=norm(target(8:9,1))+norm(target(10:11,1));
            data=struct('target',target,'shape',shape,'reserve',reserve, ...
                'support',support,'yawReserve',2*targetRadius*sin(tube.yawRadius/2));
            data.parameters=nonlinearBicycleModel.nominalGuidanceParameters(cfg,model.nominalReference.curvature);
            context=terminalSafeSet.context(model);
            if isfield(model,'solveTimer')
                context.solveTimer=model.solveTimer;context.solveTimeLimitSeconds=model.solveTimeLimitSeconds;
            end
            data.context=context;
            free=localCirculationRollout(model,data,0,0,count,1);
            conflict=free.gaps<=0;first=NaN;last=NaN;
            if any(conflict),first=time(find(conflict,1));last=time(find(conflict,1,'last'));end
            best=free;total=1;diagnostics=localCandidateSummary(free);
            if any(conflict)
                % Near rest, a stopping continuation is meaningful whereas
                % acceleration-to-steering inversion has little authority.
                if hypot(model.initialState(4),model.initialState(5))<cfg.model.kinematicSpeedRange(1)
                    candidate=localCirculationRollout(model,data,0,0,count,-1);
                    total=total+1;diagnostics(end+1)=localCandidateSummary(candidate);
                    if localCandidateBetter(candidate,best),best=candidate;end
                end
                parameters=struct('direction',1,'speed',1,'inflation',reserve+support.positionRadius+data.yawReserve(1));
                [~,field]=predictiveSafetyGeometry.circulationField(model.initialState,target(:,1),cfg,parameters);
                projection=laneGeometry.project(model.initialState(1:2),model.lane);
                lateral=[-sin(projection.heading);cos(projection.heading)];
                preferred=sign((field.rotation*field.tangent).'*lateral)*side;
                if preferred==0,preferred=1;end
                directions=[preferred,-preferred];
                if isfield(model,'circulationDirection') && model.circulationDirection~=0
                    directions=model.circulationDirection;
                end
                budget=cfg.planner.lateralAccelerationFraction*min(cfg.tire.frictionCoefficient)*cfg.vehicle.gravity;
                radius=field.egoRadius+cfg.planner.contourOffsetFraction*cfg.vehicle.width+parameters.inflation;
                speed=min(max(norm(field.velocity),1),sqrt(budget*radius));
                usable=false;
                for fraction=cfg.planner.circulationSpeedFractions
                    for direction=directions
                        % Coasting keeps lateral authority when radial damping
                        % would brake prematurely. Both variants use the same
                        % circulation field and the actual Fiala rollout.
                        for longitudinalWeight=[0,1]
                            localPlannerDeadline(model);
                            candidate=localCirculationRollout(model,data,direction,speed*fraction,count,longitudinalWeight);
                            total=total+1;diagnostics(end+1)=localCandidateSummary(candidate); %#ok<AGROW>
                            if localCandidateBetter(candidate,best),best=candidate;end
                            usable=candidate.stateViolation<=1e-9 && min(candidate.gaps)>0 && candidate.terminalMember;
                            if usable,break;end
                        end
                        if usable,break;end
                    end
                    if usable,break;end
                end
            end
            guide=best;guide.time=time;guide.target=target;
            guide.position=best.states(1:2,:);guide.velocity=zeros(2,count+1);
            for index=1:count+1
                angle=best.states(3,index);rotation=[cos(angle),-sin(angle);sin(angle),cos(angle)];
                guide.velocity(:,index)=rotation*best.states(4:5,index);
            end
            guide.acceleration=[diff(guide.velocity,1,2)/cfg.controller.sampleTime,zeros(2,1)];
            displacement=guide.position-free.states(1:2,:);
            [~,furthest]=max(vecnorm(displacement,2,1));guide.direction=displacement(:,furthest);
            if norm(guide.direction)>1e-9,guide.direction=guide.direction/norm(guide.direction);
            else,guide.direction=side*[-sin(model.initialState(3));cos(model.initialState(3))];end
            guide.candidateCount=total;guide.candidates=diagnostics;
            guide.conflictStartSeconds=first;guide.conflictEndSeconds=last;
            guide.minimumGap=min(best.gaps);guide.nominalMinimumGap=min(best.nominalGaps);
            guide.initialConflict=conflict(1);guide.robustCertificate=false;
        end

        function [acceleration,field] = circulationField(x,target,cfg,parameters)
            % Ideal second-order field in the target rectangle frame. Target
            % translation, rotation and reference-point offset are retained.
            % Its barrier algebra is not a certificate for limited Fiala inputs.
            j=[0,-1;1,0];angle=target(3);r=[cos(angle),-sin(angle);sin(angle),cos(angle)];
            course=predictiveSafetyGeometry.direction(angle+target(6));
            curvature=sin(target(6))/target(7);omega=curvature*target(4);omegaDot=curvature*target(5);
            center=target(1:2)+r*target(10:11);
            centerVelocity=target(4)*course+omega*r*j*target(10:11);
            centerAcceleration=target(5)*course+curvature*target(4)^2*j*course ...
                +omegaDot*r*j*target(10:11)-omega^2*r*target(10:11);
            re=[cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
            q=r.'*(x(1:2)-center);w=r.'*(re*x(4:5)-centerVelocity)-omega*j*q;
            closest=min(target(8:9),max(-target(8:9),q));difference=q-closest;distance=norm(difference);
            corner=all(abs(q)>target(8:9));
            if distance>1e-10
                normal=difference/distance;kappa=double(corner)/distance;
            else
                % A colliding seed still needs finite guidance. This extension
                % has no clearance theorem and is never labelled certified.
                [depth,axis]=min(target(8:9)-abs(q));normal=zeros(2,1);
                normal(axis)=sign(q(axis));if normal(axis)==0,normal(axis)=1;end
                distance=-depth;kappa=0;
            end
            tangent=j*normal;nu=normal.'*w;tau=tangent.'*w;
            corners=cfg.vehicle.rectangleOffset+[1,1,-1,-1;1,-1,1,-1].*[cfg.vehicle.length/2;cfg.vehicle.width/2];
            egoRadius=max(vecnorm(corners,2,1));
            h=distance-egoRadius-parameters.inflation;
            desired=cfg.planner.contourOffsetFraction*cfg.vehicle.width;
            rate=cfg.planner.radialRate;
            relative=(-kappa*tau^2-2*rate*nu-rate^2*(h-desired))*normal ...
                +(kappa*nu*tau+cfg.planner.tangentialRate*(parameters.direction*parameters.speed-tau))*tangent;
            transport=r.'*centerAcceleration+2*omega*j*w+omegaDot*j*q-omega^2*q;
            acceleration=r*(relative+transport);
            field=struct('clearance',h,'normalSpeed',nu,'tangentSpeed',tau,'curvature',kappa, ...
                'normal',normal,'tangent',tangent,'position',q,'velocity',w,'rotation',r, ...
                'transport',transport,'egoRadius',egoRadius,'initialBarrier',nu+rate*h);
        end

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
            data=predictiveSafetyGeometry.targetSupportData(q,errorSet,time);
            support=predictiveSafetyGeometry.targetSupportFromData(data,1,normal);
        end

        function data = targetSupportData(q,errorSet,time)
            % Batch the parameter-family support coefficients once per frame.
            time=reshape(time,1,[]);count=numel(time);
            data=struct('positionRadius',0,'direction',zeros(2,count),'arcError',zeros(1,count), ...
                'courseDerivative',zeros(2,count),'curvatureDerivative',zeros(2,count), ...
                'courseRadius',0,'curvatureRadius',0,'remainder',zeros(1,count),'ballRadius',zeros(1,count));
            if isempty(errorSet),return;end
            arc=predictiveSafetyGeometry.targetArc(q,time);kappa=sin(q(6,:))./q(7,:);
            ds=errorSet.speedRadius*abs(time)+.5*errorSet.accelerationRadius*time.^2;
            dg=errorSet.courseRadius;dk=errorSet.curvatureRadius;
            course=q(3,:)+q(6,:);theta=course+kappa.*arc;
            half=kappa.*arc/2;scale=ones(size(half));nonzero=half~=0;
            scale(nonzero)=sin(half(nonzero))./half(nonzero);
            displacement=(arc.*scale).*predictiveSafetyGeometry.direction(course+half);
            a=kappa.*arc;small=abs(a)<1e-3;
            realPart=-a/3+a.^3/30-a.^5/840;
            imaginaryPart=.5-a.^2/8+a.^4/144-a.^6/5760;
            curvatureDerivative=arc.^2.*[cos(course).*realPart-sin(course).*imaginaryPart; ...
                sin(course).*realPart+cos(course).*imaginaryPart];
            large=~small;denominator=kappa;
            if isscalar(denominator),denominator=repmat(denominator,1,count);end
            if any(large)
                curvatureDerivative(:,large)=(arc(large).*predictiveSafetyGeometry.direction(theta(large)) ...
                    -displacement(:,large))./denominator(large);
            end
            seriesPadding=zeros(1,count);seriesPadding(small)=abs(arc(small)).^2.*abs(a(small)).^7/45360;
            length=abs(arc);
            remainder=.5*length*dg^2+.5*length.^2*dg*dk+length.^3*dk^2/6 ...
                +ds.*(dg+length*dk)+.5*(abs(kappa)+dk).*ds.^2+dk*seriesPadding;
            ball=predictiveSafetyGeometry.targetErrorTube(q,errorSet,time);
            data.positionRadius=errorSet.positionRadius;
            data.direction=predictiveSafetyGeometry.direction(theta);data.arcError=ds;
            data.courseDerivative=[-displacement(2,:);displacement(1,:)];
            data.curvatureDerivative=curvatureDerivative;
            data.courseRadius=dg;data.curvatureRadius=dk;
            data.remainder=remainder;data.ballRadius=ball.positionRadius;
        end

        function support = targetSupportFromData(data,index,normal)
            magnitude=vecnorm(normal,2,2);
            support=data.positionRadius*magnitude ...
                +abs(normal*data.direction(:,index))*data.arcError(index) ...
                +abs(normal*data.courseDerivative(:,index))*data.courseRadius ...
                +abs(normal*data.curvatureDerivative(:,index))*data.curvatureRadius ...
                +magnitude*data.remainder(index);
            support=min(support,magnitude*data.ballRadius(index));
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

function candidate=localCirculationRollout(model,data,direction,speed,count,longitudinalWeight)
    cfg=model.cfg;h=cfg.controller.sampleTime;
    candidate=struct('states',zeros(6,count+1),'inputs',zeros(2,count), ...
        'gaps',Inf(1,count+1),'nominalGaps',Inf(1,count+1),'directionSign',direction, ...
        'circulationSpeed',speed,'longitudinalWeight',longitudinalWeight, ...
        'releaseNode',NaN,'terminalMember',false,'stateViolation',0,'cost',0);
    candidate.states(:,1)=model.initialState;prior=model.previousInput;active=direction~=0;
    attempts=0;lastRelease=-Inf;encounterEnded=false;
    for index=1:count+1
        if mod(index,8)==1,localPlannerDeadline(model);end
        x=candidate.states(:,index);target=data.target(:,index);
        [gap,nominal]=localPlannerGap(x,target,data,index);
        outside=norm(x(1:2)-target(1:2))-data.reserve-data.support.ballRadius(index)>cfg.collision.encounterRangeMeters;
        % Only an observed active encounter can end. Never suppress a future
        % approach merely because an unobserved initial target is outside.
        if index>1 && outside && norm(model.initialState(1:2)-model.target(1:2))<=cfg.collision.encounterRangeMeters
            encounterEnded=true;
        end
        if encounterEnded,gap=Inf;nominal=Inf;end
        candidate.gaps(index)=gap;candidate.nominalGaps(index)=nominal;
        violation=localPlannerStateViolation(x,model);
        candidate.stateViolation=max(candidate.stateViolation,violation);
        error=nonlinearBicycleModel.error(x,model.lane,model.nominalReference);
        candidate.cost=candidate.cost+h*(error(1)^2+.2*error(3)^2+.1*x(6)^2);
        if index>count,break;end
        if active && encounterEnded,active=false;candidate.releaseNode=index-1;end
        if longitudinalWeight<0
            b=-cfg.terminal.brakingDecelerationMetersPerSecondSquared/modifiedFialaTire.accelerationGain(cfg);
            rate=cfg.model.brakingRatioRateMaximum*h;
            b=min(prior(2)+rate,max(prior(2)-rate,b));
            [lower,upper]=nonlinearBicycleModel.inputBounds(cfg,true);
            u=[prior(1);min(upper(2),max(lower(2),b))];
        elseif active
            parameters=struct('direction',direction,'speed',speed, ...
                'inflation',data.reserve+data.support.positionRadius+data.yawReserve(index));
            [acceleration,field]=predictiveSafetyGeometry.circulationField(x,target,cfg,parameters);
            % Release only after a complete return-to-route rollout clears
            % the same sampled target forecast. No local moving-away rule
            % alone authorizes the return.
            if field.normalSpeed>0 && index>2 && attempts<cfg.planner.releaseAttempts ...
                    && index-lastRelease>=max(1,ceil(count/(cfg.planner.releaseAttempts+1)))
                attempts=attempts+1;lastRelease=index;
                continuation=localFreeContinuation(x,prior,index,model,data,count);
                if continuation.valid
                    candidate.states(:,index:end)=continuation.states;
                    candidate.inputs(:,index:end)=continuation.inputs;
                    active=false;candidate.releaseNode=index-1;
                end
            end
            if active,u=nonlinearBicycleModel.accelerationGuidance(x,acceleration,prior,cfg,longitudinalWeight);
            else,u=candidate.inputs(:,index);end
        else
            u=nonlinearBicycleModel.nominalFeedback(x,prior,model.lane,model.nominalReference,cfg,data.parameters);
        end
        candidate.inputs(:,index)=u;candidate.states(:,index+1)=nonlinearBicycleModel.sample(x,u,cfg);prior=u;
    end
    candidate.terminalMember=terminalSafeSet.member(data.context,candidate.states(:,end),count);
    candidate.cost=candidate.cost+.01*sum(candidate.inputs(1,:).^2)+.01*sum(candidate.inputs(2,:).^2);
end

function continuation=localFreeContinuation(x,prior,index,model,data,count)
    cfg=model.cfg;steps=count-index+1;
    continuation=struct('states',zeros(6,steps+1),'inputs',zeros(2,steps),'valid',true);
    continuation.states(:,1)=x;
    for node=1:steps
        if mod(node,8)==1,localPlannerDeadline(model);end
        u=nonlinearBicycleModel.nominalFeedback(x,prior,model.lane,model.nominalReference,cfg,data.parameters);
        x=nonlinearBicycleModel.sample(x,u,cfg);prior=u;
        continuation.inputs(:,node)=u;continuation.states(:,node+1)=x;
        [gap,~]=localPlannerGap(x,data.target(:,index+node),data,index+node);
        if gap<=0 || localPlannerStateViolation(x,model)>1e-9,continuation.valid=false;return;end
    end
end

function violation=localPlannerStateViolation(x,model)
    cfg=model.cfg;
    violation=max([0,cfg.model.speedMinimum-x(4),x(4)-cfg.model.speedMaximum, ...
        abs(x(5))-cfg.model.lateralVelocityMaximum,abs(x(6))-cfg.model.yawRateMaximum]);
    if x(4)>cfg.model.brakeHoldSpeed
        violation=max(violation,abs(x(5))-tan(cfg.model.sideslipMaximum)*x(4));
    end
    if ~isempty(model.road.lateralClearance)
        body=cfg.vehicle.rectangleOffset+[cfg.vehicle.length/2;cfg.vehicle.width/2].*[1,1,-1,-1;1,-1,1,-1];
        rotation=[cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
        projection=laneGeometry.project(x(1:2)+rotation*body,model.lane);
        violation=max([violation,max(projection.lateralPosition)-model.road.lateralClearance(2), ...
            -min(projection.lateralPosition)-model.road.lateralClearance(1)]);
    end
end

function [gap,nominal]=localPlannerGap(x,target,data,index)
    re=[cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
    rt=[cos(target(3)),-sin(target(3));sin(target(3)),cos(target(3))];axes=[re,rt];
    center=x(1:2)+re*data.shape(3:4)-target(1:2)-rt*target(10:11);
    supports=data.shape(1:2).'*abs(re.'*axes)+target(8:9).'*abs(rt.'*axes);
    rows=abs(center.'*axes)-supports;nominal=max(rows);
    targetReserve=predictiveSafetyGeometry.targetSupportFromData(data.support,index,axes.');
    gap=max(rows-targetReserve.'-data.reserve-data.yawReserve(index));
end

function better=localCandidateBetter(candidate,best)
    % A validated nominal is preferred, but an incomplete seed may still be
    % repaired by RTI. Never label a least-violating seed as a safe witness.
    a=[candidate.stateViolation,max(0,-min(candidate.nominalGaps)),max(0,-min(candidate.gaps)),double(~candidate.terminalMember),candidate.cost];
    b=[best.stateViolation,max(0,-min(best.nominalGaps)),max(0,-min(best.gaps)),double(~best.terminalMember),best.cost];
    difference=find(abs(a-b)>1e-9,1);better=~isempty(difference) && a(difference)<b(difference);
end

function summary=localCandidateSummary(candidate)
    summary=struct('direction',candidate.directionSign,'speed',candidate.circulationSpeed, ...
        'longitudinalWeight',candidate.longitudinalWeight, ...
        'minimumGap',min(candidate.gaps),'nominalMinimumGap',min(candidate.nominalGaps), ...
        'terminalMember',candidate.terminalMember,'stateViolation',candidate.stateViolation, ...
        'releaseNode',candidate.releaseNode,'cost',candidate.cost);
end

function localPlannerDeadline(model)
    if isfield(model,'solveTimer') && toc(model.solveTimer)>=model.solveTimeLimitSeconds
        error('collisionAvoidanceController:timeBudgetExceeded','Circulation initialization exhausted the RTI budget.');
    end
end
