classdef predictiveSafetyGeometry
    %predictiveSafetyGeometry Target prediction, uncertainty tubes and polygon geometry.
    methods (Static)
        function pose = targetPoseSet(target,errorSet,time)
            % One persistent NRMM family, propagated from its actual epoch.
            % Interval arc length uses monotonicity in initial speed and A,
            % including forward-only stopping. No future measurement reset.
            if isempty(errorSet)
                q=predictiveSafetyGeometry.predictTarget(target,time);pose=nrmmSet(q(1:3));return;
            end
            speed=max(0,target(4)+[-1,1]*errorSet.speedRadius);
            acceleration=target(5)+[-1,1]*errorSet.accelerationRadius;
            arc=zeros(1,2);
            for j=1:2
                duration=time;
                if acceleration(j)<0,duration=min(time,-speed(j)/acceleration(j));end
                arc(j)=speed(j)*duration+.5*acceleration(j)*duration^2;
            end
            q=nrmmSet([target(1:2);target(3)+target(6);sin(target(6))/target(7);target(6)], ...
                diag([errorSet.positionRadius;errorSet.positionRadius;errorSet.courseRadius; ...
                    errorSet.curvatureRadius;errorSet.sideslipRadius]));
            s=nrmmSet.range(arc(1),arc(2));half=q(4)*s/2;
            position=q(1:2)+s*sinc(half)*[cos(q(3)+half);sin(q(3)+half)];
            pose=[position;q(3)+2*half-q(5)];
        end

        function [gap,normal] = setSeparation(ego,shapeE,target,shapeT,normal)
            % Verified fixed-world-normal supports of uncertain rectangles.
            if nargin<5 || isempty(normal)
                point=predictiveSafetyGeometry.supportLinearization(ego.center,shapeE,target.center,shapeT);
                normal=point.normal;
            end
            assert(abs(norm(normal)-1)<1e-8,'predictiveSafetyGeometry:invalidNormal','A unit normal is required.');
            normal=normal(:);minimum=Inf;maximum=-Inf;
            rE=[cos(ego(3)),-sin(ego(3));sin(ego(3)),cos(ego(3))];
            rT=[cos(target(3)),-sin(target(3));sin(target(3)),cos(target(3))];
            signs=[-1,1,1,-1;-1,-1,1,1];
            for j=1:4
                [lo,~]=bounds(normal.'*(ego(1:2)+rE*(shapeE(3:4)+shapeE(1:2).*signs(:,j))));
                [~,hi]=bounds(normal.'*(target(1:2)+rT*(shapeT(3:4)+shapeT(1:2).*signs(:,j))));
                minimum=min(minimum,lo);maximum=max(maximum,hi);
            end
            gap=minimum-maximum;
        end

        function valid = stateSetAdmissible(state,cfg)
            [lo,hi]=bounds(state);
            valid=lo(4)>=cfg.model.speedMinimum && hi(4)<=cfg.model.speedMaximum ...
                && max(abs([lo(5),hi(5)]))<=cfg.model.lateralVelocityMaximum ...
                && max(abs([lo(6),hi(6)]))<=cfg.model.yawRateMaximum;
            tangent=tan(cfg.model.sideslipMaximum);
            [~,one]=bounds(state(5)-tangent*state(4));[~,two]=bounds(-state(5)-tangent*state(4));
            valid=valid && max(one,two)<=0;
        end

        function guide = avoidanceGuide(model,count)
            % Route-following initialization, corrected by an avoidance field.
            % No candidate scores or admission gate. RTI receives the states
            % and inputs over exactly count holds. Terminal constraints
            % belong to RTI, not this heuristic initialization.
            cfg=model.cfg;time=(0:count)*cfg.controller.sampleTime;
            target=predictiveSafetyGeometry.predictTarget(model.target,time);
            shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
            generator=model.uncertainty.collisionGenerator;
            reserve=sum(vecnorm(generator(1:2,:),2,1));
            % Match the runtime controller's per-hold target enclosure. A
            % separate frozen-observer, full-horizon tube would rank seeds
            % against a different problem and can swamp the actual geometry.
            holdTime=[0,repmat(cfg.controller.sampleTime,1,count)];
            starts=predictiveSafetyGeometry.predictTarget(model.target,max(0,time-holdTime));
            support=predictiveSafetyGeometry.targetSupportData(starts,model.uncertainty.target,holdTime);
            tube=predictiveSafetyGeometry.targetErrorTube(starts,model.uncertainty.target,holdTime);
            data=struct('target',target,'shape',shape,'reserve',reserve, ...
                'generator',generator,'egoYawRadius',sum(abs(generator(3,:))), ...
                'support',support,'targetYawRadius',tube.yawRadius);
            parameters=localFieldParameters(data,1,0);
            [~,field]=predictiveSafetyGeometry.circulationField(model.initialState,target(:,1),cfg,parameters);
            course=model.initialState(3)+atan2(model.initialState(5),model.initialState(4));
            forward=[cos(course);sin(course)];
            lateral=[-forward(2);forward(1)];
            budget=cfg.planner.lateralAccelerationFraction*min(cfg.tire.frictionCoefficient)*cfg.vehicle.gravity;
            % Convert dimensionless prediction gradients to acceleration
            % using the encounter's relative-motion and turning scales.
            % This positive scale sets neither a side nor a target velocity.
            contourRadius=max(field.egoRadius,norm(field.position));
            speed=min(max(norm(field.velocity),1),sqrt(budget*contourRadius));
            % One autonomous field, integrated once. Route attraction,
            % obstacle circulation and road guidance act at every moving node;
            % no separate nominal rollout classifies or selects a behavior.
            best=localCirculationRollout(model,data,speed,count);
            total=1;diagnostics=localCandidateSummary(best);
            time=(0:numel(best.gaps)-1)*cfg.controller.sampleTime;
            conflict=best.gaps<=0;first=NaN;last=NaN;
            if any(conflict),first=time(find(conflict,1));last=time(find(conflict,1,'last'));end
            guide=best;
            guide.time=(0:size(best.inputs,2))*cfg.controller.sampleTime;
            guide.target=predictiveSafetyGeometry.predictTarget(model.target,guide.time);
            guide.position=best.states(1:2,:);guide.velocity=zeros(2,size(best.states,2));
            for index=1:size(best.states,2)
                angle=best.states(3,index);rotation=[cos(angle),-sin(angle);sin(angle),cos(angle)];
                guide.velocity(:,index)=rotation*best.states(4:5,index);
            end
            guide.acceleration=[diff(guide.velocity,1,2)/cfg.controller.sampleTime,zeros(2,1)];
            % Record transverse excursion relative to the initial motion,
            % independently of the displacement to the cruise route.
            transverse=lateral.'*(guide.position-model.initialState(1:2));
            [~,furthest]=max(abs(transverse));
            % A displacement observed AFTER integration, not a turn command.
            guide.direction=transverse(furthest)*lateral;
            guide.candidateCount=total;guide.candidates=diagnostics;
            guide.conflictStartSeconds=first;guide.conflictEndSeconds=last;
            guide.minimumGap=min(best.gaps);guide.nominalMinimumGap=min(best.nominalGaps);
            guide.initialConflict=conflict(1);guide.robustCertificate=false;
        end

        function [acceleration,active] = wallGuidance(x,acceleration,model)
            % Perceived boundary guidance, with a parallel-wall fallback.
            % The fallback clips transverse acceleration and anticipates outward speed.
            % Exact realization would enforce second-order wall inequalities;
            % Fiala tracking and RTI modification still need replay validation.
            if isfield(model.road,'boundaryObservation')
                [acceleration,active]=predictiveSafetyGeometry.roadBoundaryGuidance(x,acceleration,model);return;
            end
            width=model.cfg.planner.guideHalfWidthMeters;active=false;
            if isinf(width),return;end
            projection=laneGeometry.project(x(1:2),model.lane);
            tangent=[cos(projection.heading);sin(projection.heading)];normal=[-tangent(2);tangent(1)];
            rotation=[cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];velocity=rotation*x(4:5);
            along=tangent.'*velocity;across=normal.'*velocity;e=projection.lateralPosition;
            frame=predictiveSafetyGeometry.roadFrame(model.lane,model.road);curvature=frame(4);
            feedforward=curvature*along^2/max(1-curvature*e,.1);
            rate=model.cfg.planner.wallRate;
            center=feedforward-2*rate*across-rate^2*e;
            requested=normal.'*acceleration;
            limited=min(center+rate^2*width,max(center-rate^2*width,requested));
            acceleration=acceleration+(limited-requested)*normal;
            active=limited~=requested;
        end

        function [acceleration,active,info] = roadBoundaryGuidance(x,preferred,model)
            % Local signed distance to the observed road boundary UNION.
            % A chain's left side is only a local orientation, not a global
            % halfspace containing every drivable component. Query the
            % nearest boundary at the footprint points instead of requiring
            % membership on the left of every chain (separate carriageways
            % and islands otherwise impose contradictory conditions).
            % Range-cut ends retain their tangent continuation. No unobserved
            % map geometry or hard controller road constraint is introduced.
            acceleration=preferred;active=false;
            info=struct('normal',zeros(0,2),'lower',zeros(0,1),'distance',zeros(0,1), ...
                'compatible',true,'maximumDeficit',0);
            if ~isfield(model.road,'boundaryObservation'),return;end
            chains=model.road.boundaryObservation.chains;
            if isempty(chains),return;end
            rows=zeros(5*numel(chains),2);lower=zeros(size(rows,1),1);
            distances=zeros(size(lower));count=0;
            rotation=[cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];velocity=rotation*x(4:5);
            cfg=model.cfg;shape=cfg.vehicle.rectangleOffset+[cfg.vehicle.length/2;cfg.vehicle.width/2].*[1,1,-1,-1;1,-1,1,-1];
            vertices=rotation*shape;rate=cfg.planner.wallRate;
            queries=x(1:2)+[zeros(2,1),vertices];
            for point=1:size(queries,2)
                values=zeros(numel(chains),1);normals=zeros(2,numel(chains));hessians=zeros(2,2,numel(chains));
                for j=1:numel(chains)
                    [values(j),normals(:,j),hessians(:,:,j)]=localBoundaryDistance(queries(:,point),chains(j));
                end
                nearest=min(abs(values));
                % Retain all ties at medial axes; no arbitrary side choice.
                selected=find(abs(values)<=nearest+1e-9*max(1,nearest));
                for j=selected.'
                    normal=normals(:,j);hessian=hessians(:,:,j);
                    count=count+1;rows(count,:)=normal.';distances(count)=values(j);
                    lower(count)=-velocity.'*hessian*velocity-2*rate*(normal.'*velocity)-rate^2*values(j);
                end
            end
            rows=rows(1:count,:);lower=lower(1:count);distances=distances(1:count);
            [acceleration,compatible]=localAccelerationProjection(preferred,rows,lower);
            active=norm(acceleration-preferred)>1e-10;
            info.normal=rows;info.lower=lower;info.distance=distances;
            info.compatible=compatible;info.maximumDeficit=max([0;lower-rows*acceleration]);
        end

        function [input,next,active] = boundaryReferenceInput(x,input,previous,model)
            % Modify a warm/nominal seed only, never an optimized command.
            active=false;cfg=model.cfg;
            if isfield(model.road,'boundaryObservation')
                derivative=nonlinearBicycleModel.derivative(x,input,cfg);
                rotation=[cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
                preferred=rotation*[derivative(4)-x(6)*x(5);derivative(5)+x(6)*x(4)];
                [acceleration,active]=predictiveSafetyGeometry.roadBoundaryGuidance(x,preferred,model);
                if active,input=nonlinearBicycleModel.accelerationGuidance(x,acceleration,previous,cfg);end
            end
            if nargout>1,next=nonlinearBicycleModel.sample(x,input,cfg);end
        end

        function [acceleration,field] = circulationField(x,target,cfg,parameters)
            % Second-order target-frame guidance with directional uncertainty.
            % Body-yaw support and uncertainty change along the rollout.
            % The frozen-geometry ODE is not a certificate for this heuristic
            % or for the limited Fiala inputs. Gap checks are diagnostics.
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
                depth=min(target(8:9)-abs(q));
                faceGap=[q-target(8:9);-q-target(8:9)];
                weight=exp((faceGap-max(faceGap))/min(target(8:9)));
                weight=weight/sum(weight);normal=weight(1:2)-weight(3:4);
                distance=-depth;kappa=0;
            end
            tangent=j*normal;nu=normal.'*w;tau=tangent.'*w;
            corners=cfg.vehicle.rectangleOffset+[1,1,-1,-1;1,-1,1,-1].*[cfg.vehicle.length/2;cfg.vehicle.width/2];
            egoRadius=max(vecnorm(corners,2,1));
            normalWorld=r*normal;targetYawRadius=0;
            if isfield(parameters,'targetYawRadius'),targetYawRadius=parameters.targetYawRadius;end
            [~,targetReserve]=predictiveSafetyGeometry.rotationSupport(target(8:11),target(3),targetYawRadius,normalWorld.');
            inflation=parameters.inflation+targetReserve;
            if isfield(parameters,'generator')
                inflation=inflation+sum(abs(normalWorld.'*parameters.generator(1:2,:))) ...
                    +predictiveSafetyGeometry.targetSupportFromData(parameters.support,parameters.index,normalWorld.');
            end
            % Directional ego support avoids an artificial disk-width buffer
            % beside the target. The derivative terms remain guidance only:
            % body yaw and directional uncertainty change during the rollout.
            shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
            yawRadius=0;if isfield(parameters,'egoYawRadius'),yawRadius=parameters.egoYawRadius;end
            egoSupport=predictiveSafetyGeometry.rotationSupport(shape,x(3),yawRadius,-normalWorld.');
            h=distance-egoSupport-inflation;
            anticipation=localAnticipation(x,target,cfg,parameters,normalWorld);
            tangentialAcceleration=cfg.planner.tangentialRate*parameters.speed ...
                *(anticipation.transportGradient+anticipation.separationGradient);
            desired=0;
            rate=cfg.planner.radialRate;
            relative=(-kappa*tau^2-2*rate*nu-rate^2*(h-desired))*normal ...
                +(kappa*nu*tau)*tangent;
            transport=r.'*centerAcceleration+2*omega*j*w+omegaDot*j*q-omega^2*q;
            acceleration=r*(relative+transport)+tangentialAcceleration;
            correction=0;activation=1;incidence=1;
            if isfield(parameters,'preferredAcceleration')
                % Keep only the outward normal correction. Add both signed
                % tangential vectors directly: there is no selected side,
                % one-sided tangential clipping, or release timer.
                preferred=parameters.preferredAcceleration;normalWorld=r*normal;
                incidence=anticipation.incidence;
                correction=incidence*max(0,normalWorld.'*(acceleration-preferred));
                budget=cfg.planner.lateralAccelerationFraction*min(cfg.tire.frictionCoefficient)*cfg.vehicle.gravity;
                activation=min(1,correction/budget);
                acceleration=preferred+correction*normalWorld ...
                    +activation*tangentialAcceleration;
            end
            field=struct('clearance',h,'normalSpeed',nu,'tangentSpeed',tau,'curvature',kappa, ...
                'normal',normal,'tangent',tangent,'position',q,'velocity',w,'rotation',r, ...
                'transport',transport,'egoRadius',egoRadius, ...
                'inflation',inflation,'desiredClearance',desired,'initialBarrier',nu+rate*h, ...
                'normalCorrection',correction,'activation',activation,'incidence',incidence, ...
                'anticipationTime',anticipation.time, ...
                'transportGradient',anticipation.transportGradient, ...
                'separationGradient',anticipation.separationGradient, ...
                'tangentialAcceleration',tangentialAcceleration);
        end

        function [support,inflation] = rotationSupport(shape,yaw,yawRadius,normal)
            % Exact directional support of a rectangle over a yaw interval.
            % Each offset vertex contributes a*cos(delta)+b*sin(delta).
            % Its maximum is at an endpoint or its interior angular maximum.
            % Includes reference-point offsets, wrapped angles and full yaw
            % uncertainty; normal rows need not be normalized.
            rotation=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
            vertices=rotation*(shape(3:4)+shape(1:2).*[-1,1,1,-1;-1,-1,1,1]);
            a=normal*vertices;nominal=max(a,[],2);
            radius=min(pi,yawRadius);
            if radius==0,support=nominal;inflation=zeros(size(nominal));return;end
            b=normal*[-vertices(2,:);vertices(1,:)];
            values=a*cos(radius)+abs(b)*sin(radius);
            interior=abs(atan2(b,a))<=radius;
            magnitude=hypot(a,b);values(interior)=magnitude(interior);
            support=max(nominal,max(values,[],2));
            inflation=max(0,support-nominal);
        end

        function uncertainty = observerUncertainty(ego,observation,q)
            % A common uncertain rigid pose cancels from pairwise distance.
            % Relative target balls are therefore used in the frame frozen at
            % this sample; path and CLF rows still use the full ego error box.
            uncertainty=struct('specified',ego.uncertaintySpecified, ...
                'egoGenerator',ego.errorGenerator,'collisionGenerator',ego.errorGenerator, ...
                'target',[],'relativeFrame',false);
            if isfield(ego,'velocityErrorGeometry') && ~isempty(ego.velocityErrorGeometry)
                uncertainty.velocityGeometry=ego.velocityErrorGeometry;
            end
            if isfield(ego,'jointPrediction') && ~isempty(ego.jointPrediction)
                uncertainty.jointPrediction=ego.jointPrediction;
            end
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
            uncertainty.predictionSet=set;
            uncertainty.collisionGenerator(1:3,:)=0;
            uncertainty.relativeFrame=true;uncertainty.specified=true;
        end

        function [q,uncertainty,memory,info] = reconcileTarget(q,prior,uncertainty,memory,time)
            % The observer point and a nominal predictor need not coincide.
            % Intersect the declared constant-A/constant-curvature intervals
            % across samples. Propagate the speed interval with that family.
            % Select the closest continuation of the previous nominal inside
            % the current enclosure, then recenter EVERY affected radius.
            % There is no smoothing gain, rate cap, or hidden reduction of
            % the observer's uncertainty. This is a box/ball outer enclosure,
            % not a nonlinear cross-horizon recursive-safety certificate.
            info=struct('available',false,'reason',"observerSetUnavailable", ...
                'observed',q,'nominal',q,'parameterIntersection',false);
            if isempty(q) || ~isfield(uncertainty,'predictionSet')
                memory=[];return;
            end
            set=uncertainty.predictionSet;
            acceleration=set.accelerationInterval;curvature=set.curvatureInterval;speed=set.speedInterval;
            reused=~isempty(memory) && ~isempty(prior);
            if reused
                elapsed=time-memory.time;
                assert(elapsed>=0,'collisionAvoidanceController:invalidTargetPredictionTime', ...
                    'A target enclosure cannot be propagated backward.');
                propagatedAcceleration=memory.accelerationInterval;
                mayStop=memory.speedInterval(1)+elapsed*propagatedAcceleration(1)<=0 ...
                    && propagatedAcceleration(1)<=0;
                allStop=memory.speedInterval(2)+elapsed*propagatedAcceleration(2)<=0 ...
                    && propagatedAcceleration(2)<=0;
                if allStop
                    propagatedAcceleration(:)=0;
                elseif mayStop
                    % predictTarget resets A to zero on the stopped branch.
                    % Retain both branches in the interval hull when only
                    % some members have stopped; this does not invent a
                    % change of the moving target's constant acceleration.
                    propagatedAcceleration(2)=max(0,propagatedAcceleration(2));
                end
                acceleration=localIntervalIntersection(acceleration,propagatedAcceleration);
                curvature=localIntervalIntersection(curvature,memory.curvatureInterval);
                startAcceleration=acceleration;
                if mayStop,startAcceleration=memory.accelerationInterval;end
                propagated=max(0,memory.speedInterval+elapsed*startAcceleration);
                speed=localIntervalIntersection(speed,propagated);
            end
            observed=q;preferred=q;if ~isempty(prior),preferred=prior;end
            q(4)=min(speed(2),max(speed(1),preferred(4)));
            q(5)=min(acceleration(2),max(acceleration(1),preferred(5)));
            kappa=min(curvature(2),max(curvature(1),sin(preferred(6))/preferred(7)));
            q(6)=asin(kappa*q(7));
            course=observed(3)+observed(6);
            change=atan2(sin(preferred(3)+preferred(6)-course),cos(preferred(3)+preferred(6)-course));
            change=min(set.courseRadius,max(-set.courseRadius,change));
            q(3)=course+change-q(6);
            offset=preferred(1:2)-observed(1:2);distance=norm(offset);
            if distance>set.positionRadius,offset=offset*(set.positionRadius/distance);end
            q(1:2)=observed(1:2)+offset;
            uncertainty.target=struct('positionRadius',set.positionRadius+norm(offset), ...
                'courseRadius',min(pi,set.courseRadius+abs(change)), ...
                'speedRadius',max(abs(speed-q(4))), ...
                'accelerationRadius',max(abs(acceleration-q(5))), ...
                'curvatureRadius',max(abs(curvature-kappa)), ...
                'sideslipRadius',max(abs(asin(curvature*q(7))-q(6))));
            memory=struct('time',time,'speedInterval',speed, ...
                'accelerationInterval',acceleration,'curvatureInterval',curvature);
            info.available=true;info.reason="observerEnclosureContinuation";
            info.nominal=q;info.parameterIntersection=reused;
        end

        function generator = egoErrorGenerator(state,uncertainty)
            % Evaluate the published measurement geometry at this nominal
            % velocity. Primitive sensor/residual budgets are retained;
            % only the rotation sensitivity and its remainder change.
            % This is a scheduled observer contract, not a certificate that
            % unknown future innovations obey the present residual budget.
            generator=uncertainty.egoGenerator;
            if ~isfield(uncertainty,'velocityGeometry'),return;end
            geometry=uncertainty.velocityGeometry;
            if size(generator,2)~=8,return;end
            velocity=state(4:5);
            residual=abs(geometry.measurementOffset);
            generator(4:5,:)=0;
            generator(4:5,3)=[velocity(2);-velocity(1)]*geometry.yawRadius;
            generator(4:5,4:5)=geometry.noiseRadius*eye(2);
            % Future residual signs are unknown. Bound the rotation of
            % their entire box, rather than reusing the present signed
            % residual as a fictitious future measurement.
            remainder=norm(velocity)*geometry.yawRadius^2/2 ...
                +norm(residual)*min(geometry.yawRadius,2);
            generator(4:5,6:7)=diag(residual+remainder);
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

        function [rows,bounds] = conservativeRotationRows(rows,bounds,yawMap,vertexReach,yawRadius)
            % Inner approximation for the rotating vertices. On |dpsi|<=eta,
            % |g-g_affine| <= reach*eta*|dpsi|/2. Globally the derivative
            % difference is <=2*reach, giving a valid unbounded-domain fall-
            % back. Both reserves vanish at dpsi=0. No new safety distance.
            validateattributes(yawRadius,{'double'},{'scalar','nonnegative','nonnan'});
            slope=vertexReach(:)*min(yawRadius/2,2);
            correction=slope*yawMap(:).';
            rows=[rows+correction;rows-correction];bounds=[bounds;bounds];
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

function prediction=localAnticipation(x,target,cfg,parameters,normal)
    % A time-aligned ballistic characteristic activates a transported field.
    % Only this diagnostic ray is ballistic; the returned seed uses Fiala.
    % Future target poses use the full NRMM arc, including braking to rest.
    rotation=[cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
    velocity=rotation*x(4:5);
    targetVelocity=target(4)*predictiveSafetyGeometry.direction(target(3)+target(6));
    relativeSpeed=norm(velocity-targetVelocity);
    scale=hypot(cfg.vehicle.length/2,cfg.vehicle.width/2)+norm(target(8:9));
    duration=(cfg.collision.encounterRangeMeters+norm(x(1:2)-target(1:2))) ...
        /max(relativeSpeed,cfg.planner.radialRate*scale);
    time=linspace(0,duration,max(2,ceil(duration/cfg.controller.sampleTime)+1));
    future=predictiveSafetyGeometry.predictTarget(target,time);count=numel(time);
    positions=x(1:2)+velocity*time;
    targetAxes=predictiveSafetyGeometry.direction(future(3,:));
    axes=[repmat(rotation(:,1).',count,1);repmat(rotation(:,2).',count,1); ...
        targetAxes.';[-targetAxes(2,:);targetAxes(1,:)].'];
    targetCenter=future(1:2,:)+target(10)*targetAxes+target(11)*[-targetAxes(2,:);targetAxes(1,:)];
    % Both orientations enter the same smooth distance gradient. No face,
    % passing side or circulation sign is selected.
    axes=[axes;-axes];
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    egoYaw=0;targetYaw=0;
    if isfield(parameters,'egoYawRadius'),egoYaw=parameters.egoYawRadius;end
    if isfield(parameters,'targetYawRadius'),targetYaw=parameters.targetYawRadius;end
    egoSupport=predictiveSafetyGeometry.rotationSupport(shape,x(3),egoYaw,-axes);
    targetSupport=localMovingSupport(target(8:11),repmat(future(3,:).',8,1),targetYaw,axes);
    inflation=parameters.inflation;
    if isfield(parameters,'generator')
        inflation=inflation+sum(abs(axes*parameters.generator(1:2,:)),2) ...
            +predictiveSafetyGeometry.targetSupportFromData(parameters.support,parameters.index,axes);
    end
    faceGap=reshape(sum(axes.*repmat((positions-future(1:2,:)).',8,1),2) ...
        -egoSupport-targetSupport-inflation,count,8);
    gaps=max(faceGap,[],2).';
    width=min(cfg.vehicle.width/2,target(9));
    % Gradient of width*log(sum(exp(faceGap/width))). The subtraction of
    % each row maximum only stabilizes evaluation. Balanced faces cancel
    % exactly on the symmetry axis instead of choosing a default direction.
    faceWeight=exp((faceGap-gaps.')/width);faceWeight=faceWeight./sum(faceWeight,2);
    separationNormal=[sum(faceWeight.*reshape(axes(:,1),count,8),2), ...
        sum(faceWeight.*reshape(axes(:,2),count,8),2)].';
    % A predicted exit ends this encounter; no distant return is penalized.
    rangeReserve=parameters.inflation;
    if isfield(parameters,'generator')
        rangeReserve=rangeReserve+sum(vecnorm(parameters.generator(1:2,:),2,1)) ...
            +parameters.support.ballRadius(parameters.index);
    end
    outside=vecnorm(positions-future(1:2,:),2,1)-rangeReserve>cfg.collision.encounterRangeMeters;
    if ~outside(1)
        exit=find(outside,1);if ~isempty(exit),gaps(exit:end)=Inf;end
    end
    depth=min(1,max(0,-gaps)/width);
    exposure=depth.*(2-depth);
    prediction=struct('incidence',max(exposure), ...
        'time',Inf,'transportGradient',zeros(2,1),'separationGradient',zeros(2,1));
    weight=exposure.*exp(-cfg.planner.radialRate*time);
    if sum(weight)<=eps,return;end
    weight=weight/sum(weight);prediction.time=sum(weight.*time);
    % Query the future obstacle's distance gradient at the CURRENT ego
    % position. Its tangential transport favors space the target is vacating.
    % Exposure above is time-aligned; these future positions are never made
    % a static forbidden union. This is a passing preference, not a barrier.
    delta=x(1:2)-targetCenter;
    q=[sum(targetAxes.*delta,1);sum([-targetAxes(2,:);targetAxes(1,:)].*delta,1)];
    residual=q-min(target(8:9),max(-target(8:9),q));
    % Gradient of sqrt(dist(q,K)^2+width^2). It remains continuous when
    % the query crosses a future rectangle boundary or enters its interior.
    distance=vecnorm(residual,2,1);localNormal=residual./hypot(distance,width);
    worldNormal=targetAxes.*localNormal(1,:)+[-targetAxes(2,:);targetAxes(1,:)].*localNormal(2,:);
    gradient=sum((worldNormal-normal).*weight,2);
    tangent=[-normal(2);normal(1)];projection=tangent*tangent.';
    prediction.transportGradient=projection*gradient;
    prediction.separationGradient=projection*sum(separationNormal.*(weight.*max(0,-gaps)/width),2);
end

function support=localMovingSupport(shape,yaw,yawRadius,normal)
    % Batched rectangle supports, one target yaw per query row.
    c=cos(yaw);s=sin(yaw);
    bodyNormal=[normal(:,1).*c+normal(:,2).*s,-normal(:,1).*s+normal(:,2).*c];
    vertices=shape(3:4)+shape(1:2).*[-1,1,1,-1;-1,-1,1,1];
    a=bodyNormal*vertices;nominal=max(a,[],2);
    radius=min(pi,yawRadius);
    b=bodyNormal*[-vertices(2,:);vertices(1,:)];
    values=a*cos(radius)+abs(b)*sin(radius);
    interior=abs(atan2(b,a))<=radius;
    magnitude=hypot(a,b);values(interior)=magnitude(interior);
    support=max(nominal,max(values,[],2));
end

function interval=localIntervalIntersection(first,second)
    interval=[max(first(1),second(1));min(first(2),second(2))];
    tolerance=64*eps(max(1,max(abs([first(:);second(:)]))));
    assert(interval(1)<=interval(2)+tolerance, ...
        'collisionAvoidanceController:inconsistentTargetPredictionSet', ...
        'Successive observer enclosures contradict the constant-parameter target model.');
    if interval(1)>interval(2),interval(:)=mean(interval);end
end

function [distance,normal,hessian]=localBoundaryDistance(position,chain)
    points=chain.points;starts=points(1:end-1,:);segments=diff(points);lengths=vecnorm(segments,2,2);
    tangent=segments./lengths;projection=sum((position.'-starts).*tangent,2);
    low=zeros(size(lengths));high=lengths;
    if chain.extendStart,low(1)=-Inf;end
    if chain.extendEnd,high(end)=Inf;end
    along=min(high,max(low,projection));
    nearest=starts+along.*tangent;delta=position.'-nearest;
    [squared,index]=min(sum(delta.^2,2));magnitude=sqrt(squared);
    inward=[-tangent(index,2);tangent(index,1)];
    endpoint=abs(along(index))<1e-10 || abs(along(index)-lengths(index))<1e-10;
    vertex=[];
    if abs(along(index))<1e-10,vertex=index;
    elseif abs(along(index)-lengths(index))<1e-10,vertex=index+1;end
    adjacent=[];
    if ~isempty(vertex) && vertex>1 && vertex<size(points,1)
        adjacent=vertex-1:vertex;
    elseif ~isempty(vertex) && norm(points(1,:)-points(end,:))<1e-10
        adjacent=[size(tangent,1),1];
    end
    if ~isempty(adjacent)
        inward=[-sum(tangent(adjacent,2));sum(tangent(adjacent,1))];
        if norm(inward)>1e-10,inward=inward/norm(inward);else,inward=[-tangent(index,2);tangent(index,1)];end
    end
    signDistance=sign(delta(index,:)*inward);if signDistance==0,signDistance=1;end
    distance=signDistance*magnitude;normal=inward;hessian=zeros(2);
    if endpoint && magnitude>1e-8
        radial=delta(index,:).'/magnitude;normal=signDistance*radial;
        hessian=signDistance*(eye(2)-radial*radial.')/magnitude;
    end
end

function [best,compatible]=localAccelerationProjection(preferred,normal,lower)
    % Exact Euclidean projection onto compatible 2-D halfplanes. Enumerate
    % zero, one or two active lines, not trajectory candidates or an NLP.
    best=preferred;compatible=all(normal*preferred>=lower-1e-9);
    if compatible,return;end
    bestCost=Inf;leastDeficit=max([0;lower-normal*preferred]);least=preferred;
    count=size(normal,1);
    for i=1:count
        candidate=preferred+(lower(i)-normal(i,:)*preferred)*normal(i,:).';
        consider(candidate,true);
        for j=i+1:count
            pair=normal([i,j],:);
            if abs(det(pair))>1e-6,consider(pair\lower([i,j]),false);end
        end
    end
    compatible=isfinite(bestCost);if ~compatible,best=least;end
    function consider(candidate,individual)
        deficit=max([0;lower-normal*candidate]);cost=sum((candidate-preferred).^2);
        % An intersection of nearly parallel incompatible lines can be
        % arbitrarily far away. Such a pair is useful only when it actually
        % satisfies ALL rows; it cannot become an infeasible fallback.
        if individual && deficit<leastDeficit,leastDeficit=deficit;least=candidate;end
        if deficit<=1e-8 && cost<bestCost,best=candidate;bestCost=cost;end
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

function candidate=localCirculationRollout(model,data,speed,count)
    cfg=model.cfg;h=cfg.controller.sampleTime;
    candidate=struct('states',zeros(6,count+1),'inputs',zeros(2,count), ...
        'gaps',zeros(1,count+1),'nominalGaps',zeros(1,count+1), ...
        'speedScale',speed, ...
        'releaseNode',NaN,'stateViolation',0,'fieldActive',false, ...
        'maximumDeviation',0,'completion',"requestedHorizon");
    candidate.states(:,1)=model.initialState;
    prior=model.previousInput;encounterEnded=false;
    for index=1:count+1
        if mod(index,8)==1,localPlannerDeadline(model);end
        x=candidate.states(:,index);target=data.target(:,index);
        [gap,nominal]=localPlannerGap(x,target,data,index);
        outside=norm(x(1:2)-target(1:2))-data.reserve-data.support.ballRadius(index)>cfg.collision.encounterRangeMeters;
        if index>1 && outside && norm(model.initialState(1:2)-model.target(1:2))<=cfg.collision.encounterRangeMeters
            encounterEnded=true;
        end
        if encounterEnded,gap=Inf;nominal=Inf;end
        candidate.gaps(index)=gap;candidate.nominalGaps(index)=nominal;
        violation=localPlannerStateViolation(x,model);
        candidate.stateViolation=max(candidate.stateViolation,violation);
        % The route attracts the seed; road boundaries also shape it.
        projection=laneGeometry.project(x(1:2),model.lane);
        candidate.maximumDeviation=max(candidate.maximumDeviation,abs(projection.lateralPosition));
        if index>count,break;end
        if hypot(x(4),x(5))<=cfg.model.brakeHoldSpeed
            b=-cfg.terminal.brakingDecelerationMetersPerSecondSquared/modifiedFialaTire.accelerationGain(cfg);
            change=cfg.model.brakingRatioRateMaximum*h;
            u=[prior(1);min(prior(2)+change,max(prior(2)-change,b))];
        else
            % Route attraction is the preferred acceleration BEFORE obstacle
            % and boundary correction. Never overwrite longitudinal obstacle
            % acceleration with a cruise-speed request afterwards.
            acceleration=localRouteAcceleration(x,projection,model);
            if ~encounterEnded
                parameters=localFieldParameters(data,index,speed);
                parameters.preferredAcceleration=acceleration;
                [acceleration,field]=predictiveSafetyGeometry.circulationField(x,target,cfg,parameters);
                candidate.fieldActive=candidate.fieldActive || field.activation>0;
                if field.activation==0 && candidate.fieldActive && isnan(candidate.releaseNode),candidate.releaseNode=index-1;end
            end
            acceleration=predictiveSafetyGeometry.wallGuidance(x,acceleration,model);
            u=nonlinearBicycleModel.accelerationGuidance(x,acceleration,prior,cfg,1,true);
        end
        candidate.inputs(:,index)=u;
        next=nonlinearBicycleModel.sample(x,u,cfg);
        candidate.states(:,index+1)=next;prior=u;
    end
end

function acceleration=localRouteAcceleration(x,projection,model)
    % A course field converges to the path without prescribing waypoints or
    % an avoidance/return schedule. The existing initialization gains set its
    % lookahead, course response and speed recovery; no CLF row is introduced.
    cfg=model.cfg;p=cfg.nominalClf;speed=hypot(x(4),x(5));
    course=x(3)+atan2(x(5),x(4));
    error=atan2(sin(course-projection.heading),cos(course-projection.heading));
    lateral=projection.lateralPosition;
    referenceSpeed=hypot(model.nominalReference.state(4),model.nominalReference.state(5));
    lookahead=max(p.minimumLookaheadMeters,p.lookaheadSeconds*referenceSpeed);
    desired=-atan(lateral/lookahead);
    desiredRate=-lookahead/(lookahead^2+lateral^2)*speed*sin(error);
    courseError=atan2(sin(error-desired),cos(error-desired));
    curvature=model.nominalReference.curvature;
    courseRate=curvature*speed*cos(error)/max(1-curvature*lateral,.1) ...
        +desiredRate-p.courseGain*courseError;
    forward=[cos(course);sin(course)];normal=[-forward(2);forward(1)];
    acceleration=p.speedGain*(referenceSpeed-speed)*forward+speed*courseRate*normal;
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
    orientation=sign(center.'*axes);orientation(orientation==0)=1;
    normal=(axes.*orientation).';
    targetReserve=predictiveSafetyGeometry.targetSupportFromData(data.support,index,normal);
    egoSupport=predictiveSafetyGeometry.rotationSupport(data.shape,x(3),data.egoYawRadius,-normal);
    targetSupport=predictiveSafetyGeometry.rotationSupport(target(8:11),target(3),data.targetYawRadius(index),normal);
    gap=max(normal*(x(1:2)-target(1:2))-egoSupport-targetSupport ...
        -sum(abs(normal*data.generator(1:2,:)),2)-targetReserve);
end

function parameters=localFieldParameters(data,index,speed)
    parameters=struct('speed',speed,'inflation',0, ...
        'egoYawRadius',data.egoYawRadius,'targetYawRadius',data.targetYawRadius(index), ...
        'generator',data.generator,'support',data.support,'index',index);
end

function summary=localCandidateSummary(candidate)
    summary=struct('speedScale',candidate.speedScale, ...
        'minimumGap',min(candidate.gaps),'nominalMinimumGap',min(candidate.nominalGaps), ...
        'stateViolation',candidate.stateViolation, ...
        'releaseNode',candidate.releaseNode,'maximumDeviation',candidate.maximumDeviation, ...
        'completion',candidate.completion);
end

function localPlannerDeadline(model)
    if isfield(model,'solveTimer') && toc(model.solveTimer)>=model.solveTimeLimitSeconds
        error('collisionAvoidanceController:timeBudgetExceeded','Circulation initialization exhausted the RTI budget.');
    end
end
