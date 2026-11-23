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
            arc=predictiveSafetyGeometry.targetArcInterval(target,errorSet,time);
            pose=predictiveSafetyGeometry.targetArcSet(target,errorSet,arc);
        end

        function arc = targetArcInterval(target,errorSet,time)
            speed=max(0,target(4)+[-1,1]*errorSet.speedRadius);
            acceleration=target(5)+[-1,1]*errorSet.accelerationRadius;
            if isfield(errorSet,'speedInterval'),speed=errorSet.speedInterval(:).';end
            if isfield(errorSet,'accelerationInterval'),acceleration=errorSet.accelerationInterval(:).';end
            arc=zeros(1,2);
            for j=1:2
                duration=time;
                if acceleration(j)<0,duration=min(time,-speed(j)/acceleration(j));end
                arc(j)=speed(j)*duration+.5*acceleration(j)*duration^2;
            end
        end

        function pose = targetArcSet(target,errorSet,arc)
            % A spatial image of the SAME constant-curvature family. This
            % also represents every stopping location in an arc interval.
            assert(numel(arc)==2 && 0<=arc(1) && arc(1)<=arc(2));
            if isempty(errorSet)
                errorSet=struct('positionRadius',0,'courseRadius',0,'curvatureRadius',0,'sideslipRadius',0);
            end
            q=nrmmSet([target(1:2);target(3)+target(6);sin(target(6))/target(7);target(6)], ...
                diag([errorSet.positionRadius;errorSet.positionRadius;errorSet.courseRadius; ...
                    errorSet.curvatureRadius;errorSet.sideslipRadius]));
            if isfield(errorSet,'curvatureInterval')
                interval=errorSet.curvatureInterval;
                q(4)=nrmmSet(mean(interval),[zeros(1,3),diff(interval)/2,0]);
                % Curvature and body/course offset are the SAME constant
                % beta parameter, rather than two independent disturbances.
                q(5)=asin(target(7)*q(4));
            end
            % Arc uncertainty is independent of initial position. Give it
            % its own signed column; reusing column 1 would falsely link
            % travelled distance to the target's initial x-position error.
            s=nrmmSet(mean(arc),[zeros(1,size(q.generator,2)),diff(arc)/2]);half=q(4)*s/2;
            position=q(1:2)+s*sinc(half)*[cos(q(3)+half);sin(q(3)+half)];
            % Intersect the signed affine image with a direct trigonometric
            % range. The latter retains, for example, that an oncoming arc
            % initially travels backwards in x; a symmetric Taylor radius
            % can otherwise invent forward displacement at the same time.
            [kl,kh]=bounds(q(4));turn=[kl,kh].'*arc/2;
            turn=[min(turn,[],'all'),max(turn,[],'all')];
            maximum=max(abs(turn));minimum=max(0,max(turn(1),-turn(2)));
            sincRange=[-1,1];
            if maximum<=pi
                sincRange=[1,1];
                if maximum>0,sincRange(1)=sin(maximum)/maximum;end
                if minimum>0,sincRange(2)=sin(minimum)/minimum;end
            end
            amplitude=arc.'*sincRange;
            amplitude=[min(amplitude,[],'all'),max(amplitude,[],'all')];
            [cl,ch]=bounds(q(3));angle=[cl+turn(1),ch+turn(2)];
            lower=zeros(2,1);upper=lower;
            for coordinate=1:2
                phase=angle-(coordinate-1)*pi/2;
                if coordinate==1,values=cos(angle);else,values=sin(angle);end
                if ceil(phase(1)/(2*pi))<=floor(phase(2)/(2*pi)),values(end+1)=1;end
                if ceil((phase(1)-pi)/(2*pi))<=floor((phase(2)-pi)/(2*pi)),values(end+1)=-1;end
                product=amplitude.'*[min(values),max(values)];
                lower(coordinate)=target(coordinate)-errorSet.positionRadius+min(product,[],'all');
                upper(coordinate)=target(coordinate)+errorSet.positionRadius+max(product,[],'all');
            end
            guard=128*eps(max(1,max(abs([lower;upper]))));
            lower=lower-guard;upper=upper+guard;
            position=nrmmSet.intersectRange(position,lower,upper);
            pose=[position;q(3)+2*half-q(5)];
        end

        function acceleration=routeAcceleration(x,projection,model)
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

        function guide = avoidanceGuide(model,count,holds)
            if nargin<3,holds=ones(1,count);end
            [~,guide]=predictiveSafetyGeometry.avoidanceInputs(model,count,holds);
        end

        function [inputs,guide] = avoidanceInputs(model,count,holds)
            % The field supplies the held controls for one initialization.
            % Request the second output to inspect the field trajectory.
            if nargin<3,holds=ones(1,count);end
            cfg=model.cfg;durations=holds*cfg.controller.sampleTime;
            time=[0,cumsum(holds)]*cfg.controller.sampleTime;
            target=predictiveSafetyGeometry.predictTarget(model.target,time);
            shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
            generator=model.uncertainty.collisionGenerator;
            reserve=sum(vecnorm(generator(1:2,:),2,1));
            % Guide and optimizer must use the same persistent target family
            % whenever joint prediction is selected.
            holdTime=[0,durations];
            jointTarget=isfield(model.uncertainty,'jointPrediction') ...
                && (cfg.controller.jointAffinePrediction || cfg.controller.jointNonlinearPrediction);
            jointTarget=jointTarget || (~isempty(model.uncertainty.target) && isfield(model.uncertainty.target,'family'));
            if jointTarget
                holdTime=time;
            end
            starts=target;
            if ~isempty(model.uncertainty.target)
                starts=predictiveSafetyGeometry.predictTarget(model.target,max(0,time-holdTime));
            end
            [support,tube]=predictiveSafetyGeometry.targetSupportData(starts,model.uncertainty.target,holdTime);
            if isempty(tube),tube=predictiveSafetyGeometry.targetErrorTube(starts,model.uncertainty.target,holdTime);end
            data=struct('target',target,'shape',shape,'reserve',reserve, ...
                'generator',generator,'egoYawRadius',sum(abs(generator(3,:))), ...
                'support',support,'targetYawRadius',tube.yawRadius);
            parameters=localFieldParameters(data,1,0);
            [~,field]=predictiveSafetyGeometry.circulationField(model.initialState,target(:,1),cfg,parameters);
            budget=cfg.planner.lateralAccelerationFraction*min(cfg.tire.frictionCoefficient)*cfg.vehicle.gravity;
            % Bound the tangential speed scale by the available turning acceleration.
            % This positive scale sets neither a side nor a target velocity.
            contourRadius=max(field.egoRadius,norm(field.position));
            speed=min(max(norm(field.velocity),1),sqrt(budget*contourRadius));
            % Integrate route, obstacle and road guidance together on one trajectory.
            best=localCirculationRollout(model,data,speed,durations,nargout>1);
            inputs=best.inputs;
            if nargout<2,return;end
            total=1;diagnostics=localCandidateSummary(best);
            conflict=best.gaps<=0;first=NaN;last=NaN;
            if any(conflict),first=time(find(conflict,1));last=time(find(conflict,1,'last'));end
            guide=best;
            guide.time=time;
            guide.target=target;
            guide.position=best.states(1:2,:);guide.velocity=zeros(2,size(best.states,2));
            for index=1:size(best.states,2)
                angle=best.states(3,index);rotation=[cos(angle),-sin(angle);sin(angle),cos(angle)];
                guide.velocity(:,index)=rotation*best.states(4:5,index);
            end
            guide.acceleration=[diff(guide.velocity,1,2)./durations,zeros(2,1)];
            % Record transverse excursion relative to the initial motion,
            % independently of the displacement to the cruise route.
            course=model.initialState(3)+atan2(model.initialState(5),model.initialState(4));
            lateral=[-sin(course);cos(course)];
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
            % Fiala tracking and optimized commands still need replay validation.
            if isfield(model.road,'boundaryObservation')
                [acceleration,active]=predictiveSafetyGeometry.roadBoundaryGuidance(x,acceleration,model);return;
            end
            width=model.cfg.planner.guideHalfWidthMeters;active=false;
            if isinf(width),return;end
            projection=predictiveSafetyGeometry.project(x(1:2),model.lane);
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
            % Local guide for a single initialization. The PCBF evaluates safety.
            j=[0,-1;1,0];angle=target(3);r=[cos(angle),-sin(angle);sin(angle),cos(angle)];
            course=predictiveSafetyGeometry.direction(angle+target(6));
            curvature=sin(target(6))/target(7);omega=curvature*target(4);omegaDot=curvature*target(5);
            center=target(1:2)+r*target(10:11);
            centerVelocity=target(4)*course+omega*r*j*target(10:11);
            centerAcceleration=target(5)*course+curvature*target(4)^2*j*course ...
                +omegaDot*r*j*target(10:11)-omega^2*r*target(10:11);
            if isfield(parameters,'centerVelocity')
                centerVelocity=centerVelocity ...
                    +parameters.crossWeight*(parameters.centerVelocity-centerVelocity);
            end
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
            if isfield(parameters,'extentRate')
                nu=nu-parameters.crossWeight*abs(normal).'*parameters.extentRate;
            end
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
            % Transverse offset and closing velocity are evaluated at this node.
            egoOffset=re*cfg.vehicle.rectangleOffset;
            delta=x(1:2)+egoOffset-center;
            velocity=re*x(4:5)+x(6)*j*egoOffset-centerVelocity;
            transverse=j*velocity/max(norm(velocity),eps);
            offset=transverse.'*delta;
            halfWidth=abs(transverse.'*re)*shape(1:2)+abs(transverse.'*r)*target(8:9)+inflation;
            incidence=double(h<0);
            if delta.'*velocity<0
                incidence=max(incidence,max(0,1-(offset/max(halfWidth,eps))^2));
            end
            tangentWorld=r*tangent;
            rate=cfg.planner.radialRate;
            lateral=parameters.speed*transverse*(offset/hypot(offset,min(shape(2),target(9))));
            tangentialAcceleration=cfg.planner.tangentialRate*tangentWorld*(tangentWorld.'*(lateral-centerVelocity));
            desired=0;
            relative=(-kappa*tau^2-2*rate*nu-rate^2*(h-desired))*normal ...
                +(kappa*nu*tau)*tangent;
            transport=r.'*centerAcceleration+2*omega*j*w+omegaDot*j*q-omega^2*q;
            acceleration=r*(relative+transport)+tangentialAcceleration;
            correction=0;activation=1;
            if isfield(parameters,'preferredAcceleration')
                % Apply the outward correction and the local tangential field.
                preferred=parameters.preferredAcceleration;normalWorld=r*normal;
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

        function [q,uncertainty,memory,info] = reconcileTarget(q,prior,uncertainty,memory,time)
            % The observer point and a nominal predictor need not coincide.
            % Intersect the declared constant-A/constant-curvature intervals
            % across samples. Propagate the speed interval with that family.
            % Center the predictor on the CURRENT observer point, projected
            % into the intersected parameter intervals. Keeping an old point
            % merely because it remains possible delays observed acceleration
            % and unnecessarily enlarges position/course balls. This is a
            % box/ball outer enclosure,
            % not a nonlinear cross-horizon recursive-safety certificate.
            info=struct('available',false,'reason',"observerSetUnavailable", ...
                'observed',q,'nominal',q,'parameterIntersection',false,'courseIntersection',false);
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
            observed=q;preferred=q;
            relativeCourse=set.courseCenter+[-1;1]*set.courseRadius;
            if reused && isfield(memory,'relativeCourseInterval') && isfield(set,'measurementHistory')
                [transported,available]=localCourseContinuation(memory,set.measurementHistory,time,curvature);
                if available
                    center=mean(transported);
                    measured=center+atan2(sin(set.courseCenter-center),cos(set.courseCenter-center));
                    relativeCourse=localIntervalIntersection(transported,measured+[-1;1]*set.courseRadius);
                    info.courseIntersection=true;
                end
            end
            q(4)=min(speed(2),max(speed(1),preferred(4)));
            q(5)=min(acceleration(2),max(acceleration(1),preferred(5)));
            kappa=min(curvature(2),max(curvature(1),sin(preferred(6))/preferred(7)));
            q(6)=asin(kappa*q(7));
            center=mean(relativeCourse);
            measured=center+atan2(sin(set.courseCenter-center),cos(set.courseCenter-center));
            selected=min(relativeCourse(2),max(relativeCourse(1),measured));
            q(3)=set.referenceEgoPose(3)+selected-q(6);
            offset=preferred(1:2)-observed(1:2);distance=norm(offset);
            if distance>set.positionRadius,offset=offset*(set.positionRadius/distance);end
            q(1:2)=observed(1:2)+offset;
            uncertainty.target=struct('positionRadius',set.positionRadius+norm(offset), ...
                'courseRadius',min(pi,max(abs(relativeCourse-selected))), ...
                'speedRadius',max(abs(speed-q(4))), ...
                'accelerationRadius',max(abs(acceleration-q(5))), ...
                'curvatureRadius',max(abs(curvature-kappa)), ...
                'sideslipRadius',max(abs(asin(curvature*q(7))-q(6))), ...
                'speedInterval',speed,'accelerationInterval',acceleration,'curvatureInterval',curvature);
            family=[];
            if reused && isfield(memory,'family'),family=memory.family;end
            if isfield(set,'measurementHistory') && isfield(set.measurementHistory,'time') ...
                    && ~isempty(set.measurementHistory.time) && set.measurementHistory.constantCurvature
                yawRadius=sum(abs(uncertainty.egoGenerator(3,:)));
                intervals=[speed,acceleration,curvature, ...
                    set.referenceEgoPose(3)+relativeCourse+[-1;1]*yawRadius];
                family=nrmmTargetHistory('constantFamily',set.measurementHistory,time,intervals,family);
                uncertainty.target.family=family;
                uncertainty.target.familyTime=time;uncertainty.target.familyYawRadius=yawRadius;
            end
            memory=struct('time',time,'speedInterval',speed, ...
                'accelerationInterval',acceleration,'curvatureInterval',curvature, ...
                'relativeCourseInterval',relativeCourse);
            if ~isempty(family),memory.family=family;end
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
            if isfield(errorSet,'family')
                image=nrmmTargetHistory("constantMotion", 'image',errorSet.family,errorSet.familyTime, ...
                    errorSet.familyTime+time,errorSet.familyYawRadius);
                half=sin(q(6,:))./q(7,:).*arc/2;
                scale=ones(size(half));active=half~=0;scale(active)=sin(half(active))./half(active);
                displacement=(arc.*scale).*predictiveSafetyGeometry.direction(q(3,:)+q(6,:)+half);
                cells=size(errorSet.family.lower,2);
                extent=max(abs(reshape(image.lower,2,cells,[])-reshape(displacement,2,1,[])), ...
                    abs(reshape(image.upper,2,cells,[])-reshape(displacement,2,1,[])));
                radius=reshape(max(sqrt(sum(extent.^2,1)),[],2),1,[]);
                tube.positionRadius=min(tube.positionRadius,errorSet.positionRadius+radius);
            end
        end

        function support = targetDirectionalSupport(q,errorSet,time,normal)
            % Upper support of the complete constant-NRMM target family.
            normal=normal(:);magnitude=norm(normal);angle=atan2(normal(2),normal(1));
            signs=[-1,1,1,-1;-1,-1,1,1];vertices=q(10:11)+q(8:9).*signs;
            if isempty(errorSet)
                pose=predictiveSafetyGeometry.predictTarget(q,time);
                support=normal.'*pose(1:2)+max(magnitude*vecnorm(vertices).* ...
                    cos(pose(3)+atan2(vertices(2,:),vertices(1,:))-angle));return;
            end
            arc=predictiveSafetyGeometry.targetArcInterval(q,errorSet,time);
            curve=sin(q(6))/q(7)+[-1,1]*errorSet.curvatureRadius;
            if isfield(errorSet,'curvatureInterval'),curve=errorSet.curvatureInterval(:).';end
            course=q(3)+q(6)+[-1,1]*errorSet.courseRadius;
            kl=curve(1);kh=curve(2);sl=arc(1);sh=arc(2);
            cl=course(1);ch=course(2);
            product=[kl.*sl,kl.*sh,kh.*sl,kh.*sh]/2;
            hl=min(product,[],2);hh=max(product,[],2);
            far=max(abs([hl,hh]),[],2);near=max(0,max(hl,-hh));
            sincLow=-ones(size(far));sincHigh=ones(size(far));inside=far<=pi;
            sincLow(inside)=1;nonzero=inside & far>0;sincLow(nonzero)=sin(far(nonzero))./far(nonzero);
            nonzero=inside & near>0;sincHigh(nonzero)=sin(near(nonzero))./near(nonzero);
            product=[sl.*sincLow,sl.*sincHigh,sh.*sincLow,sh.*sincHigh];
            al=min(product,[],2);ah=max(product,[],2);
            [lo,hi]=localCosRange(cl+hl-angle,ch+hh-angle);
            displacement=max([al.*lo,al.*hi,ah.*lo,ah.*hi],[],2);
            if isfield(errorSet,'curvatureInterval')
                betaLow=asin(max(-1,min(1,q(7)*kl)));betaHigh=asin(max(-1,min(1,q(7)*kh)));
            else
                betaLow=q(6)-errorSet.sideslipRadius;betaHigh=q(6)+errorSet.sideslipRadius;
            end
            yawLow=cl+2*hl-betaHigh;yawHigh=ch+2*hh-betaLow;
            vertexUpper=-Inf(size(displacement));
            for k=1:4
                offset=atan2(vertices(2,k),vertices(1,k))-angle;
                [~,hi]=localCosRange(yawLow+offset,yawHigh+offset);
                vertexUpper=max(vertexUpper,norm(vertices(:,k))*hi);
            end
            support=normal.'*q(1:2)+magnitude*(errorSet.positionRadius+max(displacement+vertexUpper));
            if isfield(errorSet,'family')
                image=nrmmTargetHistory("constantMotion", 'image',errorSet.family,errorSet.familyTime, ...
                    errorSet.familyTime+time,errorSet.familyYawRadius);
                position=sum(max(normal.*image.lower,normal.*image.upper),1);
                yawLow=image.courseLower-asin(max(-1,min(1,q(7)*image.curvatureUpper)));
                yawHigh=image.courseUpper-asin(max(-1,min(1,q(7)*image.curvatureLower)));
                footprint=-Inf(size(position));
                for k=1:4
                    offset=atan2(vertices(2,k),vertices(1,k))-angle;
                    [~,hi]=localCosRange(yawLow+offset,yawHigh+offset);
                    footprint=max(footprint,magnitude*norm(vertices(:,k))*hi);
                end
                conditioned=normal.'*q(1:2)+magnitude*errorSet.positionRadius+max(position+footprint);
                support=min(support,conditioned);
            end
            support=support+256*eps(max(1,abs(support)));
        end

        function [data,ball] = targetSupportData(q,errorSet,time)
            % Batch the parameter-family support coefficients once per frame.
            time=reshape(time,1,[]);count=numel(time);
            data=struct('positionRadius',0,'direction',zeros(2,count),'arcError',zeros(1,count), ...
                'courseDerivative',zeros(2,count),'curvatureDerivative',zeros(2,count), ...
                'courseRadius',0,'curvatureRadius',0,'remainder',zeros(1,count),'ballRadius',zeros(1,count));
            ball=[];if isempty(errorSet),return;end
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

        function [box,centers] = targetMotionEnvelope(root,limits,time,nominal,domain)
            % Enclose the continuous constant-A, constant-beta footprint in
            % axes centered on the possible body headings at each node.
            time=reshape(time,1,[]);
            if root(4)==0
                box=repmat(root,1,numel(time));centers=zeros(4,numel(time));return
            end
            if nargin<4 || isempty(nominal),nominal=predictiveSafetyGeometry.predictTarget(root,time);end
            box=nominal;
            centers=zeros(4,numel(time));
            if isempty(limits),return;end
            % Forward arc length is monotone in A, including absorption.
            slow=root;slow(5)=limits(1,1);
            fast=root;fast(5)=limits(2,1);
            arcLow=predictiveSafetyGeometry.targetArc(slow,time);
            arcHigh=predictiveSafetyGeometry.targetArc(fast,time);
            curvature=sin(limits(:,2))/root(7);
            if nargin>=5 && isstruct(domain) && isfield(domain,'yawRateMaximum')
                % On a persistent-curvature arc, |kappa|*V is the physical
                % yaw rate. Bound distance traveled before constructing its
                % Cartesian image, including times far beyond the handoff.
                speedLimit=Inf;
                if isfield(domain,'speedMaximum'),speedLimit=domain.speedMaximum;end
                if prod(curvature)>0
                    speedLimit=min(speedLimit,domain.yawRateMaximum/min(abs(curvature)));
                end
                if isfinite(speedLimit),arcHigh=min(arcHigh,speedLimit*time);end
                arcLow=min(arcLow,arcHigh);
            end
            turns=[curvature(1)*arcLow;curvature(1)*arcHigh;curvature(2)*arcLow;curvature(2)*arcHigh];
            halfLow=min(turns,[],1)/2;halfHigh=max(turns,[],1)/2;
            if nargin>=5 && isstruct(domain) && isfield(domain,'yawRateMaximum')
                halfTurnLimit=domain.yawRateMaximum*time/2;
                halfLow=max(-halfTurnLimit,min(halfTurnLimit,halfLow));
                halfHigh=max(-halfTurnLimit,min(halfTurnLimit,halfHigh));
            end
            far=max(abs(halfLow),abs(halfHigh));near=max(0,max(halfLow,-halfHigh));
            sincLow=-ones(size(time));sincHigh=ones(size(time));
            short=far<=pi;sincLow(short)=1;
            use=short & far>0;sincLow(use)=sin(far(use))./far(use);
            use=short & near>0;sincHigh(use)=sin(near(use))./near(use);
            products=[arcLow.*sincLow;arcHigh.*sincLow;arcLow.*sincHigh;arcHigh.*sincHigh];
            chordLow=min(products,[],1);chordHigh=max(products,[],1);
            % On a turning interval the chord is also 2*sin(half turn)/curvature.
            % This bounds its radius through arbitrarily many revolutions.
            if any(~short) && curvature(1)*curvature(2)>0 && min(abs(curvature))>eps
                [low,high]=localCosRange(halfLow-pi/2,halfHigh-pi/2);
                chords=2*[low/curvature(1);low/curvature(2);high/curvature(1);high/curvature(2)];
                % At small curvature the sinc form avoids cancellation.
                use=far>pi;
                chordLow(use)=max(chordLow(use),min(chords(:,use),[],1));
                chordHigh(use)=min(chordHigh(use),max(chords(:,use),[],1));
            end
            midHeading=halfLow+halfHigh;
            box(3,:)=root(3)+midHeading;
            courseLow=limits(1,2)+halfLow-midHeading;courseHigh=limits(2,2)+halfHigh-midHeading;
            yawLow=2*halfLow-midHeading;yawHigh=2*halfHigh-midHeading;
            vertices=root(10:11)+root(8:9).*[-1,1,1,-1;-1,-1,1,1];
            vertexAngles=atan2(vertices(2,:),vertices(1,:)).';
            radii=vecnorm(vertices).';
            axes=[cos(box(3,:));sin(box(3,:))];
            offset=nominal(1:2,:)-root(1:2);
            padding=64*eps(max([1;abs(root(1:2));root(8:9);abs(root(10:11))])+arcHigh).*(1+far);
            for k=1:2
                phase=(k-1)*pi/2;
                [low,high]=localCosRange(courseLow-phase,courseHigh-phase);
                projections=[chordLow.*low;chordLow.*high;chordHigh.*low;chordHigh.*high]-sum(axes.*offset,1);
                row=2*k-1;
                lowerCenter=min(projections,[],1);
                upperCenter=max(projections,[],1);
                % Project the same circular arc using the difference of its
                % endpoint sines. This keeps the signed position on a turn
                % bounded even when the arc spans several revolutions.
                if curvature(1)*curvature(2)>0
                    alpha=midHeading+phase;
                    [sl,sh]=localCosRange(limits(1,2)+2*halfLow-alpha-pi/2, ...
                        limits(2,2)+2*halfHigh-alpha-pi/2);
                    [il,ih]=localCosRange(limits(1,2)-alpha-pi/2,limits(2,2)-alpha-pi/2);
                    q=[(sl-ih)/curvature(1);(sl-ih)/curvature(2); ...
                        (sh-il)/curvature(1);(sh-il)/curvature(2)]-sum(axes.*offset,1);
                    lo=max(lowerCenter,min(q,[],1));hi=min(upperCenter,max(q,[],1));
                    consistent=lo<=hi;
                    lowerCenter(consistent)=lo(consistent);upperCenter(consistent)=hi(consistent);
                end
                centers(row,:)=lowerCenter-padding;
                centers(row+1,:)=upperCenter+padding;
                [vl,vh]=localCosRange(yawLow+vertexAngles-phase,yawHigh+vertexAngles-phase);
                lower=centers(row,:)+min(radii.*vl,[],1);
                upper=centers(row+1,:)+max(radii.*vh,[],1);
                box(7+k,:)=(upper-lower)/2;
                box(9+k,:)=(upper+lower)/2;
                axes=[-axes(2,:);axes(1,:)];
            end
        end

        function range=targetEnvelopeRange(ego,target,centers)
            % Distance to the box of possible dynamics reference points.
            d=ego(1:2,:)-target(1:2,:);c=cos(target(3,:));s=sin(target(3,:));
            relative=[c.*d(1,:)+s.*d(2,:);-s.*d(1,:)+c.*d(2,:)];
            nearest=max(0,max(centers([1,3],:)-relative,relative-centers([2,4],:)));
            range=vecnorm(nearest);
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

        function [gaps,gradient,activeProjection] = rectangleAxisGaps(poseE,shapeE,poseT,shapeT,smoothing)
            % Signed projection gaps on both rectangles' edge normals.
            % A positive maximum is a lower bound on Euclidean clearance.
            % Positive smoothing lowers the center projection and increases
            % support radii, so each returned gap remains conservative.
            if nargin<5,smoothing=0;end
            ce=cos(poseE(3,:));se=sin(poseE(3,:));ct=cos(poseT(3,:));st=sin(poseT(3,:));
            delta=poseE(1:2,:)-poseT(1:2,:) ...
                +[ce.*shapeE(3,:)-se.*shapeE(4,:);se.*shapeE(3,:)+ce.*shapeE(4,:)] ...
                -[ct.*shapeT(3,:)-st.*shapeT(4,:);st.*shapeT(3,:)+ct.*shapeT(4,:)];
            projection=[ce.*delta(1,:)+se.*delta(2,:);-se.*delta(1,:)+ce.*delta(2,:); ...
                ct.*delta(1,:)+st.*delta(2,:);-st.*delta(1,:)+ct.*delta(2,:)];
            angle=poseE(3,:)-poseT(3,:);
            c=sqrt(cos(angle).^2+smoothing^2);s=sqrt(sin(angle).^2+smoothing^2);
            support=[shapeE(1,:)+shapeT(1,:).*c+shapeT(2,:).*s; ...
                shapeE(2,:)+shapeT(1,:).*s+shapeT(2,:).*c; ...
                shapeT(1,:)+shapeE(1,:).*c+shapeE(2,:).*s; ...
                shapeT(2,:)+shapeE(1,:).*s+shapeE(2,:).*c];
            gaps=sqrt(projection.^2+smoothing^2)-smoothing-support;
            if nargout>1
                % One signed, attaining facet of the penetration distance.
                % At a centered projection either sign attains the same gap.
                [~,facet]=max(gaps,[],1);
                index=sub2ind(size(gaps),facet,1:numel(facet));
                batch=zeros(size(facet));
                ce=ce+batch;se=se+batch;ct=ct+batch;st=st+batch;
                nx=[ce;-se;ct;-st];ny=[se;ce;st;ct];
                activeProjection=projection(index);
                if smoothing==0
                    side=ones(size(facet));side(activeProjection<0)=-1;
                    dc=-sin(angle).*sign(cos(angle));ds=cos(angle).*sign(sin(angle));
                else
                    side=activeProjection./hypot(activeProjection,smoothing);
                    product=sin(angle).*cos(angle);dc=-product./c;ds=product./s;
                end
                normal=side.*[nx(index);ny(index)];
                offsetYaw=[-se.*shapeE(3,:)-ce.*shapeE(4,:);ce.*shapeE(3,:)-se.*shapeE(4,:)];
                % Ego axes rotate with yaw; axes belonging to the target stay fixed.
                projectionYaw=sum(normal.*offsetYaw,1)+(facet<=2).*(normal(1,:).*delta(2,:)-normal(2,:).*delta(1,:));
                dc=dc+batch;ds=ds+batch;
                supportYaw=[shapeT(1,:).*dc+shapeT(2,:).*ds;shapeT(1,:).*ds+shapeT(2,:).*dc; ...
                    shapeE(1,:).*dc+shapeE(2,:).*ds;shapeE(1,:).*ds+shapeE(2,:).*dc];
                gradient=[normal;projectionYaw-supportYaw(index)];
            end
        end

        function [clearance,gradient] = rectangleClearance(states,targets,shape)
            % Batched closest-feature evaluation of the rectangle distance dual.
            if size(states,2)==size(targets,2) ...
                    && size(targets,1)==11 && isequal(size(shape),[4,1]) ...
                    && isa(states,'double') && isa(targets,'double') && isa(shape,'double')
                kernel=localClearanceKernel();
                if ~isempty(kernel)
                    [clearance,gradient]=kernel(states(1:3,:),targets,shape,nargout>1);
                    return
                end
            end
            if nargout>1
                [clearance,gradient]=predictiveSafetyGeometry.rectangleClearanceNumeric(states,targets,shape);
            else
                clearance=predictiveSafetyGeometry.rectangleClearanceNumeric(states,targets,shape);
            end
        end

        function [clearance,gradient] = rectangleClearanceNumeric(states,targets,shape)
            ce=cos(states(3,:));se=sin(states(3,:));ct=cos(targets(3,:));st=sin(targets(3,:));
            signs=[-1,1,1,-1;-1,-1,1,1];ego=cell(1,4);target=cell(1,4);
            for j=1:4
                e=shape(3:4)+shape(1:2).*signs(:,j);
                t=targets(10:11,:)+targets(8:9,:).*signs(:,j);
                ego{j}=states(1:2,:)+[ce*e(1)-se*e(2);se*e(1)+ce*e(2)];
                target{j}=targets(1:2,:)+[ct.*t(1,:)-st.*t(2,:);st.*t(1,:)+ct.*t(2,:)];
            end
            if nargout<2
                % For disjoint rectangles at least one closest pair contains
                % a vertex. Project all eight vertices onto the opposite box.
                % For overlap the signed dual attains a face normal.
                penetration=max(predictiveSafetyGeometry.rectangleAxisGaps(states(1:3,:),shape,targets(1:3,:),targets(8:11,:)),[],1);
                clearance=penetration;
                if ~any(penetration>0),return;end
                squared=Inf(size(clearance));
                for j=1:4
                    d=ego{j}-targets(1:2,:);
                    u=max(0,abs(ct.*d(1,:)+st.*d(2,:)-targets(10,:))-targets(8,:));
                    v=max(0,abs(-st.*d(1,:)+ct.*d(2,:)-targets(11,:))-targets(9,:));
                    squared=min(squared,u.^2+v.^2);
                    d=target{j}-states(1:2,:);
                    u=max(0,abs(ce.*d(1,:)+se.*d(2,:)-shape(3))-shape(1));
                    v=max(0,abs(-se.*d(1,:)+ce.*d(2,:)-shape(4))-shape(2));
                    squared=min(squared,u.^2+v.^2);
                end
                outside=penetration>0;clearance(outside)=sqrt(squared(outside));return
            end
            square=Inf(1,size(states,2));closest=zeros(2,size(states,2));difference=closest;
            for j=1:4
                for k=1:4
                    next=mod(k,4)+1;
                    edge=target{next}-target{k};offset=ego{j}-target{k};
                    t=max(0,min(1,sum(offset.*edge,1)./sum(edge.^2,1)));
                    delta=offset-edge.*t;distance=sum(delta.^2,1);selected=distance<square;
                    square(selected)=distance(selected);
                    if nargout>1,closest(:,selected)=ego{j}(:,selected);difference(:,selected)=delta(:,selected);end
                    edge=ego{next}-ego{k};offset=target{j}-ego{k};
                    t=max(0,min(1,sum(offset.*edge,1)./sum(edge.^2,1)));
                    delta=edge.*t-offset;distance=sum(delta.^2,1);selected=distance<square;
                    square(selected)=distance(selected);
                    if nargout>1
                        point=ego{k}+edge.*t;closest(:,selected)=point(:,selected);difference(:,selected)=delta(:,selected);
                    end
                end
            end
            clearance=sqrt(square);
            [gaps,facetGradient,projection]=predictiveSafetyGeometry.rectangleAxisGaps(states(1:3,:),shape,targets(1:3,:),targets(8:11,:));
            gap=max(gaps,[],1);
            overlap=gap<=0;clearance(overlap)=gap(overlap);
            if nargout>1
                normal=difference./max(sqrt(square),eps);arm=closest-states(1:2,:);
                gradient=[normal;normal(2,:).*arm(1,:)-normal(1,:).*arm(2,:)];
                % A centered difference across |projection| can cancel the two
                % separating normals. Differentiate one attaining facet in that stencil.
                step=1e-5;
                crossesZero=overlap & abs(projection)<=step;
                gradient(:,crossesZero)=facetGradient(:,crossesZero);
                regular=overlap & ~crossesZero;
                if any(regular)
                    x=states(1:3,regular);q=targets(:,regular);
                    for j=1:3
                        plus=x;minus=x;plus(j,:)=plus(j,:)+step;minus(j,:)=minus(j,:)-step;
                        a=max(predictiveSafetyGeometry.rectangleAxisGaps(plus,shape,q(1:3,:),q(8:11,:)),[],1);
                        b=max(predictiveSafetyGeometry.rectangleAxisGaps(minus,shape,q(1:3,:),q(8:11,:)),[],1);
                        gradient(j,regular)=(a-b)/(2*step);
                    end
                end
            end
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

        function [value,jacobian] = separationWitnessRows(poseE,shapeE,poseT,shapeT,witness,reserve,reserveDerivative)
            % A continuous unit normal and a relative plane offset describe
            % ONE nonlinear separating-plane constraint, without angle bins.
            % Columns: ego X, Y, yaw, normal angle, plane offset. The first
            % four rows admit collision slack; the target support rows do not.
            if nargin<6,reserve=0;end
            if nargin<7,reserveDerivative=0;end
            normal=[cos(witness(1));sin(witness(1))];tangent=[-normal(2);normal(1)];
            signs=[-1,1,1,-1;-1,-1,1,1];
            re=[cos(poseE(3)),-sin(poseE(3));sin(poseE(3)),cos(poseE(3))];
            rt=[cos(poseT(3)),-sin(poseT(3));sin(poseT(3)),cos(poseT(3))];
            body=shapeE(3:4)+shapeE(1:2).*signs;
            ego=poseE(1:2)-poseT(1:2)+re*body;
            target=rt*(shapeT(3:4)+shapeT(1:2).*signs);
            value=[(normal.'*ego-witness(2)-reserve).'; ...
                (witness(2)-normal.'*target).'];
            jacobian=zeros(8,5);
            jacobian(1:4,1:2)=repmat(normal.',4,1);
            jacobian(1:4,3)=(normal.'*re*[0,-1;1,0]*body).';
            jacobian(1:4,4)=(tangent.'*ego-reserveDerivative).';
            jacobian(1:4,5)=-1;
            jacobian(5:8,4)=-(tangent.'*target).';jacobian(5:8,5)=1;
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

        function curve = validateReferenceCurve(curve)
            required = ["origin", "heading", "curvature", "length"];
            if ~isstruct(curve) || ~isscalar(curve) || ~all(isfield(curve, required))
                error("collisionAvoidanceController:invalidReferenceCurve", ...
                    "A reference curve needs origin, heading, curvature and length.");
            end
            validateattributes(curve.origin, {'double'}, {'real','finite','numel',2});
            validateattributes(curve.heading, {'double'}, {'real','finite','scalar'});
            validateattributes(curve.curvature, {'double'}, {'real','finite','scalar'});
            validateattributes(curve.length, {'double'}, {'real','finite','scalar','positive'});
            if isfield(curve,'curvatureProfile') && ~isempty(curve.curvatureProfile)
                error('collisionAvoidanceController:unsupportedReference','Use a straight or constant-curvature reference.');
            end
            if abs(curve.curvature)*curve.length >= 2*pi
                error("collisionAvoidanceController:invalidReferenceCurve", ...
                    "The finite arc must have less than one complete revolution.");
            end
            curve.origin = curve.origin(:);
        end

        function [position, heading] = referencePose(station, lateral, curve)
            heading = curve.heading+curve.curvature*station;
            if curve.curvature == 0
                position = curve.origin+[cos(curve.heading);sin(curve.heading)]*station;
            else
                position = curve.origin+[sin(heading)-sin(curve.heading); ...
                    cos(curve.heading)-cos(heading)]/curve.curvature;
            end
            position = position+[-sin(heading);cos(heading)].*lateral;
        end

        function projection = projectReferenceCurve(position, curve, stationHint)
            if nargin<3,stationHint=[];end
            if curve.curvature == 0
                station = [cos(curve.heading),sin(curve.heading)]*(position-curve.origin);
            else
                center = curve.origin+[-sin(curve.heading);cos(curve.heading)]/curve.curvature;
                radial = position-center;
                heading = atan2(curve.curvature*radial(1,:),-curve.curvature*radial(2,:));
                middle = curve.heading+curve.curvature*curve.length/2;
                station = curve.length/2+atan2(sin(heading-middle),cos(heading-middle))/curve.curvature;
                if ~isempty(stationHint)
                    period=2*pi/abs(curve.curvature);
                    station=station+period*round((stationHint-station)/period);
                end
            end
            % The analytic reference continues beyond the display arc. On a
            % circle the optional station hint unwraps the projected coordinate.
            [point,heading] = predictiveSafetyGeometry.referencePose(station,0,curve);
            lateral = sum([-sin(heading);cos(heading)].*(position-point),1);
            projection = struct("point",point,"heading",heading, ...
                "station",station,"lateralPosition",lateral);
        end

        function projection = project(position, lane, stationHint)
            if nargin<3,stationHint=[];end
        % predictiveSafetyGeometry.project Project one point or columns of points onto a lane polyline.
        %
        % lane is the parsed centerline structure (segment starts, segments,
        % lengths, unit tangents, per-segment curvature and the cumulative
        % station of every segment start). Returns the closest polyline point
        % with the path heading there, the STATION s of the projection along
        % the centerline, and the signed left-positive lateral offset d - the
        % path coordinates (s, d) of the point.

            if isfield(lane, "referenceCurve")
                projection = predictiveSafetyGeometry.projectReferenceCurve(reshape(position,2,[]),lane.referenceCurve,stationHint);
                return;
            end
            if all(abs(lane.tangent-lane.tangent(1,:))<1e-12,'all')
                tangent=lane.tangent(1,:).';lateral=[-tangent(2);tangent(1)];
                station=tangent.'*(reshape(position,2,[])-lane.segmentStart(1,:).');
                point=lane.segmentStart(1,:).'+tangent*station;
                projection=struct('point',point,'heading',atan2(tangent(2),tangent(1))+zeros(size(station)), ...
                    'station',station,'lateralPosition',lateral.'*(reshape(position,2,[])-point));
                return;
            end
            position=reshape(position,2,[]);values=zeros(5,size(position,2));
            for pointIdx=1:size(position,2)
                values(:,pointIdx)=localProjection(position(:,pointIdx),lane);
            end
            projection = struct("point", values(1:2, :), "heading", values(3, :), ...
                "station", values(4, :), "lateralPosition", values(5, :));
        end



    end
end

function [lo,hi]=localCosRange(a,b)
    lo=min(cos(a),cos(b));hi=max(cos(a),cos(b));
    hi(ceil(a/(2*pi))<=floor(b/(2*pi)))=1;
    lo(ceil((a-pi)/(2*pi))<=floor((b-pi)/(2*pi)))=-1;
end

function [interval,available]=localCourseContinuation(memory,history,time,curvature)
    % Transport target course in the EGO frame with measured gyro increments.
    % The common absolute heading error cancels. Intersecting this prediction
    % with the new observer set prevents a weaker fresh orientation estimate
    % from discarding valid information retained from earlier measurements.
    interval=[];available=false;
    required={'records','gyroscopeNoise','yawAccelerationMaximum'};
    if ~all(isfield(history,required)) || isempty(history.records),return;end
    times=[history.records.time];guard=128*eps(max(1,abs(time)));
    first=find(abs(times-memory.time)<=guard,1);last=find(abs(times-time)<=guard,1);
    if isempty(first) || isempty(last) || first>=last,return;end
    indices=first:last;dt=diff(times(indices));gyro=[history.records(indices).yawRate];
    turn=sum((gyro(1:end-1)+gyro(2:end)).*dt/2);
    acceleration=repmat(history.yawAccelerationMaximum,size(dt));
    if isfield(history,'holdYawAcceleration') && numel(history.holdYawAcceleration)==numel(times)
        acceleration=min(acceleration,history.holdYawAcceleration(indices(1:end-1)));
    end
    radius=history.gyroscopeNoise*sum(dt)+sum(acceleration.*dt.^2/4);
    if ~isfinite(radius),return;end
    elapsed=time-memory.time;arc=zeros(2,1);
    for k=1:2
        v=memory.speedInterval(k);a=memory.accelerationInterval(k);duration=elapsed;
        if a<0,duration=min(duration,-v/a);end
        arc(k)=v*duration+.5*a*duration^2;
    end
    turns=curvature(:)*arc(:).';target=[min(turns,[],'all');max(turns,[],'all')];
    interval=memory.relativeCourseInterval+target-turn+[-radius;radius];available=true;
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

function kernel=localClearanceKernel()
    % The build compiles this same dual evaluation one pose pair at a time.
    persistent handle checked
    if isempty(checked)
        checked=true;handle=[];
        folder=fullfile(fileparts(fileparts(mfilename('fullpath'))),'solver','controller');
        if isfile(fullfile(folder,['clearanceBatchKernelMex.',mexext]))
            previous=addpath(folder);cleanup=onCleanup(@()path(previous));
            candidate=@clearanceBatchKernelMex;
            shape=[2.4;.95;.3;-.1];
            x=[0,0,1;0,0,1;0,0,.3];
            q=zeros(11,3);q(1:3,:)=[1,10,6;.1,-3,2;0,0,-.2];q(8:11,:)=repmat(shape,1,3);
            [expected,gradient]=predictiveSafetyGeometry.rectangleClearanceNumeric(x,q,shape);
            values=predictiveSafetyGeometry.rectangleClearanceNumeric(x,q,shape);
            try
                [d,g]=candidate(x,q,shape,true);
                if isequal(d,expected) && isequal(g,gradient) && isequal(candidate(x,q,shape,false),values)
                    handle=candidate;
                end
            catch
                % An incompatible binary leaves the MATLAB implementation active.
            end
        end
    end
    kernel=handle;
end

function kernel=localKernel()
    % Load the distance kernel for geometric evaluations.
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

function candidate=localCirculationRollout(model,data,speed,durations,collectDiagnostics)
    cfg=model.cfg;h=cfg.controller.sampleTime;count=numel(durations);
    candidate=struct('states',zeros(6,count+1),'inputs',zeros(2,count), ...
        'gaps',zeros(1,count+1),'nominalGaps',zeros(1,count+1), ...
        'speedScale',speed, ...
        'releaseNode',NaN,'stateViolation',0,'fieldActive',false, ...
        'maximumDeviation',0,'completion',"requestedHorizon");
    candidate.states(:,1)=model.initialState;
    hasCover=isfield(model,'motionCover') && ~isempty(model.motionCover);
    nodes=1+round([0,cumsum(durations)]/h);
    if hasCover
        % Backward differences give the local motion of the occupied rectangle.
        boxes=model.motionCover.boxes(:,nodes);angles=boxes(3,:);
        centers=boxes(1:2,:)+[cos(angles).*boxes(10,:)-sin(angles).*boxes(11,:); ...
            sin(angles).*boxes(10,:)+cos(angles).*boxes(11,:)];
        centerVelocity=diff(centers,1,2)./durations;
        extentRate=diff(boxes(8:9,:),1,2)./durations;
    end
    prior=model.previousInput;encounterEnded=false;
    for index=1:count+1
        if mod(index,8)==1,localPlannerDeadline(model);end
        x=candidate.states(:,index);target=data.target(:,index);
        if hasCover
            node=nodes(index);
            target=boxes(:,index);
            range=predictiveSafetyGeometry.targetEnvelopeRange(x,target,model.motionCover.centers(:,node));
            outside=range>cfg.collision.encounterRangeMeters;
        else
            outside=norm(x(1:2)-target(1:2))-data.reserve-data.support.ballRadius(index)>cfg.collision.encounterRangeMeters;
        end
        if index>1 && outside && norm(model.initialState(1:2)-model.target(1:2))<=cfg.collision.encounterRangeMeters
            encounterEnded=true;
        end
        % The route attracts the seed; road boundaries also shape it.
        projection=predictiveSafetyGeometry.project(x(1:2),model.lane);
        if collectDiagnostics
            if encounterEnded
                gap=Inf;nominal=Inf;
            else
                [gap,nominal]=localPlannerGap(x,target,data,index);
            end
            candidate.gaps(index)=gap;candidate.nominalGaps(index)=nominal;
            candidate.stateViolation=max(candidate.stateViolation,localPlannerStateViolation(x,model));
            candidate.maximumDeviation=max(candidate.maximumDeviation,abs(projection.lateralPosition));
        end
        if index>count,break;end
        % Route attraction is the preferred acceleration BEFORE obstacle
        % and boundary correction. Never overwrite longitudinal obstacle
        % acceleration with a cruise-speed request afterwards.
        acceleration=predictiveSafetyGeometry.routeAcceleration(x,projection,model);
        if ~encounterEnded
            parameters=localFieldParameters(data,index,speed);
            if hasCover && index>1 && target(4)~=0
                % Project the target footprint across the cruise path. Its length
                % contributes transverse drift in proportion to the width it spans.
                heading=target(3)+target(6)-projection.heading;
                lengthAcross=data.target(8,index)*abs(sin(heading));
                widthAcross=data.target(9,index)*abs(cos(heading));
                crossWidth=min(1,max(0,(lengthAcross-widthAcross)/data.target(9,index)));
                % Scale the drift by the local field response over one target
                % half-width. A braking body can slow as its occupied box grows.
                responseAcceleration=cfg.planner.radialRate^2*data.target(9,index);
                speedRate=sign(target(4))*target(5);
                transportFraction=min(1,max(0,1+speedRate/responseAcceleration));
                parameters.crossWeight=crossWidth*transportFraction;
                parameters.centerVelocity=centerVelocity(:,index-1);
                parameters.extentRate=extentRate(:,index-1);
            end
            parameters.preferredAcceleration=acceleration;
            [acceleration,field]=predictiveSafetyGeometry.circulationField(x,target,cfg,parameters);
            active=field.activation>0;
            candidate.fieldActive=candidate.fieldActive || active;
            if ~active && candidate.fieldActive && isnan(candidate.releaseNode),candidate.releaseNode=index-1;end
        end
        if ~isfield(model.road,'boundaryObservation')
            acceleration=predictiveSafetyGeometry.wallGuidance(x,acceleration,model);
        end
        if isfield(model.road,'boundaryObservation')
            acceleration=predictiveSafetyGeometry.roadBoundaryGuidance(x,acceleration,model);
        end
        u=nonlinearBicycleModel.accelerationGuidance(x,acceleration,prior,cfg,1,true,true);
        u(1)=min(cfg.outputFeedback.steeringMaximum,max(-cfg.outputFeedback.steeringMaximum,u(1)));
        candidate.inputs(:,index)=u;
        if index==count && ~collectDiagnostics,break;end
        next=nonlinearBicycleModel.sample(x,u,cfg,[],durations(index));
        candidate.states(:,index+1)=next;prior=u;
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
        projection=predictiveSafetyGeometry.project(x(1:2)+rotation*body,model.lane);
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
        error('collisionAvoidanceController:timeBudgetExceeded','Circulation initialization exhausted its time budget.');
    end
end


function values = localProjection(position, lane)
    offset = position(:).'-lane.segmentStart;
    fraction = sum(offset.*lane.segment, 2)./lane.segmentLength.^2;
    fraction = min(max(fraction, 0.0), 1.0);
    projectedPoint = lane.segmentStart+fraction.*lane.segment;
    delta = position(:).'-projectedPoint;
    distanceSquared = sum(delta.^2, 2);
    [~, segmentIdx] = min(distanceSquared);
    tangent = lane.tangent(segmentIdx, :);
    selectedDelta = delta(segmentIdx, :);
    values = [projectedPoint(segmentIdx, :).'; atan2(tangent(2), tangent(1)); ...
        lane.segmentStation(segmentIdx)+fraction(segmentIdx)*lane.segmentLength(segmentIdx); ...
        tangent(1)*selectedDelta(2)-tangent(2)*selectedDelta(1)];
end
