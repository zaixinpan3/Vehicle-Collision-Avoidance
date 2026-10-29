classdef nonlinearBicycleModel
    %nonlinearBicycleModel Nominal held-input prediction, tangents and lane trims.
    % RK4 and its variational equations define the numerical prediction model.
    methods (Static)
        function [lower,upper] = inputBounds(cfg,linearizationPoint)
            % Physical saturation includes +/-1. Only tangent seeds need an
            % interior point because the Fiala input slope is singular there.
            if nargin<2,linearizationPoint=false;end
            reserve=0;
            if linearizationPoint,reserve=10*cfg.solver.constraintTolerance;end
            lower=[-Inf;max(-1+reserve,cfg.actuation.brakingRatioMinimum)];
            upper=[Inf;min(1-reserve,cfg.actuation.brakingRatioMaximum)];
        end

        function dx = derivative(x,u,cfg,curvature)
            % The declared model: the Fiala bicycle above
            % model.kinematicSpeedRange, a kinematic bicycle below it, and a
            % smooth blend between (localBody); position and heading follow
            % the body velocities in all three. Above the range it is the
            % Fiala bicycle exactly. Braking ends at rest (exactly at the end of
            % a hold, in sample).
            if nargin<4,curvature=[];end
            if x(4)<-cfg.model.brakeHoldSpeed || abs(u(2))>1
                error('collisionAvoidanceController:nonlinearDomain', ...
                    'The declared model requires forward travel and |b|<=1.');
            end
            v=x(4);
            dx=[v*cos(x(3))-x(5)*sin(x(3));v*sin(x(3))+x(5)*cos(x(3));x(6);localBody(x,u,cfg)];
            if ~isempty(curvature)
                denominator=1-curvature*x(2);
                if denominator<=0,error('collisionAvoidanceController:nonlinearDomain','Invalid Frenet chart.');end
                dx(1)=dx(1)/denominator;dx(3)=dx(3)-curvature*dx(1);
            end
        end

        function count = meshCount(cfg)
            % RK4 steps per hold: one when integrationStep covers the hold,
            % otherwise an even number so that the midpoint is a mesh node.
            h=cfg.controller.sampleTime;count=1;
            if cfg.nonlinear.integrationStep<h,count=2*max(1,ceil(h/(2*cfg.nonlinear.integrationStep)));end
        end

        function [next,a,b] = sample(x,u,cfg,curvature,duration)
            if nargin<4,curvature=[];end
            if nargin<5,duration=cfg.controller.sampleTime;end
            variational=nargout>1;
            if coder.target('MATLAB') && isempty(curvature)
                % The compiled copy of this method; see scripts/buildControllerKernels.m.
                kernel=localKernel(cfg);
                if ~isempty(kernel)
                    [next,a,b]=kernel(x,u,localKernelParameters(cfg),duration,variational);
                    return;
                end
            end
            y=localIntegrate(x,u,cfg,curvature,duration,variational);
            next=y(1:6);
            if variational,a=reshape(y(7:42),6,6);b=reshape(y(43:54),6,2);end
        end

        function [radius,info] = trustRegion(x,u,cfg,next,a,b,requiredState)
            % Numerical, state-dependent locality estimate for the hold map.
            % Translation is exactly affine and receives no trust bound.
            % Finite differences seed the radii from mixed curvature; this
            % is not a certified Hessian bound across model transitions.
            % An optional affine consistency correction reserves state room.
            % Reallocate the SAME error budget by shrinking the other axes;
            % the required correction is not a relaxation of model accuracy.
            if nargin<7,requiredState=zeros(6,1);end
            minimum=[abs(requiredState(3:6));zeros(2,1)];
            q=[x;u];active=3:8;
            units=[.3;max(2,.25*abs(x(4)));1;.4;.25;.5];
            steps=1e-3*units;
            steps(6)=min(steps(6),max(1e-8,(1-abs(u(2)))/2));
            jac=[a,b];curvature=zeros(6,6,6);
            for j=1:6
                plus=q;minus=q;plus(active(j))=plus(active(j))+steps(j);
                minus(active(j))=minus(active(j))-steps(j);
                [~,ap,bp]=nonlinearBicycleModel.sample(plus(1:6),plus(7:8),cfg);
                [~,am,bm]=nonlinearBicycleModel.sample(minus(1:6),minus(7:8),cfg);
                difference=([ap,bp]-[am,bm])/(2*steps(j));
                curvature(:,:,j)=difference(:,active).*units.'*units(j);
            end
            budget=cfg.rti.trustRegion.errorBudget;
            % For |d_i| <= r_i, sum |H_ij| r_i r_j bounds the
            % quadratic surrogate, including mixed state/input terms.
            curvature=(curvature+permute(curvature,[1,3,2]))/2;
            envelope=abs(curvature)./reshape(budget,6,1,1);
            weights=squeeze(max(sum(envelope,3),[],1));
            scaled=min(1,sqrt(2./max(6*weights,1e-12)));
            radii=max(minimum,units.*scaled(:));
            % Probe finite displacements too: a zero local curvature can
            % otherwise miss a nearby saturation or model transition.
            signs=[eye(6),-eye(6),ones(6,1),-ones(6,1), ...
                [1;-1;1;-1;1;-1],[-1;1;-1;1;-1;1]];
            if any(minimum)
                % A required correction can cross a saturation transition.
                % Test all mixed corners instead of treating the reference
                % Hessian's absolute envelope as a finite-region bound.
                corners=2*(dec2bin(0:63,6)-'0')-1;
                signs=[eye(6),-eye(6),corners.'];
            end
            ratio=0;
            for attempt=1:6
                ratio=0;
                for j=1:size(signs,2)
                    change=zeros(8,1);change(active)=radii.*signs(:,j);
                    trial=q+change;trial(8)=min(1,max(-1,trial(8)));
                    change=trial-q;
                    try
                        actual=nonlinearBicycleModel.sample(trial(1:6),trial(7:8),cfg);
                        ratio=max(ratio,max(abs(actual-next-jac*change)./budget));
                    catch exception
                        if ~ismember(string(exception.identifier),[ ...
                                "collisionAvoidanceController:nonlinearDomain", ...
                                "collisionAvoidanceController:invalidTireOperatingPoint"])
                            rethrow(exception);
                        end
                        ratio=Inf;break;
                    end
                end
                if ratio<=1,break;end
                if attempt==6
                    error('collisionAvoidanceController:trustRegionUnavailable', ...
                        'No local box passed the finite-displacement model probes.');
                end
                radii=minimum+(radii-minimum)*max(.1,min(.8,.8/sqrt(ratio)));
            end
            radius=[Inf;Inf;radii];
            info=struct('sampledErrorRatio',ratio,'probePasses',attempt);
        end

        function [next,stopped] = sampleEndState(x,u,cfg)
            % Common discrete rest transition after prediction or plant replay.
            % Continuous integration precedes this reset. Position and heading
            % retain their integrated values; driving inputs do not trigger it.
            next=x;stopped=u(2)<=0 && x(4)<.2*cfg.model.brakeHoldSpeed;
            if stopped,next(4:6)=0;end
        end

        function [middle,am,bm,next,a,b] = hold(x,u,cfg)
            % One hold and its midpoint with their tangents. A one-step mesh
            % has no midpoint node; its midpoint is the linear interpolant
            % that the interval certificate also uses between mesh nodes.
            half=cfg.controller.sampleTime/2;
            if nonlinearBicycleModel.meshCount(cfg)==1
                if nargout>1,[next,a,b]=nonlinearBicycleModel.sample(x,u,cfg);am=(eye(6)+a)/2;bm=b/2;
                else,next=nonlinearBicycleModel.sample(x,u,cfg);
                end
                middle=(x+next)/2;
            elseif nargout>1
                [middle,am,bm]=nonlinearBicycleModel.sample(x,u,cfg,[],half);
                [next,an,bn]=nonlinearBicycleModel.sample(middle,u,cfg,[],half);
                a=an*am;b=an*bm+bn;
            else
                middle=nonlinearBicycleModel.sample(x,u,cfg,[],half);
                next=nonlinearBicycleModel.sample(middle,u,cfg,[],half);
            end
        end

        function [next,a,b] = jointSample(z,u,targetParameters,cfg)
            % Joint state [ego(6); target position(2); target yaw; target V].
            % Constant target parameters: [A; beta; lr; half extents; offset].
            % Eliminating the autonomous target prediction is exact.
            if isempty(targetParameters)
                if nargout>1,[next,a,b]=nonlinearBicycleModel.sample(z,u,cfg);
                else,next=nonlinearBicycleModel.sample(z,u,cfg);end
                return;
            end
            validateattributes(z,{'double'},{'size',[10,1],'real','finite'});
            validateattributes(targetParameters,{'double'},{'size',[7,1],'real','finite'});
            if nargout>1,[ego,ae,be]=nonlinearBicycleModel.sample(z(1:6),u,cfg);
            else,ego=nonlinearBicycleModel.sample(z(1:6),u,cfg);end
            target=[z(7:10);targetParameters];
            future=predictiveSafetyGeometry.predictTarget(target,cfg.controller.sampleTime);
            displacement=future(1:2)-target(1:2);
            at=eye(4);at(1:2,3)=[-displacement(2);displacement(1)];
            at(1:2,4)=cfg.controller.sampleTime*[cos(future(3)+target(6));sin(future(3)+target(6))];
            at(3,4)=cfg.controller.sampleTime*sin(target(6))/target(7);
            next=[ego;future(1:4)];
            if nargout>1,a=blkdiag(ae,at);b=[be;zeros(4,2)];end
        end

        function [error,jacobian] = errorLinearization(x,lane,reference)
            [error,projection]=nonlinearBicycleModel.error(x,lane,reference);
            tangent=[cos(projection.heading);sin(projection.heading)];
            jacobian=zeros(5,6);jacobian(1,1:2)=[-tangent(2),tangent(1)];
            jacobian(2,3)=1;
            jacobian(2,1:2)=-reference.curvature*tangent.'/(1-reference.curvature*projection.lateralPosition);
            jacobian(3:5,4:6)=eye(3);
        end

        function [rate,inputGradient,value] = nominalDerivative(x,u,lane,reference,cfg)
            % Exact continuous-time derivative of V=e'Pe at the current
            % state. Only its dependence on the actual Fiala inputs is
            % linearized by RTI; the error chart is differentiated exactly.
            [e,jacobian]=nonlinearBicycleModel.errorLinearization(x,lane,reference);
            gradient=2*e.'*reference.matrix*jacobian;
            rate=gradient*nonlinearBicycleModel.derivative(x,u,cfg);
            if nargout>1
                [~,b]=nonlinearBicycleModel.jacobian(x,u,cfg);
                inputGradient=gradient*b;
            end
            if nargout>2,value=sum((reference.factor*e).^2);end
        end

        function [a,b] = jacobian(x,u,cfg,curvature)
            % Analytic derivatives of derivative: combined-slip tire and
            % body forces above the kinematic speed range, the kinematic
            % bicycle below it, and between them the blend with the slope of
            % its weight.
            if nargin<4,curvature=[];end
            [weight,slope]=localBlend(x(4),cfg);
            if weight==1
                [a,b]=localFialaJacobian(x,u,cfg);
            else
                [a,b]=localKinematicJacobian(x,u,cfg);
                if weight>0
                    [af,bf]=localFialaJacobian(x,u,cfg);
                    a(4:6,:)=weight*af(4:6,:)+(1-weight)*a(4:6,:);
                    b(4:6,:)=weight*bf(4:6,:)+(1-weight)*b(4:6,:);
                    a(4:6,4)=a(4:6,4)+slope*(localFialaBody(x,u,cfg)-localKinematicBody(x,u,cfg));
                end
            end
            if ~isempty(curvature)
                cp=cos(x(3));sp=sin(x(3));
                denominator=1-curvature*x(2);
                forward=x(4)*cp-x(5)*sp;
                a(1,:)=a(1,:)/denominator;
                a(1,2)=a(1,2)+curvature*forward/denominator^2;
                a(3,:)=a(3,:)-curvature*a(1,:);
            end
        end

        function gradient = parameterJacobian(x,u,cfg)
            % Fractional tire-parameter tangent of the SAME blended plant.
            % Order: C_f,C_r,mu_f,mu_r. No additional tire constraint is used.
            weight=localBlend(x(4),cfg);gradient=zeros(6,4);
            scale=modifiedFialaTire.parameters(cfg).longitudinalForceScale;
            longitudinal=[zeros(2),diag(u(2)*scale)];
            c=cos(u(1));s=sin(u(1));body=zeros(3,4);
            if weight>0
                slip=atan2([x(5)+cfg.vehicle.lf*x(6);x(5)-cfg.vehicle.lr*x(6)],x(4))-[u(1);0];
                lateral=modifiedFialaTire.parameterJacobian(slip,u(2),cfg);
                frontX=c*longitudinal(1,:)-s*lateral(1,:);
                frontY=s*longitudinal(1,:)+c*lateral(1,:);
                body=weight*[(frontX+longitudinal(2,:))/cfg.vehicle.m; ...
                    (frontY+lateral(2,:))/cfg.vehicle.m; ...
                    (cfg.vehicle.lf*frontY-cfg.vehicle.lr*lateral(2,:))/cfg.vehicle.Iz];
            end
            if weight<1
                gain=max(u(2),0)+min(u(2),0)*tanh(x(4)/cfg.model.brakeHoldSpeed);
                body(1,3:4)=body(1,3:4)+(1-weight)*gain*[c*scale(1),scale(2)]/cfg.vehicle.m;
            end
            gradient(4:6,:)=body;
        end

        function [force, slope, components] = roadLoad(speed, cfg)
        %nonlinearBicycleModel.roadLoad Signed passive road load and its speed derivative.
        % Flat road, still air, and a quasi-static equivalent rolling force are
        % assumed. Positive force opposes forward travel. The smooth rolling sign
        % preserves rest without applying a constant backward force at zero speed.
        % Polynomial rolling coefficients have units 1, s/m, and (s/m)^4.
        % Wheel slip, wheel inertia, camber, and dynamic normal-load effects are
        % residual dynamics, not reproduced by this reduced road-load model.

            roadLoad = cfg.roadLoad;
            magnitude = abs(speed);
            direction = tanh(speed/roadLoad.rollingTransitionSpeed);
            rollingCoefficient = roadLoad.rollingCoefficient ...
                + roadLoad.rollingSpeedCoefficient*magnitude ...
                + roadLoad.rollingQuarticCoefficient*magnitude.^4;
            aerodynamicFactor = 0.5*roadLoad.airDensity ...
                * roadLoad.dragCoefficient*roadLoad.frontalArea;
            aerodynamic = aerodynamicFactor*speed.*magnitude;
            rollingScale = cfg.vehicle.m*cfg.vehicle.gravity;
            rolling = rollingScale*rollingCoefficient.*direction;
            force = aerodynamic+rolling;
            if nargout > 1
                slope = 2*aerodynamicFactor*magnitude ...
                    + rollingScale*((roadLoad.rollingSpeedCoefficient ...
                        + 4*roadLoad.rollingQuarticCoefficient*magnitude.^3) ...
                        .*sign(speed).*direction ...
                        + rollingCoefficient.*(1-direction.^2)/roadLoad.rollingTransitionSpeed);
            end
            if nargout > 2
                components = struct("aerodynamicForce", aerodynamic, ...
                    "rollingResistanceForce", rolling);
            end
        end

        function reference = cruise(cfg,curvature)
            % Trim, continuous CLF matrix P and sampled cruise certificate. P is
            % never computed here: it is read from config/clfMatrices.json,
            % which scripts/synthesizeClfMatrices.m writes before experiments.
            persistent savedKeys savedReferences
            if isempty(savedKeys),savedKeys={};savedReferences={};end
            % Character keys: strcmp does not match string objects in a cell,
            % so a string key never hit this cache and every call re-solved
            % the trim.
            key=char(nonlinearBicycleModel.clfKey(cfg,curvature));
            index=find(strcmp(savedKeys,key),1);
            if ~isempty(index),reference=savedReferences{index};return;end
            point=nonlinearBicycleModel.operatingPoint(cfg,curvature);
            entry=localClfEntry(key);
            p=reshape(entry.matrix,5,5);p=(p+p.')/2;
            % The cruise controller's gain and its nonlinear certificate
            % (certified level, hold factor) are synthesized with P.
            reference=struct('state',point.state,'input',point.input,'curvature',curvature, ...
                'matrix',p,'factor',chol(p),'contraction',exp(-2*cfg.controller.sampleTime/cfg.clf.convergenceTimeConstantSeconds), ...
                'decayRate',2/cfg.clf.convergenceTimeConstantSeconds, ...
                'continuousLinearDecayRate',entry.continuousLinearDecayRate, ...
                'certifiedContraction',entry.certifiedContraction, ...
                'gain',reshape(entry.gain,2,5),'certifiedLevel',entry.certifiedLevel,'holdFactor',entry.holdFactor, ...
                'nominalA',reshape(entry.nominalA,5,5),'nominalB',reshape(entry.nominalB,5,2), ...
                'continuousA',point.continuousA,'continuousB',point.continuousB);
            savedKeys=[{key},savedKeys(1:min(end,15))];savedReferences=[{reference},savedReferences(1:min(end,15))];
        end

        function point = operatingPoint(cfg,curvature)
            % Constant-speed, constant-curvature trim and its sampled
            % transverse model e+ = A e + B (u - u_ref).
            speed=cfg.referenceSpeed;
            % Newton's method on [sideslip; steering; braking ratio] with the
            % analytic tangent (no Optimization Toolbox start-up in the
            % controller's first sample); fsolve only if it does not converge.
            [solution,residual,flag]=localTrimNewton([0;atan(cfg.vehicle.wheelbase*curvature);.02],speed,curvature,cfg);
            if flag<=0
                objective=@(p)localTrimResidual(p,speed,curvature,cfg);
                options=optimoptions('fsolve','Display','off','FunctionTolerance',1e-12, ...
                    'StepTolerance',1e-12,'OptimalityTolerance',1e-12);
                [solution,residual,flag]=fsolve(objective,[0;atan(cfg.vehicle.wheelbase*curvature);.02],options);
            end
            if flag<=0 || norm(residual,inf)>1e-8
                error('collisionAvoidanceController:unrealizableReference','No positive-speed constant-curvature trim was found.');
            end
            [~,x,u]=localTrimResidual(solution,speed,curvature,cfg);
            [a,b]=nonlinearBicycleModel.jacobian(x,u,cfg,curvature);
            transition=expm([a(2:6,2:6),b(2:6,:);zeros(2,7)]*cfg.controller.sampleTime);
            point=struct('state',x,'input',u,'a',transition(1:5,1:5),'b',transition(1:5,6:7), ...
                'continuousA',a(2:6,2:6),'continuousB',b(2:6,:));
        end

        function key = clfKey(cfg,curvature)
            % Everything the synthesized P depends on, as one canonical text.
            % The controller's state limits (cfg.model) do not enter P.
            data=struct('vehicle',cfg.vehicle,'tire',cfg.tire,'roadLoad',cfg.roadLoad, ...
                'referenceSpeed',cfg.referenceSpeed,'sampleTime',cfg.controller.sampleTime, ...
                'integrationStep',cfg.nonlinear.integrationStep,'clf',cfg.clf,'curvature',curvature);
            key=string(jsonencode(data));
        end

        function [error,projection] = error(x,lane,reference)
            projection=laneGeometry.project(x(1:2),lane);
            angle=atan2(sin(x(3)-projection.heading-reference.state(3)), ...
                cos(x(3)-projection.heading-reference.state(3)));
            error=[projection.lateralPosition;angle;x(4:6)-reference.state(4:6)];
        end

        function u = nominalFeedback(x,previous,lane,reference,cfg,terminal)
            % Path guidance used only to construct an initialization (CONTROLLER_THEORY.md, Section 4).
            % Course guidance chi_d=-atan(e_y/D) toward the path, a course loop to a
            % yaw-rate demand within the lateral-friction limit, the front slip angle
            % from the inverse Fiala curve below its force peak, and a speed loop on
            % the braking ratio. terminal.steeringOffset makes the trim an exact
            % equilibrium (the inverse neglects the front longitudinal force).
            p=cfg.nominalClf;projection=laneGeometry.project(x(1:2),lane);
            lateral=projection.lateralPosition;curvature=reference.curvature;
            speed=hypot(x(4),x(5));course=localWrap(x(3)+atan2(x(5),x(4))-projection.heading);
            lookahead=max(p.minimumLookaheadMeters,p.lookaheadSeconds*hypot(reference.state(4),reference.state(5)));
            desired=-atan(lateral/lookahead);desiredRate=-lookahead/(lookahead^2+lateral^2)*speed*sin(course);
            yawRate=curvature*speed*cos(course)/max(1-curvature*lateral,.1)+desiredRate ...
                -p.courseGain*localWrap(course-desired);
            tire=modifiedFialaTire.parameters(cfg);
            limit=p.lateralAccelerationFraction*min(tire.frictionCoefficient)*cfg.vehicle.gravity/max(speed,1);
            yawRate=min(limit,max(-limit,yawRate));
            b=reference.input(2)+p.speedGain*(reference.state(4)-x(4));
            b=min(p.brakingRatioLimit,max(-p.brakingRatioLimit,b));
            rate=cfg.model.brakingRatioRateMaximum*cfg.controller.sampleTime;
            if isfinite(rate),b=min(previous(2)+rate,max(previous(2)-rate,b));end
            [lower,upper]=nonlinearBicycleModel.inputBounds(cfg,true);
            b=min(upper(2),max(lower(2),b));
            rear=modifiedFialaTire.evaluate([0;atan2(x(5)-cfg.vehicle.lr*x(6),x(4))],b,cfg);
            front=(cfg.vehicle.Iz*p.yawRateGain*(yawRate-x(6))+cfg.vehicle.lr*rear(2))/cfg.vehicle.lf;
            capacity=tire.longitudinalForceScale(1)*sqrt(1-b^2);
            fraction=min(abs(front)/capacity,p.frontForceFraction);
            slip=-sign(front)*atan(3*capacity*(1-(1-fraction)^(1/3))/tire.corneringStiffness(1));
            u=[atan2(x(5)+cfg.vehicle.lf*x(6),x(4))-slip+terminal.steeringOffset;b];
        end

        function u = accelerationGuidance(x,acceleration,previous,cfg,longitudinalWeight,coupled)
            % Map a virtual world acceleration to initialization inputs using
            % the existing yaw-rate loop and inverse Fiala branch. This is
            % not an affine-input barrier projection or an execution filter.
            if nargin<5,longitudinalWeight=1;end
            if nargin<6,coupled=false;end
            speed=hypot(x(4),x(5));course=x(3)+atan2(x(5),x(4));
            forward=[cos(course);sin(course)];lateral=[-forward(2);forward(1)];
            tire=modifiedFialaTire.parameters(cfg);p=cfg.nominalClf;
            desiredYaw=(lateral.'*acceleration)/max(speed,1);
            limit=cfg.planner.lateralAccelerationFraction*min(tire.frictionCoefficient)*cfg.vehicle.gravity/max(speed,1);
            % Slow turns must remain reachable in the same sideslip domain as
            % the RTI model; a friction-only yaw demand grows as speed falls.
            kinematicLimit=.8*tan(cfg.model.sideslipMaximum)*max(x(4),0)/cfg.vehicle.lr;
            limit=min(limit,kinematicLimit);
            desiredYaw=min(limit,max(-limit,desiredYaw));
            longitudinal=longitudinalWeight*(forward.'*acceleration);
            longitudinal=min(longitudinal,(cfg.model.speedMaximum-x(4))/cfg.controller.sampleTime);
            b=(longitudinal+nonlinearBicycleModel.roadLoad(x(4),cfg)/cfg.vehicle.m)/modifiedFialaTire.accelerationGain(cfg);
            % Allocate braking and lateral force together. The cruise seed's
            % brakingRatioLimit is not an avoidance actuator bound. Reserve
            % the lateral fraction requested by the yaw loop at BOTH axles,
            % then use the remaining Fiala capacity for longitudinal demand.
            rearSlip=atan2(x(5)-cfg.vehicle.lr*x(6),max(x(4),cfg.model.scheduleSpeedFloor));
            brakingCapacity=p.brakingRatioLimit;
            if coupled
                rearFree=modifiedFialaTire.evaluate([0;rearSlip],0,cfg);
                frontFree=(cfg.vehicle.Iz*p.yawRateGain*(desiredYaw-x(6))+cfg.vehicle.lr*rearFree(2))/cfg.vehicle.lf;
                utilization=min(p.frontForceFraction,max(abs([frontFree;rearFree(2)])./tire.longitudinalForceScale));
                brakingCapacity=sqrt(1-utilization^2);
            end
            b=min(brakingCapacity,max(-brakingCapacity,b));
            rate=cfg.model.brakingRatioRateMaximum*cfg.controller.sampleTime;
            b=min(previous(2)+rate,max(previous(2)-rate,b));
            [lower,upper]=nonlinearBicycleModel.inputBounds(cfg,true);b=min(upper(2),max(lower(2),b));
            rear=modifiedFialaTire.evaluate([0;rearSlip],b,cfg);
            front=(cfg.vehicle.Iz*p.yawRateGain*(desiredYaw-x(6))+cfg.vehicle.lr*rear(2))/cfg.vehicle.lf;
            capacity=tire.longitudinalForceScale(1)*sqrt(1-b^2);
            fraction=min(abs(front)/capacity,p.frontForceFraction);
            slip=-sign(front)*atan(3*capacity*(1-(1-fraction)^(1/3))/tire.corneringStiffness(1));
            steering=atan2(x(5)+cfg.vehicle.lf*x(6),max(x(4),cfg.model.scheduleSpeedFloor))-slip;
            % The front braking force rotates with steering. Invert the
            % body-frame lateral force, not Fy alone, with bounded scalar
            % Newton steps on the same adhesion branch used above.
            frontCourse=atan2(x(5)+cfg.vehicle.lf*x(6),max(x(4),cfg.model.scheduleSpeedFloor));
            slipLimit=atan(3*capacity*(1-(1-p.frontForceFraction)^(1/3))/tire.corneringStiffness(1));
            for iteration=1:3*double(coupled)
                [force,slope]=modifiedFialaTire.evaluate([frontCourse-steering;rearSlip],b,cfg);
                fx=b*tire.longitudinalForceScale(1);c=cos(steering);s=sin(steering);
                value=fx*s+force(1)*c;
                derivative=fx*c-slope(1)*c-force(1)*s;
                if derivative<=eps,break;end
                steering=min(frontCourse+slipLimit,max(frontCourse-slipLimit,steering+(front-value)/derivative));
            end
            % The declared plant blends to a kinematic bicycle at low speed;
            % invert that branch too instead of applying a tire-force inverse
            % whose steering authority vanishes near rest.
            beta=asin(min(1,max(-1,cfg.vehicle.lr*desiredYaw/max(speed,eps))));
            kinematicSteering=atan((cfg.vehicle.lf+cfg.vehicle.lr)/cfg.vehicle.lr*tan(beta));
            weight=localBlend(x(4),cfg);
            u=[weight*steering+(1-weight)*kinematicSteering;b];
        end

        function parameters = nominalGuidanceParameters(cfg,curvature)
            % Trim correction for the initialization guidance, not a CLF or command.
            persistent savedKey saved
            key={cfg.vehicle,cfg.tire,cfg.roadLoad,cfg.model,cfg.actuation,cfg.referenceSpeed, ...
                cfg.controller.sampleTime,cfg.clf,cfg.nominalClf,curvature};
            if isequaln(key,savedKey),parameters=saved;return;end
            reference=nonlinearBicycleModel.cruise(cfg,curvature);
            lane=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200));
            trim=[0;0;reference.state(3:6)];
            raw=nonlinearBicycleModel.nominalFeedback(trim,reference.input,lane,reference,cfg,struct('steeringOffset',0));
            parameters=struct('steeringOffset',reference.input(1)-raw(1));
            savedKey=key;saved=parameters;
        end

        function value = nominalValue(x,lane,reference)
            % The only controller CLF: analytic transverse error at the path trim.
            e=nonlinearBicycleModel.error(x,lane,reference);
            value=sum((reference.factor*e).^2);
        end
    end
end

function a=localWrap(a)
    a=atan2(sin(a),cos(a));
end

function [weight,slope]=localBlend(speed,cfg)
    % Weight of the Fiala bicycle: 0 below model.kinematicSpeedRange, 1
    % above it, a cubic smoothstep between; and its derivative in speed.
    range=cfg.model.kinematicSpeedRange;
    s=min(1,max(0,(speed-range(1))/(range(2)-range(1))));
    weight=s^2*(3-2*s);slope=6*s*(1-s)/(range(2)-range(1));
end

function body=localBody(x,u,cfg)
    % Body accelerations [v_x'; v_y'; r'] of the declared model.
    weight=localBlend(x(4),cfg);body=zeros(3,1);
    if weight>0,body=weight*localFialaBody(x,u,cfg);end
    if weight<1,body=body+(1-weight)*localKinematicBody(x,u,cfg);end
end

function body=localFialaBody(x,u,cfg)
    % The Fiala bicycle's body accelerations (Fahmy et al., 2018).
    if x(4)<=cfg.model.scheduleSpeedFloor
        error('collisionAvoidanceController:nonlinearDomain', ...
            'Positive-speed Fiala dynamics require vx above the declared domain floor.');
    end
    tire=modifiedFialaTire.parameters(cfg);
    slip=atan2([x(5)+cfg.vehicle.lf*x(6);x(5)-cfg.vehicle.lr*x(6)],x(4))-[u(1);0];
    fy=modifiedFialaTire.evaluate(slip,u(2),cfg);
    fx=u(2)*tire.longitudinalForceScale;
    c=cos(u(1));s=sin(u(1));frontX=fx(1)*c-fy(1)*s;frontY=fx(1)*s+fy(1)*c;
    v=x(4);load=nonlinearBicycleModel.roadLoad(v,cfg);
    body=[(frontX+fx(2)-load)/cfg.vehicle.m+x(5)*x(6); ...
        (frontY+fy(2))/cfg.vehicle.m-v*x(6); ...
        (cfg.vehicle.lf*frontY-cfg.vehicle.lr*fy(2))/cfg.vehicle.Iz];
end

function body=localKinematicBody(x,u,cfg)
    % The kinematic bicycle: the yaw rate relaxes with
    % model.kinematicLagSeconds to v_x tan(delta)/L and the lateral velocity
    % to l_r times it (no slip at the rear axle); the longitudinal force is
    % the braking ratio times the axles' force scales (no lateral tire
    % force), and braking fades below model.brakeHoldSpeed, so that it ends
    % at rest and never reverses the vehicle.
    scale=modifiedFialaTire.parameters(cfg).longitudinalForceScale;
    wheelbase=cfg.vehicle.lf+cfg.vehicle.lr;lag=cfg.model.kinematicLagSeconds;
    hold=tanh(x(4)/cfg.model.brakeHoldSpeed);
    force=(max(u(2),0)+min(u(2),0)*hold)*(scale(1)*cos(u(1))+scale(2));
    rate=x(4)*tan(u(1))/wheelbase;
    body=[(force-nonlinearBicycleModel.roadLoad(x(4),cfg))/cfg.vehicle.m+x(5)*x(6); ...
        (cfg.vehicle.lr*rate-x(5))/lag;(rate-x(6))/lag];
end

function [a,b]=localFialaJacobian(x,u,cfg)
    % Analytic combined-slip tire and body-force derivatives.
    tire=modifiedFialaTire.affineModel(x,u,cfg);
    scale=modifiedFialaTire.parameters(cfg).longitudinalForceScale;
    fx=u(2)*scale;c=cos(u(1));s=sin(u(1));
    frontX=fx(1)*c-tire.force(1)*s;frontY=fx(1)*s+tire.force(1)*c;
    forceX=-s*tire.state(1,:);
    forceY=c*tire.state(1,:);
    inputX=-s*tire.input(1,:)+[-frontY,c*scale(1)];
    inputY=c*tire.input(1,:)+[frontX,s*scale(1)];
    [~,loadSlope]=nonlinearBicycleModel.roadLoad(x(4),cfg);
    a=zeros(6);b=zeros(6,2);cp=cos(x(3));sp=sin(x(3));
    a(1,3)=-x(4)*sp-x(5)*cp;a(1,4:5)=[cp,-sp];
    a(2,3)=x(4)*cp-x(5)*sp;a(2,4:5)=[sp,cp];a(3,6)=1;
    a(4,:)=forceX/cfg.vehicle.m;a(4,4)=a(4,4)-loadSlope/cfg.vehicle.m;
    a(4,5:6)=a(4,5:6)+[x(6),x(5)];
    a(5,:)=(forceY+tire.state(2,:))/cfg.vehicle.m;
    a(5,[4,6])=a(5,[4,6])-[x(6),x(4)];
    a(6,:)=(cfg.vehicle.lf*forceY-cfg.vehicle.lr*tire.state(2,:))/cfg.vehicle.Iz;
    b(4,:)=(inputX+[0,scale(2)])/cfg.vehicle.m;
    b(5,:)=(inputY+tire.input(2,:))/cfg.vehicle.m;
    b(6,:)=(cfg.vehicle.lf*inputY-cfg.vehicle.lr*tire.input(2,:))/cfg.vehicle.Iz;
end

function [a,b]=localKinematicJacobian(x,u,cfg)
    % Derivatives of localKinematicBody with the position rows. At b = 0 the
    % braking-ratio derivative is that of braking.
    scale=modifiedFialaTire.parameters(cfg).longitudinalForceScale;
    wheelbase=cfg.vehicle.lf+cfg.vehicle.lr;lag=cfg.model.kinematicLagSeconds;
    holdSpeed=cfg.model.brakeHoldSpeed;hold=tanh(x(4)/holdSpeed);
    traction=scale(1)*cos(u(1))+scale(2);gain=max(u(2),0)+min(u(2),0)*hold;
    [~,loadSlope]=nonlinearBicycleModel.roadLoad(x(4),cfg);
    t=tan(u(1));m=cfg.vehicle.m;
    a=zeros(6);b=zeros(6,2);cp=cos(x(3));sp=sin(x(3));
    a(1,3)=-x(4)*sp-x(5)*cp;a(1,4:5)=[cp,-sp];
    a(2,3)=x(4)*cp-x(5)*sp;a(2,4:5)=[sp,cp];a(3,6)=1;
    a(4,4)=(min(u(2),0)*traction*(1-hold^2)/holdSpeed-loadSlope)/m;a(4,5:6)=[x(6),x(5)];
    a(5,4)=cfg.vehicle.lr*t/(wheelbase*lag);a(5,5)=-1/lag;
    a(6,4)=t/(wheelbase*lag);a(6,6)=-1/lag;
    b(4,1)=-gain*scale(1)*sin(u(1))/m;
    if u(2)>0,b(4,2)=traction/m;else,b(4,2)=hold*traction/m;end
    b(5,1)=cfg.vehicle.lr*x(4)*(1+t^2)/(wheelbase*lag);
    b(6,1)=x(4)*(1+t^2)/(wheelbase*lag);
end

function y=localIntegrate(x,u,cfg,curvature,duration,variational)
    % One even integration mesh for full holds and their midpoints.
    count=max(1,round(nonlinearBicycleModel.meshCount(cfg)*duration/cfg.controller.sampleTime));
    h=duration/count;
    if variational,y=[x;reshape(eye(6),[],1);zeros(12,1)];else,y=x;end
    for index=1:count
        k1=localDerivative(y,u,cfg,curvature,variational);
        k2=localDerivative(y+h*k1/2,u,cfg,curvature,variational);
        k3=localDerivative(y+h*k2/2,u,cfg,curvature,variational);
        k4=localDerivative(y+h*k3,u,cfg,curvature,variational);
        y=y+h*(k1+2*k2+2*k3+k4)/6;
    end
    % A braked vehicle slower than a fifth of brakeHoldSpeed (5 cm/s) at the
    % end of the integration is at rest: its body velocities and their
    % sensitivities are exact zeros, not the vanishing tail of the fading
    % brake, whose coefficients of 1e-4 to 1e-7 left the convex problem
    % ill-conditioned while the vehicle crept to a stop. Under light braking
    % the tail would still cover about 1.5 cm.
    [next,stopped]=nonlinearBicycleModel.sampleEndState(y(1:6),u,cfg);
    y(1:6)=next;
    if stopped
        if variational
            a=reshape(y(7:42),6,6);b=reshape(y(43:54),6,2);a(4:6,:)=0;b(4:6,:)=0;
            y(7:42)=a(:);y(43:54)=b(:);
        end
    end
end

function p=localKernelParameters(cfg)
    p=[cfg.model.scheduleSpeedFloor;cfg.vehicle.m;cfg.vehicle.Iz;cfg.vehicle.lf;cfg.vehicle.lr; ...
        cfg.vehicle.gravity;cfg.tire.corneringStiffness(:);cfg.tire.frictionCoefficient(:); ...
        cfg.roadLoad.airDensity;cfg.roadLoad.dragCoefficient;cfg.roadLoad.frontalArea; ...
        cfg.roadLoad.rollingCoefficient;cfg.roadLoad.rollingSpeedCoefficient; ...
        cfg.roadLoad.rollingQuarticCoefficient;cfg.roadLoad.rollingTransitionSpeed; ...
        cfg.controller.sampleTime;cfg.nonlinear.integrationStep; ...
        cfg.model.kinematicSpeedRange(:);cfg.model.kinematicLagSeconds;cfg.model.brakeHoldSpeed];
end

function kernel=localKernel(cfg)
    % Handle of the optional compiled copy of sample, or empty. It is used
    % only if it reproduces this source bitwise at a probe point, so that a
    % kernel built from older dynamics is never used. The handle stays valid
    % without leaving the kernel folder on the path.
    persistent handle checked
    if isempty(checked)
        checked=true;handle=[];
        native=fullfile(fileparts(fileparts(mfilename('fullpath'))),'solver','controller');
        if isfile(fullfile(native,['bicycleSampleKernelMex.',mexext]))
            previous=addpath(native);candidate=@bicycleSampleKernelMex;
            % Probes in the Fiala, blended and kinematic regimes; a kernel
            % with another parameter layout fails the call.
            range=cfg.model.kinematicSpeedRange;
            probes=[.3;-.2;.1;max(8,2*range(2));.4;-.2]+[zeros(3,3);[0,mean(range)-max(8,2*range(2)),range(1)/2-max(8,2*range(2))];zeros(2,3)];
            u=[.05;-.3];duration=cfg.controller.sampleTime/2;same=true;
            for j=1:size(probes,2)
                y=localIntegrate(probes(:,j),u,cfg,[],duration,true);
                try
                    [next,a,b]=candidate(probes(:,j),u,localKernelParameters(cfg),duration,true);
                    same=same && isequal(y,[next;a(:);b(:)]);
                catch
                    same=false;
                end
            end
            path(previous);
            if same,handle=candidate;
            else
                warning('collisionAvoidanceController:staleModelKernel', ...
                    'The compiled bicycle kernel differs from the model source and is not used. Rebuild it with buildControllerKernels.');
            end
        end
    end
    kernel=handle;
end

function dy=localDerivative(y,u,cfg,curvature,variational)
    dx=nonlinearBicycleModel.derivative(y(1:6),u,cfg,curvature);
    if ~variational,dy=dx;return;end
    [a,b]=nonlinearBicycleModel.jacobian(y(1:6),u,cfg,curvature);
    dy=[dx;reshape(a*reshape(y(7:42),6,6),[],1); ...
        reshape(a*reshape(y(43:54),6,2)+b,[],1)];
end

function entry=localClfEntry(key)
    % Precomputed CLF matrices, read again only when the file changes.
    persistent table stamp
    file=fullfile(fileparts(fileparts(mfilename('fullpath'))),'config','clfMatrices.json');
    current=-1;if isfile(file),listing=dir(file);current=listing.datenum;end
    if isempty(table) || ~isequal(stamp,current)
        table=containers.Map('KeyType','char','ValueType','any');stamp=current;
        if isfile(file)
            entries=jsondecode(fileread(file));
            if ~iscell(entries),entries=num2cell(entries);end
            for index=1:numel(entries),table(char(entries{index}.key))=entries{index};end
        end
    end
    if ~isKey(table,char(key))
        error('collisionAvoidanceController:missingClfMatrix', ...
            ['No precomputed CLF matrix for this operating point (speed, curvature, vehicle and CLF settings). ' ...
            'Run scripts/synthesizeClfMatrices before the experiment.']);
    end
    entry=table(char(key));
    for name=["gain","certifiedLevel","holdFactor","nominalA","nominalB","continuousLinearDecayRate"]
        if ~isfield(entry,name)
            error('collisionAvoidanceController:staleClfMatrix', ...
                ['The CLF entry lacks a required continuous or sampled certificate field (%s). ' ...
                'Run scripts/synthesizeClfMatrices before the experiment.'],name);
        end
    end
end

function [point,residual,flag]=localTrimNewton(point,speed,curvature,cfg)
    % Damped Newton iterations on the trim residual with its analytic
    % tangent; flag 1 when the residual is below 1e-12, else 0.
    flag=0;residual=localTrimResidual(point,speed,curvature,cfg);
    for iteration=1:40
        if norm(residual,inf)<=1e-12,flag=1;return;end
        beta=point(1);[~,x,u]=localTrimResidual(point,speed,curvature,cfg);
        try
            [a,b]=nonlinearBicycleModel.jacobian(x,u,cfg,curvature);
        catch exception
            if ~startsWith(exception.identifier,'collisionAvoidanceController:'),rethrow(exception);end
            return;
        end
        tangent=[a(4:6,:)*[0;0;-1;-speed*sin(beta);speed*cos(beta);0],b(4:6,:)];
        step=-tangent\residual;
        if any(~isfinite(step)),return;end
        for halving=0:20
            trial=point+2^-halving*step;candidate=localTrimResidual(trial,speed,curvature,cfg);
            if norm(candidate)<norm(residual),break;end
        end
        point=trial;residual=candidate;
    end
    flag=double(norm(residual,inf)<=1e-12);
end

function [residual,x,u]=localTrimResidual(point,speed,curvature,cfg)
    beta=point(1);x=[0;0;-beta;speed*cos(beta);speed*sin(beta);speed*curvature];u=point(2:3);
    try
        dx=nonlinearBicycleModel.derivative(x,u,cfg,curvature);residual=dx(4:6);
    catch exception
        if ~startsWith(exception.identifier,'collisionAvoidanceController:'),rethrow(exception);end
        residual=1e3*ones(3,1)+point;
    end
end
