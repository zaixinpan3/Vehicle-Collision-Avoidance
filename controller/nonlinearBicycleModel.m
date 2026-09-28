classdef nonlinearBicycleModel
    %nonlinearBicycleModel Nonlinear ZOH proposals, variational flow and trims.
    % RK4 evaluates the flow; acceptance uses MPFR interval enclosures separately.
    methods (Static)
        function dx = derivative(x,u,cfg,curvature)
            if nargin<4,curvature=[];end
            if x(4)<=cfg.model.scheduleSpeedFloor || abs(u(2))>1
                error('collisionAvoidanceController:nonlinearDomain', ...
                    'Positive-speed Fiala dynamics require vx above the declared domain floor and |b|<=1.');
            end
            tire=modifiedFialaTire.parameters(cfg);
            slip=atan2([x(5)+cfg.vehicle.lf*x(6);x(5)-cfg.vehicle.lr*x(6)],x(4))-[u(1);0];
            fy=modifiedFialaTire.evaluate(slip,u(2),cfg);
            fx=u(2)*tire.longitudinalForceScale;
            c=cos(u(1));s=sin(u(1));frontX=fx(1)*c-fy(1)*s;frontY=fx(1)*s+fy(1)*c;
            road=cfg.roadLoad;v=x(4);
            load=.5*road.airDensity*road.dragCoefficient*road.frontalArea*v^2 ...
                +cfg.vehicle.m*cfg.vehicle.gravity*(road.rollingCoefficient ...
                +road.rollingSpeedCoefficient*v+road.rollingQuarticCoefficient*v^4) ...
                *tanh(v/road.rollingTransitionSpeed);
            dx=[v*cos(x(3))-x(5)*sin(x(3));v*sin(x(3))+x(5)*cos(x(3));x(6); ...
                (frontX+fx(2)-load)/cfg.vehicle.m+x(5)*x(6); ...
                (frontY+fy(2))/cfg.vehicle.m-v*x(6); ...
                (cfg.vehicle.lf*frontY-cfg.vehicle.lr*fy(2))/cfg.vehicle.Iz];
            if ~isempty(curvature)
                denominator=1-curvature*x(2);
                if denominator<=0,error('collisionAvoidanceController:nonlinearDomain','Invalid Frenet chart.');end
                dx(1)=dx(1)/denominator;dx(3)=dx(3)-curvature*dx(1);
            end
        end

        function [next,a,b] = sample(x,u,cfg,curvature)
            if nargin<4,curvature=[];end
            count=max(1,ceil(cfg.controller.sampleTime/cfg.nonlinear.integrationStep));
            h=cfg.controller.sampleTime/count;variational=nargout>1;
            if variational,y=[x;reshape(eye(6),[],1);zeros(12,1)];else,y=x;end
            for index=1:count
                k1=localFlow(y,u,cfg,curvature,variational);
                k2=localFlow(y+h*k1/2,u,cfg,curvature,variational);
                k3=localFlow(y+h*k2/2,u,cfg,curvature,variational);
                k4=localFlow(y+h*k3,u,cfg,curvature,variational);
                y=y+h*(k1+2*k2+2*k3+k4)/6;
            end
            next=y(1:6);
            if variational,a=reshape(y(7:42),6,6);b=reshape(y(43:54),6,2);end
        end

        function states = rollout(x,inputs,cfg)
            states=zeros(6,size(inputs,2)+1);states(:,1)=x;
            for index=1:size(inputs,2)
                states(:,index+1)=nonlinearBicycleModel.sample(states(:,index),inputs(:,index),cfg);
            end
        end

        function [next,a,b] = jointSample(z,u,targetParameters,cfg)
            % Joint autonomous state [ego(6); target position(2); target yaw].
            % Target parameters are [speed; sideslip; yawRate; half extents;
            % body offset]. Eliminating its unactuated prediction is exact.
            if isempty(targetParameters)
                [next,a,b]=nonlinearBicycleModel.sample(z,u,cfg);
                return;
            end
            validateattributes(z,{'double'},{'size',[9,1],'real','finite'});
            validateattributes(targetParameters,{'double'},{'size',[7,1],'real','finite'});
            [ego,ae,be]=nonlinearBicycleModel.sample(z(1:6),u,cfg);
            target=[z(7:9);targetParameters];
            future=nonlinearSafetyCertificate.targetFlow(target,cfg.controller.sampleTime);
            displacement=future(1:2)-target(1:2);
            at=eye(3);at(1:2,3)=[-displacement(2);displacement(1)];
            next=[ego;future(1:3)];a=blkdiag(ae,at);b=[be;zeros(3,2)];
        end

        function [error,jacobian] = errorLinearization(x,lane,reference)
            [error,projection]=nonlinearBicycleModel.error(x,lane,reference);
            tangent=[cos(projection.heading);sin(projection.heading)];
            jacobian=zeros(5,6);jacobian(1,1:2)=[-tangent(2),tangent(1)];
            jacobian(2,3)=1;
            jacobian(2,1:2)=-reference.curvature*tangent.'/(1-reference.curvature*projection.lateralPosition);
            jacobian(3:5,4:6)=eye(3);
        end

        function [a,b] = jacobian(x,u,cfg,curvature)
            a=zeros(6);b=zeros(6,2);step=cfg.nonlinear.finiteDifferenceStep;
            for index=1:8
                point=[x;u];d=step*max(1,abs(point(index)));lo=point;hi=point;
                lo(index)=lo(index)-d;hi(index)=hi(index)+d;
                column=(nonlinearBicycleModel.derivative(hi(1:6),hi(7:8),cfg,curvature) ...
                    -nonlinearBicycleModel.derivative(lo(1:6),lo(7:8),cfg,curvature))/(2*d);
                if index<=6,a(:,index)=column;else,b(:,index-6)=column;end
            end
        end

        function reference = cruise(cfg,curvature)
            persistent savedKey saved
            key={cfg.vehicle,cfg.tire,cfg.roadLoad,cfg.model,cfg.referenceSpeed, ...
                cfg.controller.sampleTime,cfg.nonlinear.integrationStep,cfg.clf,curvature};
            if isequaln(key,savedKey),reference=saved;return;end
            speed=cfg.referenceSpeed;
            objective=@(p)localTrimResidual(p,speed,curvature,cfg);
            options=optimoptions('fsolve','Display','off','FunctionTolerance',1e-12, ...
                'StepTolerance',1e-12,'OptimalityTolerance',1e-12);
            [point,residual,flag]=fsolve(objective,[0;atan(cfg.vehicle.wheelbase*curvature);.02],options);
            if flag<=0 || norm(residual,inf)>1e-8
                error('collisionAvoidanceController:unrealizableReference','No positive-speed constant-curvature trim was found.');
            end
            [~,x,u]=localTrimResidual(point,speed,curvature,cfg);
            [a,b]=nonlinearBicycleModel.jacobian(x,u,cfg,curvature);
            transition=expm([a(2:6,2:6),b(2:6,:);zeros(2,7)]*cfg.controller.sampleTime);
            scales=[cfg.clf.lateralPositionErrorScale,cfg.clf.headingErrorScale,cfg.clf.speedErrorScale, ...
                cfg.clf.lateralVelocityErrorScale,cfg.clf.yawRateErrorScale];
            q=diag(1./scales.^2);r=diag([cfg.clf.frontWheelSteeringAngleWeight,cfg.clf.brakingRatioWeight]);
            [gain,p]=dlqr(transition(1:5,1:5),transition(1:5,6:7),q,r);
            reference=struct('state',x,'input',u,'curvature',curvature,'gain',-gain, ...
                'matrix',p,'factor',chol(p),'continuousA',a(2:6,2:6),'continuousB',b(2:6,:));
            savedKey=key;saved=reference;
        end

        function [error,projection] = error(x,lane,reference)
            projection=laneGeometry.project(x(1:2),lane);
            angle=atan2(sin(x(3)-projection.heading-reference.state(3)), ...
                cos(x(3)-projection.heading-reference.state(3)));
            error=[projection.lateralPosition;angle;x(4:6)-reference.state(4:6)];
        end
    end
end

function dy=localFlow(y,u,cfg,curvature,variational)
    dx=nonlinearBicycleModel.derivative(y(1:6),u,cfg,curvature);
    if ~variational,dy=dx;return;end
    [a,b]=nonlinearBicycleModel.jacobian(y(1:6),u,cfg,curvature);
    dy=[dx;reshape(a*reshape(y(7:42),6,6),[],1); ...
        reshape(a*reshape(y(43:54),6,2)+b,[],1)];
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
