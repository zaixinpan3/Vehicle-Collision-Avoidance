classdef nonlinearBicycleModel
    %nonlinearBicycleModel Nominal held-input prediction, tangents and lane trims.
    % RK4 and its variational equations define the numerical prediction model.
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

        function [next,a,b] = sample(x,u,cfg,curvature,duration)
            if nargin<4,curvature=[];end
            if nargin<5,duration=cfg.controller.sampleTime;end
            % One even integration mesh for full holds and their midpoints.
            fullCount=2*max(1,ceil(cfg.controller.sampleTime/(2*cfg.nonlinear.integrationStep)));
            count=max(1,round(fullCount*duration/cfg.controller.sampleTime));
            h=duration/count;variational=nargout>1;
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
                if nargout>1,[next,a,b]=nonlinearBicycleModel.sample(z,u,cfg);
                else,next=nonlinearBicycleModel.sample(z,u,cfg);end
                return;
            end
            validateattributes(z,{'double'},{'size',[9,1],'real','finite'});
            validateattributes(targetParameters,{'double'},{'size',[7,1],'real','finite'});
            if nargout>1,[ego,ae,be]=nonlinearBicycleModel.sample(z(1:6),u,cfg);
            else,ego=nonlinearBicycleModel.sample(z(1:6),u,cfg);end
            target=[z(7:9);targetParameters];
            future=predictiveSafetyGeometry.targetFlow(target,cfg.controller.sampleTime);
            displacement=future(1:2)-target(1:2);
            at=eye(3);at(1:2,3)=[-displacement(2);displacement(1)];
            next=[ego;future(1:3)];
            if nargout>1,a=blkdiag(ae,at);b=[be;zeros(3,2)];end
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
            % Analytic combined-slip tire and body-force derivatives.
            if nargin<4,curvature=[];end
            tire=modifiedFialaTire.affineModel(x,u,cfg);
            scale=modifiedFialaTire.parameters(cfg).longitudinalForceScale;
            fx=u(2)*scale;c=cos(u(1));s=sin(u(1));
            frontX=fx(1)*c-tire.force(1)*s;frontY=fx(1)*s+tire.force(1)*c;
            forceX=-s*tire.state(1,:);
            forceY=c*tire.state(1,:);
            inputX=-s*tire.input(1,:)+[-frontY,c*scale(1)];
            inputY=c*tire.input(1,:)+[frontX,s*scale(1)];
            [~,loadSlope]=ltvBicycleModel.roadLoad(x(4),cfg);
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
            if ~isempty(curvature)
                denominator=1-curvature*x(2);
                forward=x(4)*cp-x(5)*sp;
                a(1,:)=a(1,:)/denominator;
                a(1,2)=a(1,2)+curvature*forward/denominator^2;
                a(3,:)=a(3,:)-curvature*a(1,:);
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
