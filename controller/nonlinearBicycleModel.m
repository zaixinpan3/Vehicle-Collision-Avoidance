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
            v=x(4);load=nonlinearBicycleModel.roadLoad(v,cfg);
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
            if ~isempty(curvature)
                denominator=1-curvature*x(2);
                forward=x(4)*cp-x(5)*sp;
                a(1,:)=a(1,:)/denominator;
                a(1,2)=a(1,2)+curvature*forward/denominator^2;
                a(3,:)=a(3,:)-curvature*a(1,:);
            end
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
            % Trim, sampled local transverse model and the CLF matrix P. P is
            % never computed here: it is read from config/clfMatrices.json,
            % which scripts/synthesizeClfMatrices.m writes before experiments.
            persistent savedKeys savedReferences
            if isempty(savedKeys),savedKeys={};savedReferences={};end
            key=nonlinearBicycleModel.clfKey(cfg,curvature);
            index=find(strcmp(savedKeys,key),1);
            if ~isempty(index),reference=savedReferences{index};return;end
            point=nonlinearBicycleModel.operatingPoint(cfg,curvature);
            entry=localClfEntry(key);
            p=reshape(entry.matrix,5,5);p=(p+p.')/2;
            reference=struct('state',point.state,'input',point.input,'curvature',curvature, ...
                'matrix',p,'factor',chol(p),'contraction',exp(-2*cfg.controller.sampleTime/cfg.clf.convergenceTimeConstantSeconds), ...
                'certifiedContraction',entry.certifiedContraction, ...
                'continuousA',point.continuousA,'continuousB',point.continuousB);
            savedKeys=[{key},savedKeys(1:min(end,15))];savedReferences=[{reference},savedReferences(1:min(end,15))];
        end

        function point = operatingPoint(cfg,curvature)
            % Constant-speed, constant-curvature trim and its sampled
            % transverse model e+ = A e + B (u - u_ref).
            speed=cfg.referenceSpeed;
            objective=@(p)localTrimResidual(p,speed,curvature,cfg);
            options=optimoptions('fsolve','Display','off','FunctionTolerance',1e-12, ...
                'StepTolerance',1e-12,'OptimalityTolerance',1e-12);
            [solution,residual,flag]=fsolve(objective,[0;atan(cfg.vehicle.wheelbase*curvature);.02],options);
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
            data=struct('vehicle',cfg.vehicle,'tire',cfg.tire,'roadLoad',cfg.roadLoad,'model',cfg.model, ...
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
            % Path guidance used only to construct an initialization (NOMINAL_CLF.md).
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
            b=min(min(1-1e-8,cfg.actuation.brakingRatioMaximum),max(max(-1+1e-8,cfg.actuation.brakingRatioMinimum),b));
            rear=modifiedFialaTire.evaluate([0;atan2(x(5)-cfg.vehicle.lr*x(6),x(4))],b,cfg);
            front=(cfg.vehicle.Iz*p.yawRateGain*(yawRate-x(6))+cfg.vehicle.lr*rear(2))/cfg.vehicle.lf;
            capacity=tire.longitudinalForceScale(1)*sqrt(1-b^2);
            fraction=min(abs(front)/capacity,p.frontForceFraction);
            slip=-sign(front)*atan(3*capacity*(1-(1-fraction)^(1/3))/tire.corneringStiffness(1));
            u=[atan2(x(5)+cfg.vehicle.lf*x(6),x(4))-slip+terminal.steeringOffset;b];
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
end

function p=localKernelParameters(cfg)
    p=[cfg.model.scheduleSpeedFloor;cfg.vehicle.m;cfg.vehicle.Iz;cfg.vehicle.lf;cfg.vehicle.lr; ...
        cfg.vehicle.gravity;cfg.tire.corneringStiffness(:);cfg.tire.frictionCoefficient(:); ...
        cfg.roadLoad.airDensity;cfg.roadLoad.dragCoefficient;cfg.roadLoad.frontalArea; ...
        cfg.roadLoad.rollingCoefficient;cfg.roadLoad.rollingSpeedCoefficient; ...
        cfg.roadLoad.rollingQuarticCoefficient;cfg.roadLoad.rollingTransitionSpeed; ...
        cfg.controller.sampleTime;cfg.nonlinear.integrationStep];
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
            x=[.3;-.2;.1;max(8,2*cfg.model.scheduleSpeedFloor);.4;-.2];u=[.05;-.3];
            duration=cfg.controller.sampleTime/2;
            y=localIntegrate(x,u,cfg,[],duration,true);
            [next,a,b]=candidate(x,u,localKernelParameters(cfg),duration,true);
            path(previous);
            if isequal(y,[next;a(:);b(:)]),handle=candidate;
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
            for index=1:numel(entries),table(char(entries(index).key))=entries(index);end
        end
    end
    if ~isKey(table,char(key))
        error('collisionAvoidanceController:missingClfMatrix', ...
            ['No precomputed CLF matrix for this operating point (speed, curvature, vehicle and CLF settings). ' ...
            'Run scripts/synthesizeClfMatrices before the experiment.']);
    end
    entry=table(char(key));
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
