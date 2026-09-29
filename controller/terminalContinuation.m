classdef terminalContinuation
    %terminalContinuation Time-indexed endpoint family for a hard completion tail.
    % A cached local RK4 contraction construction closes the infinite suffix.
    % It is a nominal numerical-model result, not a continuous-plant certificate.
    methods (Static)
        function seed = build(cfg,curvature)
            persistent savedKey saved
            key={cfg.vehicle,cfg.tire,cfg.roadLoad,cfg.model,cfg.actuation,cfg.clf, ...
                cfg.referenceSpeed,cfg.controller.sampleTime,cfg.nonlinear,curvature};
            if isequaln(key,savedKey),seed=saved;return;end
            reference=nonlinearBicycleModel.cruise(cfg,curvature);
            base=[zeros(3,1);reference.state(4:6)];
            [next,a,b]=nonlinearBicycleModel.sample(base,reference.input,cfg);
            rotation=localRotation(-next(3));transform=blkdiag(rotation,eye(4));
            aa=[transform*a,zeros(6,2);zeros(2,8)];bb=[transform*b;eye(2)];
            scales=[1,cfg.clf.lateralPositionErrorScale,cfg.clf.headingErrorScale, ...
                cfg.clf.speedErrorScale,cfg.clf.lateralVelocityErrorScale,cfg.clf.yawRateErrorScale,.2,.2];
            [gain,p]=dlqr(aa,bb,diag(1./scales.^2),eye(2));gain=-gain;factor=chol(p);
            inverse=factor\eye(8);
            residual=localLeftInterval(factor,repmat(inverse,1,1,2))-eye(8);
            residualMagnitude=max(abs(residual),[],3);
            residualBound=sqrt(norm(residualMagnitude,inf)*norm(residualMagnitude,1))+1e-13;
            if residualBound>=1
                error('collisionAvoidanceController:noTerminalContinuation','The endpoint coordinate transform is singular.');
            end
            inverseError=norm(inverse,'fro')*residualBound/(1-residualBound);
            unit=vecnorm(inverse,2,2)+inverseError;
            [~,arithmetic,~,centerDomain]=localEnclosure(base,reference.input,gain,zeros(8,1),cfg,inverse);
            arithmetic=norm(factor,'fro')*arithmetic;
            radius=cfg.nonlinear.terminalRadius;
            seed=struct('reference',reference,'gain',gain,'factor',factor,'matrix',p, ...
                'increment',next(1:3),'base',base,'radius',0,'errorBound',zeros(8,1));
            for attempt=1:32
                [derivative,defect,samples,domain]=localEnclosure(base,reference.input,gain,radius*unit,cfg,inverse);
                derivative=localLeftInterval(blkdiag(rotation,eye(6)),derivative);
                derivative=localLeftInterval(factor,derivative);
                % R*B=I+E implies R^-1=B*(I+E)^-1. The Neumann bound
                % accounts for the numerical inverse without a guessed error.
                gamma=localNormBound(derivative)/(1-residualBound);
                drift=norm(factor*[zeros(3,1);next(4:6)-base(4:6);zeros(2,1)])+arithmetic+defect;
                inputBound=radius*(vecnorm(gain*inverse,2,2)+vecnorm(gain,2,2)*inverseError);
                difference=gain-[zeros(2,6),eye(2)];
                slewBound=radius*(vecnorm(difference*inverse,2,2)+vecnorm(difference,2,2)*inverseError);
                low=[-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum];
                high=[cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum];
                rate=cfg.controller.sampleTime*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
                admissible=all(reference.input-inputBound>low) && all(reference.input+inputBound<high) ...
                    && all(slewBound<rate) && domain && centerDomain;
                if admissible && gamma<1 && drift<(1-gamma)*radius
                    seed.radius=radius;seed.errorBound=radius*unit;
                    seed.contractionBound=gamma;seed.defectBound=drift;
                    seed.samplePositionBound=max(samples(1:2,:),[],2);
                    seed.sampleHeadingBound=max(samples(3,:));
                    half=nonlinearBicycleModel.sample(base,reference.input,cfg,[],cfg.controller.sampleTime/2);
                    if next(3)==0
                        seed.samplePositionBound=seed.samplePositionBound+abs(half(1:2)-next(1:2)/2);
                    else
                        center=(eye(2)-localRotation(next(3)))\next(1:2);
                        seed.samplePositionBound=seed.samplePositionBound+abs(norm(half(1:2)-center)-norm(center));
                    end
                    seed.construction="boundedRk4JacobianContraction";
                    savedKey=key;saved=seed;return;
                end
                radius=radius/2;
            end
            error('collisionAvoidanceController:noTerminalContinuation', ...
                'No admissible contracting endpoint neighborhood was established for this configuration.');
        end

        function seed = anchor(seed,x,sampleIndex,lane)
            projection=laneGeometry.project(x(1:2),lane);
            seed.epochIndex=sampleIndex;
            seed.epochState=[projection.point;projection.heading+seed.reference.state(3);seed.base(4:6)];
        end

        function y = referenceAt(seed,index)
            steps=index-seed.epochIndex;angle=seed.increment(3);theta=steps*angle;
            if angle==0
                displacement=steps*seed.increment(1:2);
            else
                center=(eye(2)-localRotation(angle))\seed.increment(1:2);
                displacement=(eye(2)-localRotation(theta))*center;
            end
            x=seed.epochState;x(1:2)=x(1:2)+localRotation(x(3))*displacement;x(3)=x(3)+theta;
            y=[x;seed.reference.input];
        end

        function [value,gradient,error] = membership(y,index,seed)
            reference=terminalContinuation.referenceAt(seed,index);
            transform=blkdiag(localRotation(-reference(3)),eye(6));
            error=transform*(y-reference);error(3)=atan2(sin(error(3)),cos(error(3)));
            scaled=seed.factor*error;value=scaled.'*scaled-seed.radius^2;
            gradient=2*scaled.'*seed.factor*transform;
        end

        function u = control(y,index,seed)
            [value,~,deviation]=terminalContinuation.membership(y,index,seed);
            if value>0
                error('collisionAvoidanceController:outsideContinuation','Terminal control requires membership in the endpoint family.');
            end
            u=seed.reference.input+seed.gain*deviation;
        end

        function margin = separation(seed,q,frame,cfg)
            % Sufficient all-future geometry for the endpoint only. The finite
            % completion tail compares vehicles at matching absolute times.
            % The given path defines the reference; road edges are not constraints.
            if isempty(q),margin=Inf;return;end
            x=seed.epochState;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
            reach=norm(shape(1:2)+abs(shape(3:4)));
            positionError=norm(seed.samplePositionBound);headingError=seed.sampleHeadingBound;
            body=shape(3:4)+shape(1:2).*[-1,1,1,-1;-1,-1,1,1];
            corners=localRotation(x(3))*body;clearance=cfg.collision.safetyMarginMeters;
            t=[cos(frame(3));sin(frame(3))];n=[-t(2);t(1)];
            if seed.increment(3)==0
                velocity=localRotation(x(3))*seed.increment(1:2)/cfg.controller.sampleTime;
                side=n.'*(x(1:2)-frame(1:2));extent=n.'*corners;
                padding=positionError+reach*headingError;
                egoSide=[side+min(extent)-padding;side+max(extent)+padding];
                if q(6)~=0 && any(q(4:5)~=0)
                    [center,~,outer]=localOrbit(q);
                    side=n.'*(center-frame(1:2))+[-outer;outer];
                    forward=t.'*(x(1:2)-center)+min(t.'*corners)-padding-outer;
                    if t.'*velocity<0,forward=-Inf;end
                    collision=max([egoSide(1)-side(2),side(1)-egoSide(2),forward]);
                else
                    targetVertices=q(1:2)+localRotation(q(3))*(q(10:11)+q(8:9).*[-1,1,1,-1;-1,-1,1,1]);
                    direction=predictiveSafetyGeometry.direction(q(3)+q(6)-frame(3));
                    egoDirection=predictiveSafetyGeometry.direction(x(3)-frame(3));
                    projectedVelocity=[egoDirection(1),-egoDirection(2);egoDirection(2),egoDirection(1)] ...
                        *seed.increment(1:2)/cfg.controller.sampleTime;
                    relative=projectedVelocity-q(4)*direction;
                    speeds=[relative(1),-relative(1),relative(2),-relative(2)];
                    accelerations=-q(5)*[direction(1),-direction(1),direction(2),-direction(2)];
                    % Relative quadratic progress accounts for both ahead and
                    % behind separation, acceleration, and signed reversal.
                    collision=-Inf;
                    normals=[t,-t,n,-n];
                    for axis=1:4
                        normal=normals(:,axis);
                        lower=localProgressMinimum(speeds(axis),accelerations(axis));
                        gap=min(normal.'*(x(1:2)+corners))-max(normal.'*targetVertices)-padding+lower;
                        collision=max(collision,gap);
                    end
                end
            else
                centerLocal=(eye(2)-localRotation(seed.increment(3)))\seed.increment(1:2);
                center=x(1:2)+localRotation(x(3))*centerLocal;radius=norm(centerLocal);
                width=reach+positionError;
                if q(6)~=0 && any(q(4:5)~=0)
                    [targetCenter,inner,outer]=localOrbit(q);d=norm(center-targetCenter);
                    nearest=max([0,d-outer,inner-d]);farthest=d+outer;
                else
                    direction=predictiveSafetyGeometry.direction(q(3)+q(6));d=q(1:2)-center;
                    lower=localProgressMinimum(q(4),q(5));upper=-localProgressMinimum(-q(4),-q(5));
                    arc=min(upper,max(lower,-d.'*direction));targetReach=norm(q(8:9)+abs(q(10:11)));
                    nearest=norm(d+arc*direction)-targetReach;
                    farthest=max(norm(d+lower*direction),norm(d+upper*direction))+targetReach;
                    if ~isfinite(lower) || ~isfinite(upper),farthest=Inf;end
                end
                collision=max(nearest-radius-width,radius-width-farthest);
            end
            margin=collision-clearance;
        end
    end
end

function rotation=localRotation(angle)
    rotation=[cos(angle),-sin(angle);sin(angle),cos(angle)];
end

function minimum=localProgressMinimum(v,a)
    if a<0 || (a==0 && v<0),minimum=-Inf;
    elseif a>0,minimum=-min(v,0)^2/(2*a);
    else,minimum=0;
    end
end

function [center,inner,outer]=localOrbit(q)
    radius=q(7)/sin(q(6));angle=q(3)+q(6);
    center=q(1:2)+radius*[-sin(angle);cos(angle)];
    offset=[-radius*sin(q(6));radius*cos(q(6))]-q(10:11);
    inner=norm(max(0,abs(offset)-q(8:9)));outer=norm(abs(offset)+q(8:9));
end

function [jacobian,defect,samples,domain] = localEnclosure(base,input,gain,bound,cfg,coordinates)
    % First-derivative enclosures of the held-input RK4 expression. Bounds
    % are computed once at construction, never for an online MPC candidate.
    x=cell(6,1);u=cell(2,1);
    for k=1:6,x{k}=[localOut(base(k)+[-bound(k),bound(k)]);coordinates(k,:).',coordinates(k,:).'];end
    inputBound=abs(gain)*bound;
    inputTangent=localLeftInterval(gain,repmat(coordinates,1,1,2));
    for k=1:2,u{k}=[localOut(input(k)+[-inputBound(k),inputBound(k)]);reshape(inputTangent(k,:,:),8,2)];end
    count=2*max(1,ceil(cfg.controller.sampleTime/(2*cfg.nonlinear.integrationStep)));
    h=cfg.controller.sampleTime/count;nominal=base;samples=bound(1:3);domain=true;
    try
        for k=1:count
            f1=localVectorField(x,u,cfg);f2=localVectorField(localStage(x,f1,h/2),u,cfg);
            f3=localVectorField(localStage(x,f2,h/2),u,cfg);f4=localVectorField(localStage(x,f3,h),u,cfg);
            for i=1:6
                total=localAdd(localAdd(f1{i},localScale(f2{i},2)),localAdd(localScale(f3{i},2),f4{i}));
                x{i}=localAdd(x{i},localScale(total,h/6));
            end
            if k==count/2 || k==count
                nominal=nonlinearBicycleModel.sample(nominal,input,cfg,[],cfg.controller.sampleTime/2);
                width=zeros(3,1);
                for i=1:3,width(i)=max(abs(x{i}(1,:)-nominal(i)));end
                samples(:,end+1)=width; %#ok<AGROW>
            end
            localDomain(x,cfg);
        end
        jacobian=zeros(8,8,2);
        for i=1:6,jacobian(i,:,1)=x{i}(2:end,1);jacobian(i,:,2)=x{i}(2:end,2);end
        jacobian(7:8,:,:)=inputTangent;
        % The center evaluation uses the same expression with zero initial
        % radius; its interval encloses arithmetic and elementary-function
        % evaluation errors in the reference defect.
        defect=0;
        if all(bound==0)
            for i=1:6,defect=defect+max(abs(x{i}(1,:)-nominal(i)))^2;end
            defect=sqrt(defect)+64*eps(max(1,norm(nominal)));
        end
    catch exception
        if ~strcmp(exception.identifier,'collisionAvoidanceController:terminalEnclosure'),rethrow(exception);end
        jacobian=repmat(eye(8),1,1,2);defect=Inf;domain=false;samples=Inf(3,1);
    end
end

function next=localStage(x,f,h)
    next=cell(6,1);
    for k=1:6,next{k}=localAdd(x{k},localScale(f{k},h));end
end

function localDomain(x,cfg)
    if x{4}(1,1)<=max(cfg.model.scheduleSpeedFloor+1e-4,cfg.model.speedMinimum) ...
            || x{4}(1,2)>=cfg.model.speedMaximum ...
            || max(abs(x{5}(1,:)))>=cfg.model.lateralVelocityMaximum ...
            || max(abs(x{6}(1,:)))>=cfg.model.yawRateMaximum
        error('collisionAvoidanceController:terminalEnclosure','Endpoint enclosure leaves a hard state bound.');
    end
end

function f=localVectorField(x,u,cfg)
    localDomain(x,cfg);tire=modifiedFialaTire.parameters(cfg);
    c=localTrig(u{1},false);s=localTrig(u{1},true);tanSteer=localDivide(s,c);
    fx=cell(2,1);fy=cell(2,1);lever=[cfg.vehicle.lf;-cfg.vehicle.lr];
    eta=localSqrt(localAdd(localConstant(1),localScale(localMultiply(u{2},u{2}),-1)));
    for axle=1:2
        z=localDivide(localAdd(x{5},localScale(x{6},lever(axle))),x{4});
        if axle==1
            denominator=localAdd(localConstant(1),localMultiply(z,tanSteer));
            if denominator(1,1)<=0
                error('collisionAvoidanceController:terminalEnclosure','Endpoint tire angle leaves the Fiala chart.');
            end
            z=localDivide(localAdd(z,localScale(tanSteer,-1)),denominator);
        end
        capacity=localScale(eta,tire.longitudinalForceScale(axle));stiffness=cfg.tire.corneringStiffness(axle);
        ratio=localDivide(localScale(localAbs(z),stiffness/3),capacity);
        if ratio(1,2)>=1
            error('collisionAvoidanceController:terminalEnclosure','Endpoint enclosure reaches tire saturation.');
        end
        polynomial=localAdd(localAdd(localConstant(1),localScale(ratio,-1)),localScale(localMultiply(ratio,ratio),1/3));
        fy{axle}=localScale(localMultiply(z,polynomial),-stiffness);
        fx{axle}=localScale(u{2},tire.longitudinalForceScale(axle));
    end
    frontX=localAdd(localMultiply(fx{1},c),localScale(localMultiply(fy{1},s),-1));
    frontY=localAdd(localMultiply(fx{1},s),localMultiply(fy{1},c));
    v2=localMultiply(x{4},x{4});load=cfg.roadLoad;
    polynomial=localAdd(localConstant(load.rollingCoefficient),localScale(x{4},load.rollingSpeedCoefficient));
    polynomial=localAdd(polynomial,localScale(localMultiply(v2,v2),load.rollingQuarticCoefficient));
    drag=localScale(v2,.5*load.airDensity*load.dragCoefficient*load.frontalArea);
    rolling=localScale(localMultiply(polynomial,localTanh(localScale(x{4},1/load.rollingTransitionSpeed))),cfg.vehicle.m*cfg.vehicle.gravity);
    cp=localTrig(x{3},false);sp=localTrig(x{3},true);
    f={localAdd(localMultiply(x{4},cp),localScale(localMultiply(x{5},sp),-1)); ...
        localAdd(localMultiply(x{4},sp),localMultiply(x{5},cp));x{6}; ...
        localAdd(localScale(localAdd(localAdd(frontX,fx{2}),localScale(localAdd(drag,rolling),-1)),1/cfg.vehicle.m),localMultiply(x{5},x{6})); ...
        localAdd(localScale(localAdd(frontY,fy{2}),1/cfg.vehicle.m),localScale(localMultiply(x{4},x{6}),-1)); ...
        localScale(localAdd(localScale(frontY,cfg.vehicle.lf),localScale(fy{2},-cfg.vehicle.lr)),1/cfg.vehicle.Iz)};
end

function result=localConstant(value)
    result=[value,value;zeros(8,2)];
end
function result=localAdd(a,b)
    result=localOut(a+b);
end
function result=localScale(a,b)
    result=localTimes(a,[b,b]);
end
function result=localMultiply(a,b)
    result=[localTimes(a(1,:),b(1,:));localOut(localTimes(a(2:end,:),b(1,:))+localTimes(b(2:end,:),a(1,:)))];
end
function result=localDivide(a,b)
    if b(1,1)<=0 && b(1,2)>=0
        error('collisionAvoidanceController:terminalEnclosure','Endpoint denominator enclosure contains zero.');
    end
    inverse=localOut([1/b(1,2),1/b(1,1)]);
    derivative=localScale(localTimes(b(2:end,:),localTimes(inverse,inverse)),-1);
    result=localMultiply(a,[inverse;derivative]);
end
function result=localAbs(a)
    if a(1,1)>=0,result=a;
    elseif a(1,2)<=0,result=localScale(a,-1);
    else,result=[0,max(abs(a(1,:)));localTimes(a(2:end,:),[-1,1])];
    end
end
function result=localSqrt(a)
    if a(1,1)<=0,error('collisionAvoidanceController:terminalEnclosure','Nonpositive tire-capacity enclosure.');end
    value=localOut(sqrt(a(1,:)));
    result=[value;localTimes(a(2:end,:),localOut(.5./fliplr(value)))];
end
function result=localTrig(a,isSine)
    if max(abs(a(1,:)))>1
        error('collisionAvoidanceController:terminalEnclosure','Endpoint angle enclosure is outside the Taylor chart.');
    end
    sine=localSinCos(a(1,:),true);cosine=localSinCos(a(1,:),false);
    if isSine,result=[sine;localTimes(a(2:end,:),cosine)];
    else,result=[cosine;localScale(localTimes(a(2:end,:),sine),-1)];end
end
function value=localSinCos(a,isSine)
    square=localTimes(a,a);
    if isSine,term=a;start=1;else,term=[1,1];start=0;end
    value=term;
    for k=start+2:2:start+24
        term=localScale(localTimes(term,square),-1/(k*(k-1)));value=localOut(value+term);
    end
    % |a|<=1: the omitted Taylor remainder is smaller than 1/25!.
    value=localOut(value+[-1,1]/factorial(25));
end
function result=localTanh(a)
    interval=localScale(a(1,:),-2);halvings=max(0,ceil(log2(max(1,max(abs(interval))/.5))));
    interval=localScale(interval,2^-halvings);value=[1,1];term=[1,1];
    for k=1:18,term=localScale(localTimes(term,interval),1/k);value=localOut(value+term);end
    value=localOut(value+[-1,1]*2*.5^19/factorial(19));
    for k=1:halvings,value=localTimes(value,value);end
    numerator=localOut([1,1]-fliplr(value));denominator=localOut([1,1]+value);
    tangent=localTimes(numerator,localOut(1./fliplr(denominator)));
    slope=localOut([1,1]-fliplr(localTimes(tangent,tangent)));
    result=[tangent;localTimes(a(2:end,:),slope)];
end
function value=localTimes(a,b)
    products=cat(3,a(:,1).*b(:,1),a(:,1).*b(:,2),a(:,2).*b(:,1),a(:,2).*b(:,2));
    value=localOut([min(products,[],3),max(products,[],3)]);
end
function value=localOut(value)
    padding=8*eps(max(abs(value),[],2));value=value+padding.*[-1,1];
    if any(~isfinite(value),'all'),error('collisionAvoidanceController:terminalEnclosure','Nonfinite endpoint enclosure.');end
end
function result=localLeftInterval(a,b)
    result=zeros(size(a,1),size(b,2),2);
    for i=1:size(a,1)
        for j=1:size(b,2)
            value=[0,0];
            for k=1:size(a,2),value=localOut(value+localScale(reshape(b(k,j,:),1,2),a(i,k)));end
            result(i,j,:)=value;
        end
    end
end
function result=localRightInterval(a,b)
    result=permute(localLeftInterval(b.',permute(a,[2,1,3])),[2,1,3]);
end
function bound=localNormBound(interval)
    center=mean(interval,3);radius=max(abs(interval-center),[],3);
    candidate=norm(center,2)+1e-10;
    % Validate the proposed center norm by strict diagonal dominance after
    % a numerical orthogonal change of coordinates. Its accuracy is enclosed;
    % the change need not be exactly orthogonal for the SPD implication.
    [basis,~]=eig(center.'*center);matrix=zeros(8,8,2);
    square=localTimes([candidate,candidate],[candidate,candidate]);
    for i=1:8,matrix(i,i,:)=square;end
    for i=1:8
        for j=1:8
            product=[0,0];
            for k=1:8,product=localOut(product+localTimes([center(k,i),center(k,i)],[center(k,j),center(k,j)]));end
            matrix(i,j,:)=localOut(reshape(matrix(i,j,:),1,2)-fliplr(product));
        end
    end
    matrix=localLeftInterval(basis.',localRightInterval(matrix,basis));
    absolute=max(abs(matrix),[],3);diagonal=diag(matrix(:,:,1));
    if any(diagonal<=sum(absolute,2)-diag(absolute)),bound=Inf;return;end
    bound=candidate+norm(radius,'fro')+1e-12;
end
