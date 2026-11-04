function result=nrmmJointEnclosure(action,varargin)
%nrmmJointEnclosure Nonlinear outer images and exact-flow certificates.
% Full-domain affine arithmetic retains shared primitive generators. Every
% nonlinear operation adds a proved real-arithmetic remainder. No sample
% maximum is promoted to a derivative bound. Arithmetic roundoff and model
% residuals outside the declared Fiala parameter family are separate premises.
% plant/hold enclose the declared numerical sampled map. exactPlant,
% exactObserver and compactCoupledHold include continuous-flow remainders.
    switch string(action)
        case "plant"
            result=localPlantFlow(varargin{:});
        case "field"
            result=localPlantField(varargin{:});
        case "acceleration"
            [~,result]=localPlantField(varargin{:});
        case "accelerationHold"
            result=localAccelerationHold(varargin{:});
        case "exactPlant"
            result=localExactPlant(varargin{:});
        case "stateSlope"
            result=localStateSlope(varargin{:});
        case "targetPhi"
            result=localTargetPhi(varargin{:});
        case "targetObserverResidual"
            result=localTargetObserverResidual(varargin{:});
        case "exactObserver"
            result=localExactObserver(varargin{:});
        case "compactCoupledHold"
            result=localCompactCoupledHold(varargin{:});
        case "hold"
            result=localHold(varargin{:});
        case "terminalPose"
            result=localTerminalPose(varargin{:});
        otherwise
            error('nrmmJointEnclosure:invalidAction','Unknown enclosure operation.');
    end
end

function phi=localTargetPhi(q,s,domain)
% The actual globally Lipschitz NRMM extension, evaluated on a shared set.
    minimum=domain.speedMinimum;
    if minimum==0,minimum=domain.regularizationSpeedMinimum;end
    speed=sqrt(max(0,sum(q.^2)));floor=max(speed,minimum);
    qs=q/max(1,speed/domain.speedMaximum);
    ss=s/max(1,sqrt(max(0,sum(s.^2)))/domain.accelerationNormBound);
    along=min(domain.scalarAccelerationMaximum,max(-domain.scalarAccelerationMaximum,sum(qs.*ss)/floor));
    turn=min(domain.yawRateMaximum,max(-domain.yawRateMaximum, ...
        sum(([-qs(2);qs(1)]).*ss)/floor^2));
    phi=-turn^2.*qs+3*along*turn.*([-q(2);q(1)]/floor);
end

function result=localTargetObserverResidual(target,innovation,rate,design,duration)
% A two-radius Volterra closure bounds Phi(q(t),s(t))-Phi(q(0),s(0)).
% Rotation is retained in the signed core. Only its effect on displacement
% from the initial state enters this local forcing-change bound.
    domain=design.target.domain;
    if domain.speedMinimum==0,domain.speedMinimum=domain.regularizationSpeedMinimum;end
    coefficients=nrmmObserverCertificate('targetLipschitz',domain);
    lq=coefficients.phiVelocity;ls=coefficients.phiAcceleration;
    cross=[0,-1;1,0];a=design.target.innovationGains(:);
    phi=localTargetPhi(target(3:4),target(5:6),design.target.domain);
    qfield=target(5:6)-rate.*(cross*target(3:4))+a(2)*innovation;
    sfield=phi-rate.*(cross*target(5:6))+a(3)*innovation;
    omega=localNormBound(rate);lambda=localNormBound(innovation);
    % State order: displacement q,s; integral jerk, twice integral,
    % three times integral; constant one; elapsed time.
    comparison=zeros(7);comparison(1:2,1:2)=[omega,1;lq,ls+omega];
    comparison(1:2,6)=[localNormBound(qfield);localNormBound(sfield)];
    comparison(1:2,7)=abs(a(2:3))*(abs(a(1))+omega)*lambda;
    comparison(3,1:2)=[lq,ls];comparison(4,3)=1;comparison(5,4)=1;comparison(7,6)=1;
    value=expm(comparison*duration)*[zeros(5,1);1;0];
    result=struct('radius',value([5,4,3]),'displacement',value(1:2), ...
        'comparison',comparison,'phi',phi,'realArithmeticEnclosure',true, ...
        'floatingPointCertified',false);
end

function result=localExactObserver(observer,position,velocity,acceleration,rate,pseudo,design,duration,options)
% Small exact linear maps plus a proved NRMM forcing-change remainder.
% observer = [position(2); yaw; body velocity(2); target(6); predictor(2)].
% Sensor readings and pseudo measurements are held during this sensor slab.
    if nargin<9,options=struct;end
    if ~isfield(options,'maximumStep'),options.maximumStep=.001;end
    validateattributes(duration,{'double'},{'scalar','positive','finite'});
    validateattributes(options.maximumStep,{'double'},{'scalar','positive','finite'});
    count=max(1,ceil(duration/options.maximumStep));h=duration/count;
    kv=design.velocity.gain;kp=design.position.gain;gain=design.yaw.correctionBandwidth;
    a=design.target.innovationGains(:);cross=[0,-1;1,0];
    value=[observer(4:5);observer(6:11);observer(12:13)-observer(6:7)];
    force=acceleration+kv*pseudo.body;original=observer;
    maximum=zeros(3,1);
    for step=1:count
        residual=localTargetObserverResidual(value(3:8),value(9:10),rate,design,h);
        y=[value;force;residual.phi];term=y;sumValue=y;
        for order=1:14
            term=h/order*localObserverLinearField(term,rate,kv,a);sumValue=sumValue+term;
        end
        omega=localNormBound(rate);
        scalar=[-kv,0,0,0,0;-1,0,1,0,a(1);0,0,0,1,a(2);0,0,0,0,a(3);0,0,0,0,-a(1)];
        bound=zeros(14);bound(1:10,1:10)=abs(kron(scalar,eye(2)))+omega*kron(eye(5),abs(cross));
        bound(1:2,11:12)=eye(2);bound(7:8,13:14)=eye(2);
        majorant=localAbsBound(y);
        for order=1:15,majorant=h/order*bound*majorant;end
        ratio=h/16*bound;
        assert(max(abs(eig(ratio)))<1,'nrmmJointEnclosure:seriesFailed', ...
            'The exponential-series remainder requires a smaller certification slab.');
        tail=(eye(14)-ratio)\majorant;
        radius=zeros(14,1);radius(3:8)=repelem(residual.radius,2);
        value=sumValue(1:10)+nrmmSet(zeros(10,1),zeros(10,size(sumValue.generator,2)),tail(1:10)+radius(1:10));
        maximum=maximum+residual.radius;
        yaw=observer(3);field=localObserverYaw(yaw,rate,pseudo,gain);
        % Q's physical norm is <=1; its affine outer box can be wider.
        % Cover that complete outer box as well. No positive-speed or yaw
        % contraction premise is used in this scalar integration bound.
        correspondence=localNormBound(pseudo.rotation);
        yawRemainder=h^2/2*gain*correspondence*(omega+gain*correspondence);
        observer(3)=yaw+h*field+nrmmSet(0,zeros(1,size(yaw.generator,2)),yawRemainder);
    end
    observer(1:2)=position+duration*velocity+exp(-kp*duration)*(original(1:2)-position);
    observer(4:5)=value(1:2);observer(6:11)=value(3:8);observer(12:13)=value(3:4)+value(9:10);
    result=struct('endpoint',observer,'targetLocalRemainderSum',maximum, ...
        'steps',count,'realArithmeticEnclosure',true,'floatingPointCertified',false, ...
        'scope',"exact continuous observer flow under one held sensor block");
end

function value=localObserverLinearField(y,rate,kv,a)
    cross=[0,-1;1,0];
    value=[-kv*y(1:2)-rate.*(cross*y(1:2))+y(11:12); ...
        y(5:6)-y(1:2)-rate.*(cross*y(3:4))+a(1)*y(9:10); ...
        y(7:8)-rate.*(cross*y(5:6))+a(2)*y(9:10); ...
        -rate.*(cross*y(7:8))+a(3)*y(9:10)+y(13:14); ...
        -a(1)*y(9:10)-rate.*(cross*y(9:10));nrmmSet(zeros(4,1))];
end

function value=localAbsBound(x)
    [lo,hi]=bounds(x);value=max(abs(lo),abs(hi));
end

function value=localNormBound(x)
    value=norm(localAbsBound(x));
end

function result=localCompactCoupledHold(center,generator,context,options)
% Construct the full nonlinear joint outer image using separate physical
% slabs and small observer kernels. This is a research certificate evaluator,
% not the ordinary affine RTI approximation or a safety claim for that solve.
    if nargin<4,options=struct;end
    c=context.contract;h=c.samplePeriod;count=round(context.duration/h);
    assert(count>=1 && abs(count*h-context.duration)<1e-10);
    old=size(generator,2);width=old+9*count;
    symbols=nrmmSet.symbolPool('begin',width);
    cleanup=onCleanup(@()nrmmSet.symbolPool('restore',symbols));
    joint=nrmmSet(center,[generator,zeros(35,width-old)]);
    amplitude=[c.noiseRadius;repmat(c.targetObserver.noiseRadius,2,1)];
    columns=zeros(9*count,width);
    for sample=1:count
        if context.first && sample==1,continue;end
        rows=9*(sample-1)+(1:9);columns(rows,old+rows)=diag(amplitude);
    end
    noise=nrmmSet(zeros(9,count),columns);
    physical=joint(1:6);observer=[joint(7:11);joint(28:35)];
    parameters=joint(12:15);previous=joint(16:17);frontend=joint(18:21);target=joint(22:27);
    input=nrmmSet(context.input);
    if ~context.first
        yaw=observer(3);rotation=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
        referenceRotation=[cos(context.referenceYaw),sin(context.referenceYaw);-sin(context.referenceYaw),cos(context.referenceYaw)];
        feature=[-rotation*observer(6:7);yaw;observer(4:5)-referenceRotation*rotation*observer(8:9);physical(6)];
        relative=feature-context.reference;relative(3)=localWrap(relative(3));
        ego=[observer(1:5);physical(6)]-context.egoReference;ego(3)=localWrap(ego(3));
        input=input+context.gain*(ego+[nrmmSet(zeros(5,1));noise(7,1)]) ...
            +context.targetGain*(relative-ego);
    end
    [ul,uh]=bounds(input);
    assert(ul(2)>=-1 && uh(2)<=1,'nrmmJointEnclosure:inputDomain', ...
        'The complete held causal command must be admissible.');
    targetEpoch=target;middle=[];inputRadius=radius(input);steps=0;joined=0;
    for sample=1:count
        ambiguous=false;withoutReset=observer;
        if context.first && sample==1
            m=c.currentMeasurement;
            position=nrmmSet(m.gnssPosition);velocity=nrmmSet(m.gnssVelocity);
            acceleration=nrmmSet(m.bodyAcceleration);rate=nrmmSet(m.yawRate);
            frontend=nrmmSet(c.positionVelocity.point);pseudo=localKnownPseudo(m,c.design);
            detected=m.radarDetectionAvailable;
            if detected,observer(12:13)=nrmmSet(m.radarRelativePosition);end
        else
            measured=input;if sample==1,measured=previous;end
            [~,body]=localPlantField(physical,measured,parameters,context.cfg);
            position=physical(1:2)+noise(1:2,sample);
            acceleration=body+noise(5:6,sample);rate=physical(6)+noise(7,sample);
            frontend=frontend+[c.positionVelocity.gain(1)*eye(2);c.positionVelocity.gain(2)*eye(2)]*(position-frontend(1:2));
            velocity=frontend(3:4);pseudo=localPseudo(velocity,acceleration,rate,measured,c,physical);
            yaw=physical(3);rotation=[cos(yaw),sin(yaw);-sin(yaw),cos(yaw)];
            relative=rotation*(target(1:2)-physical(1:2));
            [rl,rh]=bounds(relative);nearest=max(0,max(rl,-rh));
            inside=norm(max(abs(rl),abs(rh)))<=context.cfg.collision.encounterRangeMeters;
            outside=norm(nearest)>context.cfg.collision.encounterRangeMeters;
            ambiguous=~inside && ~outside;detected=inside || ambiguous;
            if detected,observer(12:13)=relative+noise(8:9,sample);end
        end
        yaw=observer(3);rotation=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
        inertial=rotation*acceleration;
        frontend=[frontend(1:2)+h*frontend(3:4)+h^2*inertial/2;frontend(3:4)+h*inertial];
        design=c.design;if ~detected,design.target.innovationGains(:)=0;end
        observation=localExactObserver(observer,position,velocity,acceleration,rate,pseudo,design,h,options);
        if ambiguous
            noRadar=c.design;noRadar.target.innovationGains(:)=0;
            alternative=localExactObserver(withoutReset,position,velocity,acceleration,rate,pseudo,noRadar,h,options);
            difference=(observation.endpoint-alternative.endpoint)/2;
            average=(observation.endpoint+alternative.endpoint)/2;
            observation.endpoint=average+nrmmSet(zeros(13,1), ...
                zeros(13,size(average.generator,2)),localAbsBound(difference));
            joined=joined+1;
        end
        motion=localExactPlant(physical,input,parameters,context.cfg,h,options);
        physical=motion.endpoint;observer=observation.endpoint;
        target=localExactTarget(targetEpoch,sample*h,context.targetRearAxleDistance);
        steps=steps+motion.steps;
        if 2*sample==count,middle=[physical;target];end
    end
    % The continuous theoretical vehicle has no artificial sample-end stop
    % reset. A discrete implementation wanting that convention must apply
    % and enclose its reset separately.
    image=[physical;observer(1:5);parameters;input;frontend;target;observer(6:13)];
    result=struct('endpoint',image,'midpoint',middle,'input',input,'inputRadius',inputRadius, ...
        'physicalSteps',steps,'sensorSamples',count,'joinedDetectionSamples',joined, ...
        'realArithmeticEnclosure',true,'floatingPointCertified',false, ...
        'futureTargetObserverIncluded',true,'futureTargetFeedbackIncluded',true, ...
        'scope',"exact continuous Fiala/observer flows and sampled measurements; ambiguous detection histories joined");
end

function target=localExactTarget(initial,time,lr)
    speed=initial(4);acceleration=initial(5);[al,ah]=bounds(acceleration);
    [vl,~]=bounds(speed);assert(vl>=0,'nrmmJointEnclosure:domain','Target speed must be nonnegative.');
    moving=time;
    if ah<0
        moving=min(time,speed/(-acceleration));
    elseif al<0
        % Arc is monotone in initial V and A. Retain the initial parameter
        % family, but use its complete arc range when the stop guard crosses.
        [vl,vh]=bounds(speed);[al,ah]=bounds(acceleration);arc=zeros(1,2);
        for j=1:2
            v=[vl,vh];a=[al,ah];t=time;if a(j)<0,t=min(t,-v(j)/a(j));end
            arc(j)=v(j)*t+.5*a(j)*t^2;
        end
        distance=nrmmSet.range(arc(1),arc(2),size(initial.generator,2));
    end
    if ~(al<0 && ah>=0),distance=speed*moving+acceleration*moving^2/2;end
    curvature=sin(initial(6))/lr;half=curvature*distance/2;
    target=initial;direction=initial(3)+initial(6)+half;
    target(1:2)=initial(1:2)+distance*sinc(half)*[cos(direction);sin(direction)];
    target(3)=initial(3)+2*half;target(4)=max(0,speed+time*acceleration);
    % Acceleration is a persistent encounter parameter, including after a
    % stop; only the target's physical forward speed is clipped there.
end

function result=localExactPlant(state,input,parameters,cfg,duration,options)
% Centered Taylor enclosure of the EXACT continuous held-input flow.
% Picard inclusion proves the slab; the explicit state-Jacobian majorant
% proves its time remainder. Input derivatives are never required, even at
% full braking. Shared input/parameter generators survive the signed update.
    if nargin<6,options=struct;end
    if ~isfield(options,'maximumStep'),options.maximumStep=.001;end
    validateattributes(duration,{'double'},{'scalar','positive','finite'});
    validateattributes(options.maximumStep,{'double'},{'scalar','positive','finite'});
    count=max(1,ceil(duration/options.maximumStep));h=duration/count;
    remainder=zeros(6,1);iterations=0;lower=Inf(6,1);upper=-lower;
    accelerationLower=Inf(2,1);accelerationUpper=-accelerationLower;
    for step=1:count
        [xl,xh]=bounds(state);lo=xl;hi=xh;accepted=false;
        for iteration=1:24
            box=nrmmSet.range(lo,hi);
            [field,acceleration]=localPlantField(box,input,parameters,cfg);
            [fl,fh]=bounds(field);
            il=xl+h*min(0,fl);ih=xh+h*max(0,fh);
            if all(il>=lo & ih<=hi),accepted=true;break;end
            lo=min(lo,il);hi=max(hi,ih);
            pad=.1*(hi-lo)+1e-12;lo=lo-pad;hi=hi+pad;
        end
        assert(accepted,'nrmmJointEnclosure:picardFailed', ...
            'The complete physical hold could not be enclosed in its model domain.');
        iterations=iterations+iteration;lower=min(lower,lo);upper=max(upper,hi);
        [al,ah]=bounds(acceleration);
        accelerationLower=min(accelerationLower,al);accelerationUpper=max(accelerationUpper,ah);
        slope=localStateSlope(box,input,parameters,cfg);
        localRemainder=h^2/2*slope*max(abs(fl),abs(fh));
        state=state+h*localPlantField(state,input,parameters,cfg) ...
            +nrmmSet(zeros(6,1),zeros(6,size(state.generator,2)),localRemainder);
        remainder=remainder+localRemainder;
    end
    result=struct('endpoint',state,'slab',nrmmSet.range(lower,upper), ...
        'acceleration',nrmmSet.range(accelerationLower,accelerationUpper), ...
        'localRemainderSum',remainder,'steps',count,'picardIterations',iterations, ...
        'realArithmeticEnclosure',true,'floatingPointCertified',false, ...
        'scope',"exact continuous held-input physical flow; no discrete stop reset");
end

function slope=localStateSlope(x,u,parameters,cfg)
% Explicit componentwise |D_x f| bound on the COMPLETE admitted box.
% Fiala is globally 1-Lipschitz in stress C*t and capacity Q; Q is held
% during a physical segment. There is no division by sqrt(1-b^2).
    [lo,hi]=bounds(x);[ul,uh]=bounds(u);[pl,ph]=bounds(parameters);
    assert(all(pl>-1),'nrmmJointEnclosure:domain','Physical tire factors must remain positive.');
    v=max(abs([lo(4),hi(4)]));vy=max(abs([lo(5),hi(5)]));r=max(abs([lo(6),hi(6)]));
    mass=cfg.vehicle.m;lf=cfg.vehicle.lf;lr=cfg.vehicle.lr;iz=cfg.vehicle.Iz;
    load=localLoadSlope(v,cfg);range=cfg.model.kinematicSpeedRange;
    normal=mass*cfg.vehicle.gravity/(lf+lr)*[lr;lf];
    scale=cfg.tire.frictionCoefficient.*(1+ph(3:4)).*normal;
    stiff=cfg.tire.corneringStiffness.*(1+ph(1:2));
    lag=cfg.model.kinematicLagSeconds;hold=cfg.model.brakeHoldSpeed;
    [~,tangent]=bounds(abs(tan(u(1))));
    kin=[(sum(scale)/hold+load)/mass,r,vy; ...
        lr*tangent/((lf+lr)*lag),1/lag,0; ...
        tangent/((lf+lr)*lag),0,1/lag];
    if lo(4)>=range(2) || hi(4)>range(1)
        % Only the portion where the dynamic branch is used needs its chart.
        dynamic=x;dynamic(4)=nrmmSet.range(max(lo(4),range(1)),hi(4));
        vmin=max(lo(4),range(1));
        slip=[atan((dynamic(5)+lf*dynamic(6))/dynamic(4))-u(1); ...
            atan((dynamic(5)-lr*dynamic(6))/dynamic(4))];
        [al,ah]=bounds(slip);
        assert(all(al>-pi/2 & ah<pi/2),'nrmmJointEnclosure:domain', ...
            'A state derivative certificate requires the complete tire chart.');
        alpha=max(abs(al),abs(ah));gain=stiff./cos(alpha).^2;
        tire=gain.*[(vy+[lf;lr]*r)/vmin^2,ones(2,1)/vmin,[lf;lr]/vmin];
        [~,sine]=bounds(abs(sin(u(1))));[~,cosine]=bounds(abs(cos(u(1))));
        dyn=[sine*tire(1,:)/mass+[load/mass,r,vy]; ...
            (cosine*tire(1,:)+tire(2,:))/mass+[r,0,v]; ...
            (lf*cosine*tire(1,:)+lr*tire(2,:))/iz];
        if lo(4)>=range(2)
            body=dyn;
        else
            body=max(kin,dyn);
            % Product rule for the smooth transition; this is a model
            % derivative bound, not a new tire/actuator constraint.
            [f,~]=localFiala(dynamic,u, ...
                cfg.tire.frictionCoefficient.*(1+parameters(3:4)).*normal, ...
                cfg.tire.corneringStiffness.*(1+parameters(1:2)),cfg);
            [k,~]=localKinematic(dynamic,u, ...
                cfg.tire.frictionCoefficient.*(1+parameters(3:4)).*normal,cfg);
            [dl,dh]=bounds(f-k);
            body(:,1)=body(:,1)+1.5/(range(2)-range(1))*max(abs(dl),abs(dh));
        end
    else
        body=kin;
    end
    slope=zeros(6);slope(1:2,3)=v+vy;slope(1:2,4:5)=1;
    slope(3,6)=1;slope(4:6,4:6)=body;
    assert(all(isfinite(slope),'all') && ul(2)>=-1 && uh(2)<=1, ...
        'nrmmJointEnclosure:domain','A finite whole-domain state slope is required.');
end

function bound=localLoadSlope(speed,cfg)
    r=cfg.roadLoad;
    coefficient=abs(r.rollingCoefficient)+abs(r.rollingSpeedCoefficient)*speed ...
        +abs(r.rollingQuarticCoefficient)*speed^4;
    derivative=abs(r.rollingSpeedCoefficient)+4*abs(r.rollingQuarticCoefficient)*speed^3;
    bound=abs(r.airDensity*r.dragCoefficient*r.frontalArea)*speed ...
        +cfg.vehicle.m*cfg.vehicle.gravity*(derivative+coefficient/r.rollingTransitionSpeed);
end

function result=localAccelerationHold(state,input,parameters,cfg,duration)
    % A Picard inclusion bounds every continuous body-state path over one
    % known actuator hold. It is used to bound the bias of position secants,
    % not to pretend that sampled IMU acceleration is constant in between.
    % Failure leaves the caller's original acceleration-domain bound intact.
    result=struct('valid',true,'normMaximum',0,'yawAccelerationMaximum',0,'iterations',0);
    % The stiff lateral/yaw subsystem needs shorter Picard intervals than
    % its sampled RK flow. Subdivision is for the proof, not a new sensor.
    for substep=1:8
        image=localAccelerationSegment(state,input,parameters,cfg,duration/8);
        result.iterations=result.iterations+image.iterations;
        if ~image.valid,result.valid=false;result.normMaximum=Inf;return;end
        result.normMaximum=max(result.normMaximum,image.normMaximum);
        result.yawAccelerationMaximum=max(result.yawAccelerationMaximum,image.yawAccelerationMaximum);
        state(4:6)=image.endpoint;
    end
end

function result=localAccelerationSegment(state,input,parameters,cfg,duration)
    [initialLower,initialUpper]=bounds(state(4:6));
    boxLower=initialLower;boxUpper=initialUpper;
    result=struct('valid',false,'normMaximum',Inf,'iterations',0);
    for iteration=1:20
        box=[nrmmSet(zeros(3,1));nrmmSet.range(boxLower,boxUpper)];
        try
            [field,acceleration]=localPlantField(box,input,parameters,cfg);
            [lower,upper]=bounds(field(4:6));
            result.yawAccelerationMaximum=max(abs([lower(3),upper(3)]));
        catch exception
            if ~startsWith(string(exception.identifier),["nrmmSet:","nrmmJointEnclosure:"]),rethrow(exception);end
            return;
        end
        imageLower=initialLower+duration*min(0,lower);
        imageUpper=initialUpper+duration*max(0,upper);
        result.iterations=iteration;
        if all(imageLower>=boxLower & imageUpper<=boxUpper)
            [lower,upper]=bounds(acceleration);
            result.valid=true;result.normMaximum=norm(max(abs(lower),abs(upper)));
            [lower,upper]=bounds(field(4:6));
            result.endpoint=nrmmSet.range(initialLower+duration*lower,initialUpper+duration*upper);
            return;
        end
        boxLower=min(boxLower,imageLower);boxUpper=max(boxUpper,imageUpper);
        pad=.1*(boxUpper-boxLower)+1e-10;
        boxLower=boxLower-pad;boxUpper=boxUpper+pad;
    end
end

function next=localPlantFlow(x,u,parameters,cfg,h,first)
    f=@(state)localPlantField(state,u,parameters,cfg);
    if nargin<6,first=f(x);end
    a=first;b=f(x+h*a/2);c=f(x+h*b/2);d=f(x+h*c);
    next=x+h*(a+2*b+2*c+d)/6;
end

function [value,acceleration]=localPlantField(x,u,parameters,cfg)
    [xl,xh]=bounds(x);[ul,uh]=bounds(u);
    if xl(4)<-cfg.model.brakeHoldSpeed || ul(2)<-1 || uh(2)>1
        error('nrmmJointEnclosure:domain','The whole admitted set must lie in the plant domain.');
    end
    [pl,~]=bounds(parameters);
    if any(pl<=-1),error('nrmmJointEnclosure:domain','Nonpositive physical tire parameters.');end
    mass=cfg.vehicle.m;lf=cfg.vehicle.lf;lr=cfg.vehicle.lr;
    normal=mass*cfg.vehicle.gravity/(lf+lr)*[lr;lf];
    scale=cfg.tire.frictionCoefficient.*(1+parameters(3:4)).*normal;
    stiffness=cfg.tire.corneringStiffness.*(1+parameters(1:2));
    range=cfg.model.kinematicSpeedRange;
    if xl(4)>=range(2)
        [body,acceleration]=localFiala(x,u,scale,stiffness,cfg);
    elseif xh(4)<=range(1)
        [body,acceleration]=localKinematic(x,u,scale,cfg);
    elseif xl(4)>cfg.model.scheduleSpeedFloor
        s=(x(4)-range(1))/(range(2)-range(1));
        weight=smoothstep(s);
        [fiala,fialaAcceleration]=localFiala(x,u,scale,stiffness,cfg);
        [kinematic,kinematicAcceleration]=localKinematic(x,u,scale,cfg);
        body=kinematic+weight.*(fiala-kinematic);
        acceleration=kinematicAcceleration+weight.*(fialaAcceleration-kinematicAcceleration);
    else
        % Intersect the two branch domains before evaluating them. An
        % invalid Fiala branch below its floor is never evaluated or dropped.
        low=x;high=x;
        low(4)=nrmmSet.range(xl(4),min(xh(4),range(1)),size(x.generator,2));
        high(4)=nrmmSet.range(range(1),xh(4),size(x.generator,2));
        [a,aa]=localKinematic(low,u,scale,cfg);
        s=(high(4)-range(1))/(range(2)-range(1));weight=smoothstep(s);
        [fiala,fa]=localFiala(high,u,scale,stiffness,cfg);
        [kinematic,ka]=localKinematic(high,u,scale,cfg);
        b=kinematic+weight.*(fiala-kinematic);ba=ka+weight.*(fa-ka);
        [al,ah]=bounds(a);[bl,bh]=bounds(b);
        body=nrmmSet.range(min(al,bl),max(ah,bh),size(a.generator,2));
        [al,ah]=bounds(aa);[bl,bh]=bounds(ba);
        acceleration=nrmmSet.range(min(al,bl),max(ah,bh),size(aa.generator,2));
    end
    value=[x(4)*cos(x(3))-x(5)*sin(x(3)); ...
        x(4)*sin(x(3))+x(5)*cos(x(3));x(6);body];
end

function [body,acceleration]=localFiala(x,u,scale,stiffness,cfg)
    [lo,~]=bounds(x(4));
    if lo<=cfg.model.scheduleSpeedFloor,error('nrmmJointEnclosure:domain','Fiala speed domain is not enclosed.');end
    slip=[atan((x(5)+cfg.vehicle.lf*x(6))/x(4))-u(1);atan((x(5)-cfg.vehicle.lr*x(6))/x(4))];
    [al,ah]=bounds(slip);
    if any(al<=-pi/2 | ah>=pi/2),error('nrmmJointEnclosure:domain','The slip-angle chart is not enclosed.');end
    capacity=scale.*sqrt(max(0,1-u(2)^2));
    lateral=(x(5)+cfg.vehicle.lf*x(6))/x(4);cosine=cos(u(1));sine=sin(u(1));
    tangent=[(lateral*cosine-sine)/(cosine+lateral*sine); ...
        (x(5)-cfg.vehicle.lr*x(6))/x(4)];
    fy=localTires(tangent,stiffness,capacity,true);fx=u(2).*scale;
    frontX=fx(1)*cos(u(1))-fy(1)*sin(u(1));frontY=fx(1)*sin(u(1))+fy(1)*cos(u(1));
    acceleration=[(frontX+fx(2)-localRoadLoad(x(4),cfg))/cfg.vehicle.m;(frontY+fy(2))/cfg.vehicle.m];
    body=[acceleration(1)+x(5)*x(6);acceleration(2)-x(4)*x(6); ...
        (cfg.vehicle.lf*frontY-cfg.vehicle.lr*fy(2))/cfg.vehicle.Iz];
end

function force=localTires(slip,stiffness,capacity,isTangent)
    if nargin<4,isTangent=false;end
    force=nrmmSet(zeros(2,1));
    for i=1:2
        [cl,ch]=bounds(capacity(i));
        if ch==0,force(i)=nrmmSet(0);continue;end
        if cl<=0
            % The force has a finite range at full braking although its
            % input derivative is singular. Use the actual range there.
            [al,ah]=bounds(slip(i));
            if al>=0,lo=-ch;hi=0;elseif ah<=0,lo=0;hi=ch;else,lo=-ch;hi=ch;end
            force(i)=nrmmSet.range(lo,hi,size(capacity.generator,2));
        else
            tangent=slip(i);if ~isTangent,tangent=tan(tangent);end
            value=nrmmSet.fialaForce(stiffness(i)*tangent,capacity(i));
            % This is an exact range of the Fiala formula, not an added
            % tire-force constraint. It prevents polynomial interval
            % dependency from inventing forces beyond its own saturation.
            force(i)=nrmmSet.intersectRange(value,-ch,ch);
        end
    end
end

function [body,acceleration]=localKinematic(x,u,scale,cfg)
    wheelbase=cfg.vehicle.lf+cfg.vehicle.lr;lag=cfg.model.kinematicLagSeconds;
    force=(max(u(2),0)+min(u(2),0)*tanh(x(4)/cfg.model.brakeHoldSpeed)) ...
        *(scale(1)*cos(u(1))+scale(2));
    rate=x(4)*tan(u(1))/wheelbase;
    longitudinal=(force-localRoadLoad(x(4),cfg))/cfg.vehicle.m;
    lateral=(cfg.vehicle.lr*rate-x(5))/lag;
    body=[longitudinal+x(5)*x(6);lateral;(rate-x(6))/lag];
    acceleration=[longitudinal;lateral+x(4)*x(6)];
end

function force=localRoadLoad(speed,cfg)
    r=cfg.roadLoad;magnitude=abs(speed);
    coefficient=r.rollingCoefficient+r.rollingSpeedCoefficient*magnitude+r.rollingQuarticCoefficient*magnitude^4;
    force=.5*r.airDensity*r.dragCoefficient*r.frontalArea*speed*magnitude ...
        +cfg.vehicle.m*cfg.vehicle.gravity*coefficient*tanh(speed/r.rollingTransitionSpeed);
end

function result=localHold(center,generator,context,decisionRadius)
    if nargin<4,decisionRadius=zeros(8,1);end
    decisionCenter=zeros(8,1);
    if isstruct(decisionRadius)
        domain=decisionRadius;
        assert(all(isfinite([domain.lower(:);domain.upper(:)])) && all(domain.lower<=domain.upper));
        decisionCenter=(domain.lower+domain.upper)/2;decisionRadius=(domain.upper-domain.lower)/2;
    end
    validateattributes(decisionRadius,{'double'},{'size',[8,1],'finite','nonnegative'});
    contract=context.contract;samples=round(context.duration/contract.samplePeriod);
    assert(samples>=2 && mod(samples,2)==0 ...
        && abs(samples*contract.samplePeriod-context.duration)<1e-10, ...
        'nrmmJointEnclosure:unalignedSensors','A hold must contain an even positive number of sensor samples.');
    old=size(generator,2);width=old+7*samples+8;
    symbols=nrmmSet.symbolPool('begin',width);
    cleanup=onCleanup(@()nrmmSet.symbolPool('restore',symbols));
    g=[generator,zeros(21,width-old)];joint=nrmmSet(center,g);
    decisions=nrmmSet(decisionCenter,[zeros(8,width-8),diag(decisionRadius)]);
    % Coordinates of the uncertain joint state are relative to the current
    % nominal state correction. Observer pose/body velocity share that
    % correction; parameters and velocity-filter memory retain their values.
    lift=zeros(21,6);lift(1:6,:)=eye(6);lift(7:11,1:5)=eye(5);
    initialLift=lift;
    if context.first,initialLift(7:11,:)=0;end % The current numerical observer is already known.
    joint=joint+initialLift*decisions(1:6);
    physical=joint(1:6);observer=joint(7:11);parameters=joint(12:15);previous=joint(16:17);frontend=joint(18:21);
    input=context.input+decisions(7:8);
    ng=zeros(7*samples,width);
    for i=1:samples
        if context.first&&i==1,continue;end
        ng(7*(i-1)+(1:7),old+7*(i-1)+(1:7))=diag(contract.noiseRadius);
    end
    noise=nrmmSet(zeros(7,samples),ng);
    if ~context.first
        deviation=[observer;physical(6)+noise(7,1)]-context.reference-decisions(1:6);
        deviation(3)=localWrap(deviation(3));input=input+context.gain*deviation;
    end
    [ul,uh]=bounds(input);
    if ul(2)<-1 || uh(2)>1
        error('nrmmJointEnclosure:inputDomain', ...
            'Admitted brake interval [%.17g, %.17g] exceeds [-1,1].',ul(2),uh(2));
    end
    middle=nrmmSet(zeros(6,1));
    for sample=1:samples
        if context.first&&sample==1
            m=contract.currentMeasurement;
            position=nrmmSet(m.gnssPosition);velocity=nrmmSet(m.gnssVelocity);
            acceleration=nrmmSet(m.bodyAcceleration);rate=nrmmSet(m.yawRate);
            frontend=nrmmSet(contract.positionVelocity.point);
            pseudo=localKnownPseudo(m,contract.design);
        else
            measured=input;if sample==1,measured=previous;end
            [measuredField,bodyAcceleration]=localPlantField(physical,measured,parameters,context.cfg);
            position=physical(1:2)+noise(1:2,sample);
            acceleration=bodyAcceleration+noise(5:6,sample);
            rate=physical(6)+noise(7,sample);held=measured;
            gain=contract.positionVelocity.gain;
            frontend=frontend+[gain(1)*eye(2);gain(2)*eye(2)]*(position-frontend(1:2));
            velocity=frontend(3:4);
            pseudo=localPseudo(velocity,acceleration,rate,held,contract,physical);
        end
        dt=contract.samplePeriod;rotation=[cos(observer(3)),-sin(observer(3));sin(observer(3)),cos(observer(3))];
        inertial=rotation*acceleration;
        frontend=[frontend(1:2)+dt*frontend(3:4)+dt^2*inertial/2;frontend(3:4)+dt*inertial];
        observer=localObserver(observer,position,velocity,acceleration,rate,pseudo,contract);
        if sample==1
            physical=localPlantFlow(physical,input,parameters,context.cfg,dt);
        else
            % IMU and the first RK stage use the SAME force realization.
            physical=localPlantFlow(physical,input,parameters,context.cfg,dt,measuredField);
        end
        % Preserve the identity of each new residual across later sensor
        % updates. Repeatedly carrying an unsigned radius through the flow
        % would erase the stabilizing signs of the observer and vehicle.
        packed=promote([physical;observer;parameters;previous;frontend;input]);
        physical=packed(1:6);observer=packed(7:11);parameters=packed(12:15);
        previous=packed(16:17);frontend=packed(18:21);input=packed(22:23);
        if sample==samples/2,middle=physical;end
    end
    physical=localStop(physical,input,context.cfg);
    observer(3)=physical(3)+localWrap(observer(3)-physical(3));
    joint=[physical;observer;parameters;input;frontend];
    if isfield(context,'a')
        % The nominal multiple-shooting equalities, including their defect,
        % determine the next correction. Subtract it from the joint image;
        % the remaining set is an error about every admitted affine iterate.
        nextDelta=context.a*decisions(1:6)+context.b*decisions(7:8)+context.defect;
        joint=joint-lift*nextDelta;
        middle=middle-context.am*decisions(1:6)-context.bm*decisions(7:8);
    end
    feedback=input-context.input-decisions(7:8);
    image=[joint;middle;feedback];
    result=struct('point',image.center,'generator',[image.generator,diag(image.remainder)], ...
        'remainderRadius',image.remainder,'realArithmeticEnclosure',true, ...
        'floatingPointCertified',false,'decisionRadius',decisionRadius, ...
        'scope',"declared sampled ego/observer map; no target observer or external plant residual");
end

function pseudo=localKnownPseudo(input,design)
    geometry=nrmmEgoCourseGeometry('course',input,design);
    lateral=geometry.lateralMeasurement.center;
    if geometry.lateralMeasurement.source=="kinematic" || ~isfinite(lateral),lateral=NaN;end
    velocity=nrmmObserverVectorField('velocityMeasurement',input.gnssVelocity,input.yawRate,design,lateral);
    rotation=geometry.correspondence.rotationVector;
    pseudo=struct('body',nrmmSet(velocity),'rotation',nrmmSet(rotation), ...
        'phase',nrmmSet(atan2(rotation(2),rotation(1))),'magnitude',nrmmSet(norm(rotation)));
end

function pseudo=localPseudo(velocity,acceleration,rate,held,contract,physical)
    design=contract.design;speed=sqrt(max(0,sum(velocity.^2)));
    forceInverse=contract.heldInputAvailable && isfield(design,'egoModel') && ~isempty(design.egoModel);
    if forceInverse
        lateral=localInverse(speed,rate,acceleration(2),held,design,physical);
    else
        lateral=design.yaw.courseModel.rearAxleDistance*rate;
    end
    cosine=cos(design.yaw.courseModel.sideslipDomainMaximum);
    body=[sqrt(max(speed^2-lateral^2,(cosine*speed)^2));lateral];
    scale=design.positionVelocity.bandwidth*design.sensors.positionNoiseMaximum;
    if forceInverse
        % The runtime inverse is clamped to |v_y| <= V*sin(beta_max).
        % Consequently ||body|| == V in real arithmetic. Reuse that
        % identity instead of enclosing the same speed a second time.
        denominator=max(speed^2+scale^2,eps);
    else
        denominator=max(sqrt(max(0,sum(body.^2)))*speed+scale^2,eps);
    end
    [dl,~]=bounds(denominator);
    if dl<=0
        % Cauchy--Schwarz bounds the regularized correspondence even when
        % its separate interval denominator cannot establish positivity.
        rotation=nrmmSet.range(-ones(2,1),ones(2,1),size(body.generator,2));
    else
        rotation=[body(1)*velocity(1)+body(2)*velocity(2);body(1)*velocity(2)-body(2)*velocity(1)]/denominator;
        rotation=nrmmSet.intersectRange(rotation,-ones(2,1),ones(2,1));
    end
    pseudo=struct('body',body,'rotation',rotation,'phase',[],'magnitude',[]);
    % The yaw innovation is a sine of a DIFFERENCE of directions. Computing
    % two uncertain rotations independently loses their shared phase. The
    % local chart below is exact wherever the complete velocity set lies in
    % its open forward half-plane; otherwise retain the vector formula.
    heading=atan2(velocity.center(2),velocity.center(1));
    along=[cos(heading),sin(heading)]*velocity;
    transverse=[-sin(heading),cos(heading)]*velocity;
    [alongLower,~]=bounds(along);[bodyLower,~]=bounds(body(1));
    if alongLower>0 && bodyLower>0 && dl>0
        pseudo.phase=heading+atan(transverse/along)-atan(body(2)/body(1));
        pseudo.magnitude=sqrt(max(0,sum(body.^2)))*speed/denominator;
        if forceInverse,pseudo.magnitude=speed^2/denominator;end
        pseudo.magnitude=nrmmSet.intersectRange(pseudo.magnitude,0,1);
    end
end

function lateral=localInverse(speed,rate,ay,held,design,physical)
    % Enclose the SAME clamped, finite-bisection nominal inverse used online.
    % Normalized lateral velocity has a fixed interval [-sin(beta),sin(beta)].
    % If a predicate is ambiguous, both branches are retained. No physical
    % parameter uncertainty is added to this nominal observer inversion.
    % The range-only bisection discards its temporary expressions. Allocate
    % no residual symbols for them; resume before the shared inverse image.
    symbols=nrmmSet.symbolPool('pause');
    cleanup=onCleanup(@()nrmmSet.symbolPool('restore',symbols));
    s=sin(design.yaw.courseModel.sideslipDomainMaximum);[vl,vh]=bounds(speed);
    if vh==0,lateral=nrmmSet(0);return;end
    if vl<=0,lateral=nrmmSet.range(-vh*s,vh*s,size(speed.generator,2));return;end
    % The runtime bisection presumes a decreasing force balance. Establish
    % that premise on the whole speed/rate/steering box, not at its center:
    % d atan((v+c)/sqrt(V^2-v^2))/dv has the sign of V^2+c*v.
    [rl,rh]=bounds(rate);[ul,uh]=bounds(held);
    model=design.egoModel;rateMaximum=max(abs([rl,rh]));
    monotone=vl>max(model.lf,model.lr)*rateMaximum*s ...
        && ul(1)>-pi/2 && uh(1)<pi/2;
    angleMaximum=atan((vh*s+max(model.lf,model.lr)*rateMaximum) ...
        /(vl*cos(design.yaw.courseModel.sideslipDomainMaximum)));
    monotone=monotone && angleMaximum+max(abs([ul(1),uh(1)]))<pi/2;
    if ~monotone
        % All possible clamped bisection outputs remain in this interval,
        % even where a sharper inverse/monotonicity certificate is absent.
        lateral=nrmmSet.range(-vh*s,vh*s,size(speed.generator,2));return;
    end
    % Resolve each OUTER endpoint to the runtime inverse's physical
    % velocity tolerance. Further bisection would refine a bound far below
    % that existing 1e-5 m/s error without reducing the actual inverse error.
    low=-s;high=s;
    while (high-low)*vh>1e-5
        mid=(low+high)/2;balance=localInverseForce(mid,speed,rate,held,design)-design.egoModel.mass*ay;
        [lo,~]=bounds(balance);
        if lo>0,low=mid;else,high=mid;end
    end
    lower=low;low=-s;high=s;
    while (high-low)*vh>1e-5
        mid=(low+high)/2;balance=localInverseForce(mid,speed,rate,held,design)-design.egoModel.mass*ay;
        [~,hi]=bounds(balance);
        if hi<0,high=mid;else,low=mid;end
    end
    upper=high;
    root=nrmmSet.range(lower,upper,size(speed.generator,2));
    clear cleanup
    % An interval implicit-function step retains the shared IMU/velocity
    % symbols. A box-only inverse would lose their cancellation in the next
    % observer/feedback step. Use it only for a uniformly interior root and
    % a strictly negative derivative on its whole enclosure.
    if lower>-s && upper<s
        try
            % Expand about the shared physical sideslip, rather than an
            % unrelated interval midpoint. This is an enclosure identity;
            % the actual observer still runs its unchanged nominal inverse.
            anchor=physical(5)/sqrt(physical(4)^2+physical(5)^2);
            anchor=min(s,max(-s,anchor));[al,ah]=bounds(anchor);
            domain=nrmmSet.range(min(lower,al),max(upper,ah),size(anchor.generator,2));
            [~,derivative]=localInverseForce(domain,speed,rate,held,design);
            [dl,dh]=bounds(derivative);
            if dh<0
                residual=localInverseForce(anchor,speed,rate,held,design)-model.mass*ay;
                root=anchor-residual/nrmmSet.range(dl,dh,size(speed.generator,2));
                root=nrmmSet.intersectRange(root,lower,upper);
            end
        catch exception
            if ~strcmp(exception.identifier,'nrmmSet:singularDomain'),rethrow(exception);end
            % The previously established clamped root interval remains
            % sound when this optional derivative refinement is unavailable.
        end
    end
    % The runtime returns the right endpoint with <=1e-5 m/s bisection error.
    lateral=speed*root+nrmmSet.range(-1e-5,1e-5,size(speed.generator,2));
end

function [force,derivative]=localInverseForce(s,speed,rate,held,design)
    model=design.egoModel;v=s*speed;vx=sqrt(1-s^2)*speed;
    % Use the SAME rational slip coordinates as the physical Fiala map.
    % Separately enclosing atan(z) followed by tan(atan(z)-delta) creates
    % artificial inverse-observer uncertainty even though the two functions
    % cancel exactly. The whole slip chart is established by localInverse.
    front=(v+model.lf*rate)/vx;cosine=cos(held(1));sine=sin(held(1));
    tangent=[(front*cosine-sine)/(cosine+front*sine);(v-model.lr*rate)/vx];
    load=model.mass*model.gravity/(model.lf+model.lr)*[model.lr;model.lf];
    scale=model.frictionCoefficient(:).*load;
    capacity=scale.*sqrt(max(0,1-held(2)^2));
    fy=localTires(tangent,nrmmSet(model.corneringStiffness(:)),capacity,true);
    force=held(2)*scale(1)*sin(held(1))+fy(1)*cos(held(1))+fy(2);
    resolution=model.mass*design.sensors.accelerometerNoiseMaximum ...
        +model.parameterUncertainty*sum(scale);
    proximal=resolution*speed/max(speed*sin(design.yaw.courseModel.sideslipDomainMaximum),eps);
    force=force-proximal*s;
    if nargout<2,return;end
    derivative=-proximal;offset=[model.lf;-model.lr]*rate;
    for axle=1:2
        [cl,~]=bounds(capacity(axle));
        if cl<=0
            [~,hi]=bounds(derivative);
            derivative=nrmmSet.range(-realmax,hi,size(speed.generator,2));return;
        end
        q=model.corneringStiffness(axle)*tangent(axle)/(3*capacity(axle));
        tireSlope=-model.corneringStiffness(axle)*(1+tangent(axle)^2)*max(0,1-abs(q))^2;
        c=offset(axle);
        angleSlope=speed*(speed+c*s)/(sqrt(1-s^2)*(speed^2+2*c*speed*s+c^2));
        term=tireSlope*angleSlope;if axle==1,term=term*cos(held(1));end
        derivative=derivative+term;
    end
end

function observer=localObserver(observer,position,velocity,acceleration,rate,pseudo,contract)
    % The packed runtime's position predictor obeys q'=velocity and the
    % error p-q obeys e'=-Kp*e during this held measurement. Apply its EXACT
    % RK4 polynomial rather than repeatedly enclosing cancelling q/p terms.
    h=contract.integrationStep;gain=contract.design.position.gain;
    if isscalar(gain),gain=gain*eye(2);end
    a=-h*gain;transition=eye(2)+a+a^2/2+a^3/6+a^4/24;
    observer(1:2)=position+contract.samplePeriod*velocity ...
        +transition^contract.integrationSubstepCount*(observer(1:2)-position);
    body=observer(4:5);yaw=observer(3);
    gain=contract.design.velocity.gain;force=acceleration+gain*pseudo.body;
    f=@(x)force-gain*x-rate*([0,-1;1,0]*x);
    yawField=@(x)localObserverYaw(x,rate,pseudo,contract.design.yaw.correctionBandwidth);
    for k=1:contract.integrationSubstepCount
        a=f(body);b=f(body+h*a/2);c=f(body+h*b/2);d=f(body+h*c);
        body=body+h*(a+2*b+2*c+d)/6;
        a=yawField(yaw);b=yawField(yaw+h*a/2);c=yawField(yaw+h*b/2);d=yawField(yaw+h*c);
        yaw=yaw+h*(a+2*b+2*c+d)/6;
        % Working on the continuous yaw lift is equivalent to wrapping
        % after each substep; only periodic sin/cos consume this coordinate.
    end
    observer(3)=yaw;observer(4:5)=body;
end

function field=localObserverYaw(yaw,rate,pseudo,gain)
    if isempty(pseudo.phase)
        innovation=pseudo.rotation(2)*cos(yaw)-pseudo.rotation(1)*sin(yaw);
    else
        innovation=pseudo.magnitude*sin(pseudo.phase-yaw);
    end
    field=rate+gain*innovation;
end

function x=localStop(x,u,cfg)
    [lo,hi]=bounds(x);[ul,uh]=bounds(u);threshold=.2*cfg.model.brakeHoldSpeed;
    if uh(2)<=0 && hi(4)<threshold,x(4:6)=nrmmSet(zeros(3,1));
    elseif ul(2)<=0 && lo(4)<threshold
        x(4:6)=nrmmSet.range(min(0,lo(4:6)),max(0,hi(4:6)),size(x.generator,2));
    end
end

function a=localWrap(a)
    [lo,hi]=bounds(a);branch=floor((a.center+pi)/(2*pi));
    if lo>=-pi+2*pi*branch && hi<pi+2*pi*branch
        a=a-2*pi*branch;
    else
        a=nrmmSet.range(-pi,pi,size(a.generator,2));
    end
end

function pose=localTerminalPose(x,context,time)
    validateattributes(time,{'double'},{'scalar','finite','nonnegative'});
    validateattributes(context.deceleration,{'double'},{'scalar','finite','positive'});
    speed=sqrt(max(0,x(4)^2+x(5)^2));[vl,vh]=bounds(speed);[xl,~]=bounds(x);
    if vh==0,pose=x(1:3);return;end
    if vl<=0 || xl(4)<=0
        % A finite reachable disk and turn bound enclose the
        % zero-speed chart. This is conservative, not an omitted branch.
        displacement=vh^2/(2*context.deceleration);
        [rl,rh]=bounds(x(6));turn=max(abs([rl,rh]))*vh/(2*context.deceleration);
        pose=[x(1:2)+nrmmSet.range(-displacement*ones(2,1),displacement*ones(2,1),size(x.generator,2)); ...
            x(3)+nrmmSet.range(-turn,turn,size(x.generator,2))];return;
    end
    beta=atan(x(5)/x(4));moving=min(time,speed/context.deceleration);
    arc=speed*moving-context.deceleration*moving^2/2;bend=x(6)*arc/speed;
    direction=x(3)+beta+bend/2;
    pose=[x(1:2)+arc*sinc(bend/2)*[cos(direction);sin(direction)];x(3)+bend];
end
