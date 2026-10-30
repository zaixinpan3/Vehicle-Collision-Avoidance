function result=nrmmJointEnclosure(action,varargin)
%nrmmJointEnclosure Nonlinear outer images for the declared sampled joint map.
% Full-domain affine arithmetic retains shared primitive generators. Every
% nonlinear operation adds a proved real-arithmetic remainder. No sample
% maximum is promoted to a derivative bound. Arithmetic roundoff and model
% residuals outside the declared Fiala parameter family are separate premises.
    switch string(action)
        case "plant"
            result=localPlantFlow(varargin{:});
        case "field"
            result=localPlantField(varargin{:});
        case "acceleration"
            [~,result]=localPlantField(varargin{:});
        case "accelerationHold"
            result=localAccelerationHold(varargin{:});
        case "hold"
            result=localHold(varargin{:});
        case "terminalPose"
            result=localTerminalPose(varargin{:});
        otherwise
            error('nrmmJointEnclosure:invalidAction','Unknown enclosure operation.');
    end
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
        s=min(1,max(0,(x(4)-range(1))/(range(2)-range(1))));
        weight=nrmmSet.intersectRange(s^2*(3-2*s),0,1);
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
        s=min(1,(high(4)-range(1))/(range(2)-range(1)));weight=nrmmSet.intersectRange(s^2*(3-2*s),0,1);
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
    low=-s;high=s;
    for j=1:28
        mid=(low+high)/2;balance=localInverseForce(mid,speed,rate,held,design)-design.egoModel.mass*ay;
        [lo,~]=bounds(balance);
        if lo>0,low=mid;else,high=mid;end
    end
    lower=low;low=-s;high=s;
    for j=1:28
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
    slips=[atan((v+model.lf*rate)/vx)-held(1);atan((v-model.lr*rate)/vx)];
    load=model.mass*model.gravity/(model.lf+model.lr)*[model.lr;model.lf];
    scale=model.frictionCoefficient(:).*load;
    capacity=scale.*sqrt(max(0,1-held(2)^2));
    fy=localTires(slips,nrmmSet(model.corneringStiffness(:)),capacity);
    force=held(2)*scale(1)*sin(held(1))+fy(1)*cos(held(1))+fy(2);
    if nargout<2,return;end
    derivative=nrmmSet(0);offset=[model.lf;-model.lr]*rate;
    for axle=1:2
        [cl,~]=bounds(capacity(axle));
        if cl<=0
            derivative=nrmmSet.range(-realmax,0,size(speed.generator,2));return;
        end
        tangent=tan(slips(axle));q=model.corneringStiffness(axle)*tangent/(3*capacity(axle));
        tireSlope=-model.corneringStiffness(axle)*(1+tangent^2)*max(0,1-abs(q))^2;
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
