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
        case "hold"
            result=localHold(varargin{:});
        case "terminalPose"
            result=localTerminalPose(varargin{:});
        otherwise
            error('nrmmJointEnclosure:invalidAction','Unknown enclosure operation.');
    end
end

function next=localPlantFlow(x,u,parameters,cfg,h)
    f=@(state)localPlantField(state,u,parameters,cfg);
    a=f(x);b=f(x+h*a/2);c=f(x+h*b/2);d=f(x+h*c);
    next=x+h*(a+2*b+2*c+d)/6;
end

function value=localPlantField(x,u,parameters,cfg)
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
        body=localFiala(x,u,scale,stiffness,cfg);
    elseif xh(4)<=range(1)
        body=localKinematic(x,u,scale,cfg);
    elseif xl(4)>cfg.model.scheduleSpeedFloor
        s=min(1,max(0,(x(4)-range(1))/(range(2)-range(1))));
        weight=nrmmSet.intersectRange(s^2*(3-2*s),0,1);
        body=weight.*localFiala(x,u,scale,stiffness,cfg)+(1-weight).*localKinematic(x,u,scale,cfg);
    else
        % Intersect the two branch domains before evaluating them. An
        % invalid Fiala branch below its floor is never evaluated or dropped.
        low=x;high=x;
        low(4)=nrmmSet.range(xl(4),min(xh(4),range(1)),size(x.generator,2));
        high(4)=nrmmSet.range(range(1),xh(4),size(x.generator,2));
        a=localKinematic(low,u,scale,cfg);
        s=min(1,(high(4)-range(1))/(range(2)-range(1)));weight=nrmmSet.intersectRange(s^2*(3-2*s),0,1);
        b=weight.*localFiala(high,u,scale,stiffness,cfg)+(1-weight).*localKinematic(high,u,scale,cfg);
        [al,ah]=bounds(a);[bl,bh]=bounds(b);
        body=nrmmSet.range(min(al,bl),max(ah,bh),size(a.generator,2));
    end
    value=[x(4)*cos(x(3))-x(5)*sin(x(3)); ...
        x(4)*sin(x(3))+x(5)*cos(x(3));x(6);body];
end

function body=localFiala(x,u,scale,stiffness,cfg)
    [lo,~]=bounds(x(4));
    if lo<=cfg.model.scheduleSpeedFloor,error('nrmmJointEnclosure:domain','Fiala speed domain is not enclosed.');end
    slip=[atan((x(5)+cfg.vehicle.lf*x(6))/x(4))-u(1);atan((x(5)-cfg.vehicle.lr*x(6))/x(4))];
    [al,ah]=bounds(slip);
    if any(al<=-pi/2 | ah>=pi/2),error('nrmmJointEnclosure:domain','The slip-angle chart is not enclosed.');end
    capacity=scale.*sqrt(max(0,1-u(2)^2));
    fy=localTires(slip,stiffness,capacity);fx=u(2).*scale;
    frontX=fx(1)*cos(u(1))-fy(1)*sin(u(1));frontY=fx(1)*sin(u(1))+fy(1)*cos(u(1));
    body=[(frontX+fx(2)-localRoadLoad(x(4),cfg))/cfg.vehicle.m+x(5)*x(6); ...
        (frontY+fy(2))/cfg.vehicle.m-x(4)*x(6); ...
        (cfg.vehicle.lf*frontY-cfg.vehicle.lr*fy(2))/cfg.vehicle.Iz];
end

function force=localTires(slip,stiffness,capacity)
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
            q=stiffness(i)*tan(slip(i))/(3*capacity(i));
            value=-capacity(i)*fialaFraction(q);
            % This is an exact range of the Fiala formula, not an added
            % tire-force constraint. It prevents polynomial interval
            % dependency from inventing forces beyond its own saturation.
            force(i)=nrmmSet.intersectRange(value,-ch,ch);
        end
    end
end

function body=localKinematic(x,u,scale,cfg)
    wheelbase=cfg.vehicle.lf+cfg.vehicle.lr;lag=cfg.model.kinematicLagSeconds;
    force=(max(u(2),0)+min(u(2),0)*tanh(x(4)/cfg.model.brakeHoldSpeed)) ...
        *(scale(1)*cos(u(1))+scale(2));
    rate=x(4)*tan(u(1))/wheelbase;
    body=[(force-localRoadLoad(x(4),cfg))/cfg.vehicle.m+x(5)*x(6); ...
        (cfg.vehicle.lr*rate-x(5))/lag;(rate-x(6))/lag];
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
            field=localPlantField(physical,measured,parameters,context.cfg);
            position=physical(1:2)+noise(1:2,sample);
            acceleration=[field(4)-physical(6)*physical(5);field(5)+physical(6)*physical(4)]+noise(5:6,sample);
            rate=physical(6)+noise(7,sample);held=measured;
            gain=contract.positionVelocity.gain;
            frontend=frontend+[gain(1)*eye(2);gain(2)*eye(2)]*(position-frontend(1:2));
            velocity=frontend(3:4);
            pseudo=localPseudo(velocity,acceleration,rate,held,contract);
        end
        dt=contract.samplePeriod;rotation=[cos(observer(3)),-sin(observer(3));sin(observer(3)),cos(observer(3))];
        inertial=rotation*acceleration;
        frontend=[frontend(1:2)+dt*frontend(3:4)+dt^2*inertial/2;frontend(3:4)+dt*inertial];
        observer=localObserver(observer,position,velocity,acceleration,rate,pseudo,contract);
        physical=localPlantFlow(physical,input,parameters,context.cfg,dt);
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
    pseudo=nrmmSet([velocity;geometry.correspondence.rotationVector]);
end

function pseudo=localPseudo(velocity,acceleration,rate,held,contract)
    design=contract.design;speed=sqrt(max(0,sum(velocity.^2)));
    if contract.heldInputAvailable && isfield(design,'egoModel') && ~isempty(design.egoModel)
        lateral=localInverse(speed,rate,acceleration(2),held,design);
    else
        lateral=design.yaw.courseModel.rearAxleDistance*rate;
    end
    cosine=cos(design.yaw.courseModel.sideslipDomainMaximum);
    body=[sqrt(max(speed^2-lateral^2,(cosine*speed)^2));lateral];
    scale=design.positionVelocity.bandwidth*design.sensors.positionNoiseMaximum;
    denominator=max(sqrt(max(0,sum(body.^2)))*speed+scale^2,eps);
    [dl,~]=bounds(denominator);
    if dl<=0
        % Cauchy--Schwarz bounds the regularized correspondence even when
        % its separate interval denominator cannot establish positivity.
        rotation=nrmmSet.range(-ones(2,1),ones(2,1),size(body.generator,2));
    else
        rotation=[body(1)*velocity(1)+body(2)*velocity(2);body(1)*velocity(2)-body(2)*velocity(1)]/denominator;
        rotation=nrmmSet.intersectRange(rotation,-ones(2,1),ones(2,1));
    end
    pseudo=[body;rotation];
end

function lateral=localInverse(speed,rate,ay,held,design)
    % Enclose the SAME clamped, finite-bisection nominal inverse used online.
    % Normalized lateral velocity has a fixed interval [-sin(beta),sin(beta)].
    % If a predicate is ambiguous, both branches are retained. No physical
    % parameter uncertainty is added to this nominal observer inversion.
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
    % An interval implicit-function step retains the shared IMU/velocity
    % symbols. A box-only inverse would lose their cancellation in the next
    % observer/feedback step. Use it only for a uniformly interior root and
    % a strictly negative derivative on its whole enclosure.
    if lower>-s && upper<s
        try
            [~,derivative]=localInverseForce(root,speed,rate,held,design);
            [dl,dh]=bounds(derivative);
            if dh<0
                middle=(lower+upper)/2;
                residual=localInverseForce(middle,speed,rate,held,design)-model.mass*ay;
                root=middle-residual/nrmmSet.range(dl,dh,size(speed.generator,2));
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
    state=[observer(4:5);observer(1:2);position;observer(3)];
    f=@(x)localObserverField(x,velocity,acceleration,rate,pseudo,contract.design);
    for k=1:contract.integrationSubstepCount
        h=contract.integrationStep;a=f(state);b=f(state+h*a/2);c=f(state+h*b/2);d=f(state+h*c);
        state=state+h*(a+2*b+2*c+d)/6;
        % Working on the continuous yaw lift is equivalent to wrapping
        % after each substep; only periodic sin/cos consume this coordinate.
    end
    observer=state([3,4,7,1,2]);
end

function field=localObserverField(x,velocity,acceleration,rate,pseudo,design)
    cross=[0,-1;1,0];gain=design.yaw.correctionBandwidth;
    field=[acceleration-rate*(cross*x(1:2))+design.velocity.gain*(pseudo(1:2)-x(1:2)); ...
        velocity+design.position.gain*(x(5:6)-x(3:4));velocity; ...
        rate+gain*(pseudo(4)*cos(x(7))-pseudo(3)*sin(x(7)))];
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
