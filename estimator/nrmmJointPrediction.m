function value = nrmmJointPrediction(action,varargin)
%nrmmJointPrediction Lift the sampled plant, observer and future feedback.
% State: [physical ego (6); numerical ego observer (5); tire factors (4);
% previous held input (2); position/IMU filter prior (4)]. At a control boundary the IMU still measures the
% previous input; the new command drives the subsequent plant integration.
% Five real noise channels are position (2), acceleration (2), and gyro (1).
% Slots 3:4 of the legacy seven-slot noise array are zero placeholders.
% A parameter column is introduced once and retained across the whole horizon.
% Future sensor errors enter once per measurement, shared by the observer and
% feedback. The actual current measurement and first command are conditioned
% data. Supports are exact for the constructed joint affine maps, NOT certified
% nonlinear reachable sets. No current observer budget is reset in the horizon.
    switch string(action)
        case "contract"
            value=localContract(varargin{:});
        case "observerStep"
            value=localObserverStep(varargin{:});
        case "propagate"
            value=localPropagate(varargin{:});
        case "hold"
            value=localHold(varargin{:});
        case "linearize"
            [point,matrix,noise]=localLinearize(varargin{:});
            value=struct('point',point,'matrix',matrix,'noise',noise);
        case "finiteDifference"
            [point,matrix,noise]=localFiniteDifference(varargin{:});
            value=struct('point',point,'matrix',matrix,'noise',noise);
        case "resetEnvelope"
            [point,generator,mixed]=localResetEnvelope(varargin{:});
            value=struct('point',point,'generator',generator,'mixed',mixed);
        otherwise
            error('nrmmJointPrediction:invalidAction','Unknown joint prediction action.');
    end
end

function contract=localContract(runtime,input)
    design=runtime.observerDesign;
    sensors=design.sensors;
    radius=0;
    if isfield(design,'egoModel') && ~isempty(design.egoModel)
        radius=design.egoModel.parameterUncertainty;
    end
    contract=struct('kind',"sampled-nrmm-joint-v2",'time',input.time, ...
        'point',[runtime.positionEstimate;runtime.yawEstimate;runtime.bodyVelocityEstimate], ...
        'samplePeriod',runtime.samplePeriod,'integrationStep',runtime.integrationStep, ...
        'integrationSubstepCount',runtime.integrationSubstepCount, ...
        'design',design,'currentMeasurement',input, ...
        'positionVelocity',input.positionVelocity,'sensorContract',"position-only-GNSS", ...
        'heldInputAvailable',numel(input.heldInput)==2, ...
        'noiseRadius',[repmat(sensors.positionNoiseMaximum,2,1); ...
            zeros(2,1); ...
            repmat(sensors.accelerometerNoiseMaximum,2,1);sensors.gyroscopeNoiseMaximum], ...
        'parameterRadius',repmat(radius,4,1), ...
        'parameterOrder',["frontCornering";"rearCornering";"frontFriction";"rearFriction"], ...
        'physicalModel',"declaredFialaWithPersistentTireParameters", ...
        'nonlinearCertificate',false);
end

function point=localObserverStep(point,input,contract)
    % Exactly the ego projection of the runtime's packed RK4 update. Target
    % states do not drive this projection. Retain the resetting GNSS predictor.
    design=contract.design;
    if ~isfield(input,'courseGeometry')
        input.courseGeometry=nrmmEgoCourseGeometry('course',input,design);
    end
    geometry=input.courseGeometry;correspondence=geometry.correspondence;
    lateral=geometry.lateralMeasurement.center;
    if geometry.lateralMeasurement.source=="kinematic" || ~isfinite(lateral),lateral=NaN;end
    velocity=nrmmObserverVectorField('velocityMeasurement',input.gnssVelocity,input.yawRate,design,lateral);
    state=[point(4:5);point(1:2);input.gnssPosition;point(3)];
    for j=1:contract.integrationSubstepCount
        h=contract.integrationStep;
        first=localObserverField(state,input,velocity,correspondence,design);
        second=localObserverField(state+h*first/2,input,velocity,correspondence,design);
        third=localObserverField(state+h*second/2,input,velocity,correspondence,design);
        fourth=localObserverField(state+h*third,input,velocity,correspondence,design);
        state=state+h*(first+2*second+2*third+fourth)/6;
        state(7)=atan2(sin(state(7)),cos(state(7)));
    end
    point=[state(3:4);state(7);state(1:2)];
end

function derivative=localObserverField(state,input,velocity,correspondence,design)
    cross=[0,-1;1,0];
    body=input.bodyAcceleration-input.yawRate*cross*state(1:2) ...
        +design.velocity.gain*(velocity-state(1:2));
    position=input.gnssVelocity+design.position.gain*(state(5:6)-state(3:4));
    yaw=nrmmObserverVectorField('yawDerivative',state(7),input.yawRate,correspondence,design);
    derivative=[body;position;input.gnssVelocity;yaw];
end

function tube=localPropagate(anchor,cfg,initialGenerator,contract,durations,options)
    if nargin<6,options=struct();end
    validateattributes(contract.samplePeriod,{'double'},{'scalar','finite','positive'});
    validateattributes(contract.integrationStep,{'double'},{'scalar','finite','positive'});
    validateattributes(contract.integrationSubstepCount,{'double'},{'scalar','integer','positive'});
    assert(abs(contract.integrationStep*contract.integrationSubstepCount-contract.samplePeriod)<1e-10, ...
        'nrmmJointPrediction:invalidSensorClock','The RK4 substeps must span one actual observer sample.');
    if isfield(contract.design,'egoModel') && ~isempty(contract.design.egoModel)
        declared=contract.design.egoModel;
        first=[declared.mass;declared.lf;declared.lr;declared.gravity;declared.corneringStiffness(:);declared.frictionCoefficient(:)];
        second=[cfg.vehicle.m;cfg.vehicle.lf;cfg.vehicle.lr;cfg.vehicle.gravity;cfg.tire.corneringStiffness(:);cfg.tire.frictionCoefficient(:)];
        assert(isequal(first,second),'nrmmJointPrediction:inconsistentPhysicalModel', ...
            'The observer parameter contract and predicted vehicle must share the same nominal physical model.');
    end
    count=size(anchor.inputs,2);
    assert(numel(durations)==count,'nrmmJointPrediction:invalidGrid','One duration per hold is required.');
    ratio=durations/contract.samplePeriod;
    assert(all(abs(ratio-round(ratio))<1e-8) && all(mod(round(ratio),2)==0), ...
        'nrmmJointPrediction:unalignedSensors','Each hold needs an even positive number of observer samples.');
    assert(all(ratio>=2),'nrmmJointPrediction:unalignedSensors','Observer samples must resolve the hold midpoint.');
    gains=localFeedback(anchor,cfg,durations);
    if isfield(options,'feedback') && ~options.feedback,gains(:)=0;end
    rootColumns=size(initialGenerator,2);
    previous=contract.currentMeasurement.heldInput;
    if isempty(previous),previous=anchor.inputs(:,1);end
    generators=[initialGenerator,zeros(6,4);zeros(5,rootColumns+4); ...
        zeros(4,rootColumns),diag(contract.parameterRadius);zeros(2,rootColumns+4);zeros(4,rootColumns+4)];
    center=[anchor.states(:,1);contract.point;zeros(4,1);previous;contract.positionVelocity.point];
    estimation=[-eye(5),zeros(5,1),eye(5),zeros(5,10)];
    records=2*count+1;
    tube=struct('kind',"joint-sampled-affine-tube-v2", ...
        'generator',{cell(1,records)},'observerErrorRadius',zeros(5,records), ...
        'physicalRadius',zeros(6,records),'mean',zeros(21,count+1), ...
        'inputRadius',zeros(2,count),'inputGenerator',{cell(1,count)}, ...
        'feedbackGain',gains,'transition',{cell(1,count)},'noiseMap',{cell(1,count)}, ...
        'currentInputUncertaintyRadius',zeros(2,1),'currentMeasurementConditioned',true, ...
        'futureObserverDynamicsIncluded',true,'futurePrimitiveBudgetsAssumed',false, ...
        'persistentParameterColumns',rootColumns+(1:4),'nonlinearCertificate',false, ...
        'scope',"exact supports of joint affine maps; nonlinear remainders not certified", ...
        'mode',"causalObserverFeedback",'time',[0,cumsum(durations)], ...
        'mixedStopBranches',false(1,count));
    tube.generator{1}=initialGenerator;
    tube.physicalRadius(:,1)=sum(abs(initialGenerator),2);
    tube.observerErrorRadius(:,1)=abs(estimation*center)+sum(abs(estimation*generators),2);
    tube.mean(:,1)=center;
    for k=1:count
        if k>1
            estimateGenerators=generators([7:11,6],:);
            reserve=sum(abs(gains(2,:,k)*estimateGenerators))+abs(gains(2,6,k))*contract.noiseRadius(7);
            authority=min(cfg.actuation.brakingRatioMaximum-anchor.inputs(2,k), ...
                anchor.inputs(2,k)-cfg.actuation.brakingRatioMinimum);
            if reserve>0,gains(2,:,k)=gains(2,:,k)*min(1,max(0,authority)/reserve);end
            % Ancillary steering is useful only inside the existing bicycle
            % chart. Reserve its authority before computing the tube; do not
            % clip the input after propagating a different feedback policy.
            radius=sum(abs(generators(1:6,:)),2);
            front=abs(center(5)+cfg.vehicle.lf*center(6))+radius(5)+cfg.vehicle.lf*radius(6);
            course=atan2(front,max(0,center(4)-radius(4)));
            authority=max(0,pi/2-course-abs(anchor.inputs(1,k))-cfg.solver.constraintTolerance);
            reserve=sum(abs(gains(1,:,k)*estimateGenerators))+abs(gains(1,6,k))*contract.noiseRadius(7);
            if reserve>0,gains(1,:,k)=gains(1,:,k)*min(1,authority/reserve);end
        end
        % Feedback is centered on the nominal numerical observer, which can
        % have a deterministic model bias relative to the nominal vehicle.
        % Uncertain deviations drive feedback; a known observer bias is not
        % silently turned into a different nominal command.
        context=struct('cfg',cfg,'contract',contract,'reference',[center(7:11);center(6)], ...
            'input',anchor.inputs(:,k),'gain',gains(:,:,k),'duration',durations(k), ...
            'first',k==1);
        [~,matrix,noise,branch]=localLinearize(center,context);
        beforeReset=[branch.matrix*generators,branch.noise];
        [point,nextGenerators,mixed]=localResetEnvelope(branch.point,beforeReset,cfg);
        if mixed
            matrix=branch.matrix;noise=branch.noise;
            matrix(4:6,:)=matrix(4:6,:)/2;noise(4:6,:)=noise(4:6,:)/2;
        end
        tube.mixedStopBranches(k)=mixed;
        middle=point(22:27);next=point(1:21);input=point(28:29);
        middleReference=localReferenceMiddle(anchor,k,cfg,durations(k));
        tube.generator{2*k}=[nextGenerators(22:27,:),middle-middleReference];
        tube.generator{2*k+1}=[nextGenerators(1:6,:),next(1:6)-anchor.states(:,k+1)];
        tube.physicalRadius(:,2*k:2*k+1)=[sum(abs(tube.generator{2*k}),2),sum(abs(tube.generator{2*k+1}),2)];
        tube.observerErrorRadius(:,2*k)=NaN; % No fictitious midpoint posterior.
        tube.observerErrorRadius(:,2*k+1)=abs(estimation*next)+sum(abs(estimation*nextGenerators(1:21,:)),2);
        tube.inputGenerator{k}=nextGenerators(28:29,:);
        tube.inputRadius(:,k)=abs(input-anchor.inputs(:,k))+sum(abs(tube.inputGenerator{k}),2);
        tube.transition{k}=matrix(1:21,:);tube.noiseMap{k}=noise(1:21,:);
        generators=nextGenerators(1:21,:);center=next;
        tube.mean(:,k+1)=center;
    end
    tube.feedbackGain=gains;
    tube.finalJointGenerator=generators;
    tube.firstInputRadius=tube.inputRadius(:,1);
    tube.egoStateRadius=tube.physicalRadius;
    tube.observerErrorTime=tube.time;
    tube.observerErrorRadius=tube.observerErrorRadius(:,1:2:end);
end

function middle=localReferenceMiddle(anchor,k,cfg,duration)
    cfg.controller.sampleTime=duration;
    middle=nonlinearBicycleModel.hold(anchor.states(:,k),anchor.inputs(:,k),cfg);
end

function gains=localFeedback(anchor,cfg,durations)
    % Finite-horizon ancillary feedback, separately represented from the RTI
    % nominal commands. Full position feedback avoids an uncontrolled drift
    % mode. These are design weights, not claimed contraction certificates.
    count=numel(durations);gains=zeros(2,6,count);p=diag([1,4,20,1,1,2]);
    q=p;r=diag([20,1]);
    for k=count:-1:1
        [~,a,b]=nonlinearBicycleModel.sample(anchor.states(:,k),anchor.inputs(:,k),cfg,[],durations(k));
        gains(:,:,k)=-(r+b'*p*b)\(b'*p*a);
        if abs(anchor.inputs(2,k))==1,gains(2,:,k)=0;end
        p=q+a'*p*a+a'*p*b*gains(:,:,k);p=(p+p')/2;
    end
end

function [point,matrix,noise]=localFiniteDifference(center,context)
    samples=round(context.duration/context.contract.samplePeriod);
    zero=zeros(7,samples);point=localHold(center,zero,context);
    matrix=zeros(numel(point),21);noise=zeros(numel(point),7*samples);
    steps=[1e-3;1e-3;1e-4;.005;.005;.0005;1e-3;1e-3;1e-4;.005;.005;repmat(.0025,4,1);1e-4;1e-3;1e-3;1e-3;.005;.005];
    for j=1:21
        delta=zeros(21,1);delta(j)=steps(j);
        if ~context.first
            selector=zeros(6,21);selector(1:5,7:11)=eye(5);selector(6,6)=1;
            slope=abs(context.gain(2,:)*selector(:,j));
            if slope>0
                steps(j)=min(steps(j),max(eps,.25*(1-abs(context.input(2)))/slope));delta(j)=steps(j);
            end
        end
        matrix(:,j)=localDifference(localHold(center+delta,zero,context),localHold(center-delta,zero,context))/(2*steps(j));
    end
    amplitudes=context.contract.noiseRadius;
    for j=1:7*samples
        sample=ceil(j/7);channel=mod(j-1,7)+1;
        if (context.first && sample==1) || amplitudes(channel)==0,continue;end
        delta=zero;delta(j)=.1*amplitudes(channel);
        noise(:,j)=localDifference(localHold(center,delta,context), ...
            localHold(center,-delta,context))/.2;
    end
end

function [point,matrix,noise,branch]=localLinearize(center,context)
    % Chain the plant and observer tangents on the sensor clock. The small
    % measurement inverse is differentiated once per sample; do not rerun an
    % entire hold for every state and every noise coordinate.
    contract=context.contract;cfg=context.cfg;
    samples=round(context.duration/contract.samplePeriod);dimension=21+7*samples;
    x=center(1:6);observer=center(7:11);parameters=center(12:15);previous=center(16:17);frontend=center(18:21);
    dx=[eye(6),zeros(6,dimension-6)];dobserver=zeros(5,dimension);dobserver(:,7:11)=eye(5);
    dp=zeros(4,dimension);dp(:,12:15)=eye(4);
    dprevious=zeros(2,dimension);dprevious(:,16:17)=eye(2);
    dfrontend=zeros(4,dimension);dfrontend(:,18:21)=eye(4);
    plant=localPlantConfiguration(cfg,parameters);
    u=context.input;du=zeros(2,dimension);
    if ~context.first
        difference=[observer;x(6)]-context.reference;
        difference(3)=atan2(sin(difference(3)),cos(difference(3)));
        u=u+context.gain*difference;
        du=context.gain*[dobserver;dx(6,:)];
        du(:,28)=du(:,28)+context.gain(:,6)*contract.noiseRadius(7);
    end
    middle=zeros(6,1);dmiddle=zeros(6,dimension);
    for sample=1:samples
        dm=zeros(9,dimension);
        if context.first && sample==1
            measurement=contract.currentMeasurement;
            frontend=contract.positionVelocity.point;dfrontend(:)=0;
        else
            measuredInput=u;dmeasuredInput=du;
            if sample==1,measuredInput=previous;dmeasuredInput=dprevious;end
            measurement=localMeasurement(x,measuredInput,zeros(7,1),plant,contract);
            [~,df]=localPlantFieldTangent(x,measuredInput,plant,cfg,dx,dmeasuredInput,dp);
            dm(1:2,:)=dx(1:2,:);

            dm(5,:)=df(4,:)-x(5)*dx(6,:)-x(6)*dx(5,:);
            dm(6,:)=df(5,:)+x(4)*dx(6,:)+x(6)*dx(4,:);
            dm(7,:)=dx(6,:);
            if contract.heldInputAvailable,dm(8:9,:)=dmeasuredInput;end
            columns=21+(sample-1)*7+(1:7);
            dm(1:7,columns)=dm(1:7,columns)+diag(contract.noiseRadius);
        end
        if ~(context.first && sample==1)
            gain=contract.positionVelocity.gain;
            correction=[gain(1)*eye(2);gain(2)*eye(2)];
            dfrontend=dfrontend+correction*(dm(1:2,:)-dfrontend(1:2,:));
            frontend=nrmmPositionVelocity('pointMeasure',frontend,measurement.gnssPosition,gain);
            measurement.gnssVelocity=frontend(3:4);dm(3:4,:)=dfrontend(3:4,:);
        end
        dt=contract.samplePeriod;rotation=[cos(observer(3)),-sin(observer(3));sin(observer(3)),cos(observer(3))];
        da=rotation*dm(5:6,:)+rotation*[-measurement.bodyAcceleration(2);measurement.bodyAcceleration(1)]*dobserver(3,:);
        frontend=nrmmPositionVelocity('pointPredict',frontend,measurement.bodyAcceleration,observer(3),dt);
        dfrontend=[dfrontend(1:2,:)+dt*dfrontend(3:4,:)+dt^2*da/2;dfrontend(3:4,:)+dt*da];
        [observer,dobserver]=localObserverTangent(observer,dobserver,measurement,dm,contract);
        h=contract.samplePeriod;
        [f1,d1]=localPlantFieldTangent(x,u,plant,cfg,dx,du,dp);
        [f2,d2]=localPlantFieldTangent(x+h*f1/2,u,plant,cfg,dx+h*d1/2,du,dp);
        [f3,d3]=localPlantFieldTangent(x+h*f2/2,u,plant,cfg,dx+h*d2/2,du,dp);
        [f4,d4]=localPlantFieldTangent(x+h*f3,u,plant,cfg,dx+h*d3,du,dp);
        x=x+h*(f1+2*f2+2*f3+f4)/6;dx=dx+h*(d1+2*d2+2*d3+d4)/6;
        if sample==samples/2,middle=x;dmiddle=dx;end
    end
    before=[x;observer;parameters;u;frontend;middle;u];beforeGradient=[dx;dobserver;dp;du;dfrontend;dmiddle;du];
    branch=struct('point',before,'matrix',beforeGradient(:,1:21), ...
        'noise',beforeGradient(:,22:end));
    [x,stopped]=nonlinearBicycleModel.sampleEndState(x,u,plant);
    if stopped,dx(4:6,:)=0;end
    observer(3)=x(3)+atan2(sin(observer(3)-x(3)),cos(observer(3)-x(3)));
    point=[x;observer;parameters;u;frontend;middle;u];gradient=[dx;dobserver;dp;du;dfrontend;dmiddle;du];
    matrix=gradient(:,1:21);noise=gradient(:,22:end);
end

function [point,generator,mixed]=localResetEnvelope(point,generator,cfg)
    % Enclose BOTH reset outcomes when an affine pre-reset set straddles the
    % stop guard. A nominal reset alone cannot erase the moving branch.
    % This certifies the reset image of this affine set, not the preceding
    % nonlinear flow. Preserve every existing common generator column.
    radius=sum(abs(generator),2);threshold=.2*cfg.model.brakeHoldSpeed;
    allStopped=point(end)+radius(end)<=0 && point(4)+radius(4)<threshold;
    noneStopped=point(end)-radius(end)>0 || point(4)-radius(4)>=threshold;
    mixed=~allStopped && ~noneStopped;
    if allStopped
        point(4:6)=0;generator(4:6,:)=0;
    elseif mixed
        % Standard zonotope hull of Z and its reset image:
        % c=(c1+c2)/2, G=[(G1+G2)/2,(G1-G2)/2,(c1-c2)/2].
        % It encloses both correlated joint states without duplicating the
        % full velocity radius every time the guard is crossed. The center
        % need not be the zero-noise trajectory; its anchor offset is kept.
        jump=zeros(numel(point),1);jump(4:6)=point(4:6)/2;
        removed=zeros(size(generator));removed(4:6,:)=generator(4:6,:)/2;
        removed=removed(:,any(removed,1));
        generator(4:6,:)=generator(4:6,:)/2;
        generator=[generator,removed,jump];
        point(4:6)=point(4:6)/2;
    end
    point(9)=point(3)+atan2(sin(point(9)-point(3)),cos(point(9)-point(3)));
end

function cfg=localPlantConfiguration(cfg,parameters)
    cfg.tire.corneringStiffness=cfg.tire.corneringStiffness.*(1+parameters(1:2));
    cfg.tire.frictionCoefficient=cfg.tire.frictionCoefficient.*(1+parameters(3:4));
end

function [value,gradient]=localPlantFieldTangent(x,u,plant,nominal,dx,du,dp)
    value=nonlinearBicycleModel.derivative(x,u,plant);
    [a,b]=nonlinearBicycleModel.jacobian(x,u,plant);
    % Fractional factors are relative to the root nominal parameters, even
    % when the mean parameter changes. Retain their identity at later times.
    ratio=[nominal.tire.corneringStiffness./plant.tire.corneringStiffness; ...
        nominal.tire.frictionCoefficient./plant.tire.frictionCoefficient];
    c=nonlinearBicycleModel.parameterJacobian(x,u,plant).*ratio.';
    gradient=a*dx+b*du+c*dp;
end

function [point,dpoint]=localObserverTangent(point,dpoint,input,dm,contract)
    design=contract.design;
    pseudo=localPseudoMeasurement(input,design);
    dpseudo=zeros(4,size(dm,2));
    measurement=[input.gnssPosition;input.gnssVelocity;input.bodyAcceleration;input.yawRate;zeros(2,1)];
    if contract.heldInputAvailable,measurement(8:9)=input.heldInput;end
    steps=[0;0;.005;.005;.001;.001;.00015;.0001;.001];
    if ~contract.heldInputAvailable,steps(8:9)=0;end
    if contract.heldInputAvailable,steps(9)=min(steps(9),max(eps,(1-abs(measurement(9)))/4));end
    for j=3:9
        if steps(j)==0 || ~any(dm(j,:)),continue;end
        delta=zeros(9,1);delta(j)=steps(j);
        plus=localPseudoMeasurement(localMeasurementVector(measurement+delta,input),design);
        minus=localPseudoMeasurement(localMeasurementVector(measurement-delta,input),design);
        difference=plus-minus;
        dpseudo=dpseudo+difference/(2*steps(j))*dm(j,:);
    end
    state=[point(4:5);point(1:2);input.gnssPosition;point(3)];
    gradient=[dpoint(4:5,:);dpoint(1:2,:);dm(1:2,:);dpoint(3,:)];
    for j=1:contract.integrationSubstepCount
        h=contract.integrationStep;
        [v1,d1]=localObserverFieldTangent(state,gradient,input,dm,pseudo,dpseudo,design);
        [v2,d2]=localObserverFieldTangent(state+h*v1/2,gradient+h*d1/2,input,dm,pseudo,dpseudo,design);
        [v3,d3]=localObserverFieldTangent(state+h*v2/2,gradient+h*d2/2,input,dm,pseudo,dpseudo,design);
        [v4,d4]=localObserverFieldTangent(state+h*v3,gradient+h*d3,input,dm,pseudo,dpseudo,design);
        state=state+h*(v1+2*v2+2*v3+v4)/6;gradient=gradient+h*(d1+2*d2+2*d3+d4)/6;
        state(7)=atan2(sin(state(7)),cos(state(7)));
    end
    point=state([3,4,7,1,2]);dpoint=gradient([3,4,7,1,2],:);
end

function [value,gradient]=localObserverFieldTangent(state,gradient,input,dm,pseudo,dpseudo,design)
    cross=[0,-1;1,0];kv=design.velocity.gain;kp=design.position.gain;
    value=zeros(7,1);
    value(1:2)=input.bodyAcceleration-input.yawRate*cross*state(1:2)+kv*(pseudo(1:2)-state(1:2));
    value(3:4)=input.gnssVelocity+kp*(state(5:6)-state(3:4));
    value(5:6)=input.gnssVelocity;value(7)=input.yawRate;
    result=zeros(size(gradient));
    result(1:2,:)=dm(5:6,:)-cross*state(1:2)*dm(7,:) ...
        -input.yawRate*cross*gradient(1:2,:)+kv*(dpseudo(1:2,:)-gradient(1:2,:));
    result(3:4,:)=dm(3:4,:)+kp*(gradient(5:6,:)-gradient(3:4,:));
    result(5:6,:)=dm(3:4,:);result(7,:)=dm(7,:);
    vector=pseudo(3:4);yaw=state(7);gain=design.yaw.correctionBandwidth;
    value(7)=value(7)+gain*(vector(2)*cos(yaw)-vector(1)*sin(yaw));
    result(7,:)=result(7,:)+gain*([ -sin(yaw),cos(yaw)]*dpseudo(3:4,:) ...
        -(vector(2)*sin(yaw)+vector(1)*cos(yaw))*gradient(7,:));
    gradient=result;
end

function value=localPseudoMeasurement(input,design)
    if isfield(input,'courseGeometry'),geometry=input.courseGeometry;
    else,geometry=nrmmEgoCourseGeometry('course',input,design);end
    lateral=geometry.lateralMeasurement.center;
    if geometry.lateralMeasurement.source=="kinematic" || ~isfinite(lateral),lateral=NaN;end
    velocity=nrmmObserverVectorField('velocityMeasurement',input.gnssVelocity,input.yawRate,design,lateral);
    value=[velocity;geometry.correspondence.rotationVector];
end

function input=localMeasurementVector(values,input)
    input.gnssPosition=values(1:2);input.gnssVelocity=values(3:4);
    input.bodyAcceleration=values(5:6);input.yawRate=values(7);
    if ~isempty(input.heldInput),input.heldInput=values(8:9);end
    if isfield(input,'courseGeometry'),input=rmfield(input,'courseGeometry');end
end

function difference=localDifference(first,second)
    difference=first-second;
    for j=[3,9,numel(first)-5]
        difference(j)=atan2(sin(difference(j)),cos(difference(j)));
    end
end

function value=localHold(joint,noise,context)
    cfg=context.cfg;contract=context.contract;
    physical=joint(1:6);observer=joint(7:11);parameters=joint(12:15);previous=joint(16:17);frontend=joint(18:21);
    plant=cfg;
    plant.tire.corneringStiffness=cfg.tire.corneringStiffness.*(1+parameters(1:2));
    plant.tire.frictionCoefficient=cfg.tire.frictionCoefficient.*(1+parameters(3:4));
    input=context.input;
    if ~context.first
        deviation=[observer;physical(6)+noise(7,1)]-context.reference;
        deviation(3)=atan2(sin(deviation(3)),cos(deviation(3)));
        input=input+context.gain*deviation;
    end
    if abs(input(2))>1
        error('nrmmJointPrediction:inputDomain','The declared future feedback leaves the Fiala input domain.');
    end
    samples=size(noise,2);middle=zeros(6,1);
    for sample=1:samples
        if context.first && sample==1
            measurement=contract.currentMeasurement;
            frontend=contract.positionVelocity.point;
        else
            % The same physical parameters generate the state and IMU signal.
            % They are not added again as an independent observer disturbance.
            measuredInput=input;if sample==1,measuredInput=previous;end
            measurement=localMeasurement(physical,measuredInput,noise(:,sample),plant,contract);
        end
        if ~(context.first && sample==1)
            frontend=nrmmPositionVelocity('pointMeasure',frontend,measurement.gnssPosition,contract.positionVelocity.gain);
            measurement.gnssVelocity=frontend(3:4);
        end
        frontend=nrmmPositionVelocity('pointPredict',frontend,measurement.bodyAcceleration,observer(3),contract.samplePeriod);
        observer=localObserverStep(observer,measurement,contract);
        physical=localPlantFlow(physical,input,plant,contract.samplePeriod);
        if sample==samples/2,middle=physical;end
    end
    physical=nonlinearBicycleModel.sampleEndState(physical,input,plant);
    observer(3)=physical(3)+atan2(sin(observer(3)-physical(3)),cos(observer(3)-physical(3)));
    value=[physical;observer;parameters;input;frontend;middle;input];
end

function input=localMeasurement(x,u,noise,cfg,contract)
    derivative=nonlinearBicycleModel.derivative(x,u,cfg);
    held=[];if contract.heldInputAvailable,held=u;end
    input=struct('gnssPosition',x(1:2)+noise(1:2), ...
        'velocityErrorMaximum',contract.currentMeasurement.velocityErrorMaximum, ...
        'bodyAcceleration',[derivative(4)-x(6)*x(5);derivative(5)+x(6)*x(4)]+noise(5:6), ...
        'yawRate',x(6)+noise(7),'heldInput',held);
end

function x=localPlantFlow(x,u,cfg,h)
    % Resolve the sensor clock. Apply the declared rest reset only at the
    % controller boundary; sensor sampling itself must not reset the plant.
    f=@(z)nonlinearBicycleModel.derivative(z,u,cfg);
    first=f(x);second=f(x+h*first/2);third=f(x+h*second/2);fourth=f(x+h*third);
    x=x+h*(first+2*second+2*third+fourth)/6;
end
