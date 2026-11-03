function value = nrmmJointPrediction(action,varargin)
%nrmmJointPrediction Lift the sampled plant, observer and future feedback.
% Ego lift: [physical ego (6); numerical ego observer (5); tire factors (4);
% previous held input (2); position/IMU filter prior (4)]. The coupled lift
% appends physical target (6), target observer (6), and radar predictor (2).
% Its sensor block includes radar noise as well. At a control boundary the IMU measures the
% previous input; the new command drives the subsequent plant integration.
% Five real noise channels are position (2), acceleration (2), and gyro (1).
% Slots 3:4 of the legacy seven-slot noise array are zero placeholders.
% A parameter column is introduced once and retained across the whole horizon.
% Future sensor errors enter once per measurement, shared by the observer and
% feedback. The actual current measurement and first command are conditioned
% data. Default supports are exact only for the constructed joint affine maps.
% options.nonlinearEnclosure instead uses full-domain nonlinear outer images
% of the declared sampled map (real arithmetic, not a complete safety proof).
% No current observer budget is reset in the horizon.
    switch string(action)
        case "contract"
            value=localContract(varargin{:});
        case "conditionCoupled"
            value=localConditionCoupled(varargin{:});
        case "coupledRoot"
            value=localCoupledRoot(varargin{:});
        case "coupledPropagate"
            value=localCoupledPropagate(varargin{:});
        case "coupledLinearize"
            [point,matrix,noise,branch]=localCoupledLinearize(varargin{:});
            value=struct('point',point,'matrix',matrix,'noise',noise,'branch',branch);
        case "observerStep"
            value=localObserverStep(varargin{:});
        case "observerRk4"
            % Numeric core shared by MATLAB and the optional generated MEX.
            [point,gradient]=localObserverRk4(varargin{:});
            value=struct('point',point,'gradient',gradient);
        case "posteriorOuter"
            value=localPosteriorOuter(varargin{:});
        case "plantConfiguration"
            value=localPlantConfiguration(varargin{:});
        case "plantField"
            [point,gradient]=localPlantFieldCore(varargin{:});
            value=struct('point',point,'gradient',gradient);
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
        case "rootSet"
            value=localRootSet(varargin{:});
        case "feedback"
            value=localFeedback(varargin{:});
        case "policyTube"
            value=localPolicyTube(varargin{:});
        case "conditionRetained"
            value=localConditionRetained(varargin{:});
        case "conditionMeasurements"
            value=localConditionMeasurements(varargin{:});
        otherwise
            error('nrmmJointPrediction:invalidAction','Unknown joint prediction action.');
    end
end

function set=localRootSet(state,generator,contract)
    % Absolute joint coordinates, before the observer's current RK4 hold.
    previous=contract.currentMeasurement.heldInput;
    assert(numel(previous)==2,'nrmmJointPrediction:missingHeldInput','A retained policy requires the actual previous input.');
    n=size(generator,2);
    g=[generator,zeros(6,4);zeros(5,n+4);zeros(4,n),diag(contract.parameterRadius);zeros(6,n+4)];
    set=nrmmSet([state;contract.point;zeros(4,1);previous;contract.positionVelocity.point],g);
end

function tube=localPolicyTube(root,policies)
    % Each stage encloses ALL feedforward choices in its convex input box.
    % No point-dynamics equality or derivative certificate is used here.
    % A failed new image must not replace a previously retained certificate.
    count=numel(policies);sets=cell(1,count+1);sets{1}=root;
    middles=cell(1,count);inputs=cell(1,count);changes=cell(1,count);
    for k=1:count
        context=policies(k).context;
        domain=struct('lower',[zeros(6,1);policies(k).lower], ...
            'upper',[zeros(6,1);policies(k).upper]);
        initial=promote(sets{k});
        image=nrmmJointEnclosure('hold',initial.center,initial.generator,context,domain);
        keep=any(image.generator,1);
        raw=nrmmSet(image.point,image.generator);
        % The incoming actuator and the held output share primitive columns
        % here. Compression subsequently reorders/merges those columns, so
        % their difference must be formed BEFORE that change of coordinates.
        change=raw(16:17)-initial(16:17);[lo,hi]=bounds(change);
        changes{k}=nrmmSet.range(lo,hi);
        set=nrmmSet(image.point,image.generator(:,keep));
        % Preserve common physical/estimated motion when folding old small
        % residuals. Boxing absolute observer coordinates independently
        % would erase the very cancellation that makes feedback useful.
        transform=eye(numel(set.center));
        transform(7:11,1:5)=-eye(5);
        transform(18:19,1:2)=-eye(2);
        yaw=set.center(3);rotation=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
        transform(20:21,4:5)=-rotation;
        transform(22:27,1:6)=-eye(6);transform(28:29,16:17)=-eye(2);
        set=compress(set,1e-4,transform,12:15);
        sets{k+1}=set(1:21);middles{k}=set(22:27);inputs{k}=set(16:17);
    end
    tube=struct('sets',{sets},'middles',{middles},'inputs',{inputs},'changes',{changes}, ...
        'realArithmeticEnclosure',true,'floatingPointCertified',false);
end

function posterior=localConditionRetained(set,state,generator,contract,oldContract,cfg)
    % Keep the old generators and add equations. Reboxing this intersection
    % could destroy containment in the predicted tube. The next propagation
    % may safely use the larger retained set without discarding this record.
    assert(all(contract.noiseRadius<=oldContract.noiseRadius), ...
        'nrmmJointPrediction:changedNoiseContract','New noise bounds exceed the retained contract.');
    assert(isequal(contract.design,oldContract.design) && contract.samplePeriod==oldContract.samplePeriod ...
        && contract.integrationStep==oldContract.integrationStep ...
        && contract.integrationSubstepCount==oldContract.integrationSubstepCount, ...
        'nrmmJointPrediction:changedObserverContract','The retained observer map has changed.');
    assert(isequal(contract.positionVelocity.gain,oldContract.positionVelocity.gain), ...
        'nrmmJointPrediction:changedObserverContract','The position frontend gain has changed.');
    set=promote(set);n=size(set.generator,2);m=size(generator,2);
    selector=zeros(11,21);selector(1:5,7:11)=eye(5);selector(6:7,16:17)=eye(2);
    % The retained frontend is a PRIOR at future control boundaries. The
    % current runtime publishes its position-measurement POSTERIOR.
    gain=contract.positionVelocity.gain;update=[gain(1)*eye(2);gain(2)*eye(2)];
    selector(8:11,18:21)=eye(4)-update*[eye(2),zeros(2)];
    known=[contract.point;contract.currentMeasurement.heldInput; ...
        contract.positionVelocity.point-update*contract.currentMeasurement.gnssPosition];
    known(3)=set.center(9)+atan2(sin(known(3)-set.center(9)),cos(known(3)-set.center(9)));
    state(3)=set.center(3)+atan2(sin(state(3)-set.center(3)),cos(state(3)-set.center(3)));
    equal=[selector*set.generator,zeros(11,m);set.generator(1:6,:),-generator];
    rhs=[known-selector*set.center;state-set.center(1:6)];
    % Persistent physical parameters are conditioned, never independently reset.
    a=[set.generator(12:15,:),zeros(4,m);-set.generator(12:15,:),zeros(4,m)];
    b=[contract.parameterRadius-set.center(12:15);contract.parameterRadius+set.center(12:15)];
    options=optimoptions('linprog','Display','none');
    [point,~,flag]=linprog(zeros(n+m,1),a,b,equal,rhs,-ones(n+m,1),ones(n+m,1),options);
    assert(flag>0 && ~isempty(point),'nrmmJointPrediction:inconsistentPosterior', ...
        'Current observer/filter/input data and error set do not intersect the retained joint prediction.');
    posterior=struct('base',set,'equal',equal,'rhs',rhs,'a',a,'b',b, ...
        'primitiveLower',-ones(n+m,1),'primitiveUpper',ones(n+m,1), ...
        'physicalGeneratorColumns',n,'currentGenerator',generator,'subsetByConstruction',true, ...
        'numericalFeasibilityChecked',true,'floatingPointCertified',false);
    % A smaller convenient OUTER image is only a proposal for a new full
    % certificate. It is not asserted to be a subset of the old zonotope.
    outer=localRootSet(state,generator,contract);outer.center(9)=known(3);
    priorMap=selector(8:11,18:21);
    if rcond(priorMap)>1e-10
        outer.center(18:21)=priorMap\known(8:11);
    else
        [lo,hi]=bounds(set(18:21));outer(18:21)=nrmmSet.range(lo,hi,size(outer.generator,2));
    end
    % Both old prediction and current measurement enclose the true state.
    % Their coordinate intersection is therefore another sound outer set.
    % It is still not a subset certificate for the old signed zonotope.
    [lo,hi]=bounds(set);outer=nrmmSet.intersectRange(outer,lo,hi);
    posterior.proposedOuter=outer;
    if nargin>=6
        % Project the actual constrained intersection, rather than dropping
        % its equalities and restoring a stale physical projection.
        outer=localPosteriorOuter(posterior);
        conditioned=localConditionMeasurements(outer,contract,cfg);
        posterior.proposedOuter=conditioned.set;
        posterior.measurementConditioning=conditioned.diagnostics;
        posterior.currentMeasurement=contract.currentMeasurement;
        posterior.measurementNoiseRadius=contract.noiseRadius;
    end
end

function set=localPosteriorOuter(posterior)
    % A coordinate outer projection is a proposal for FRESH admission. It
    % need not be a subset of the old signed zonotope. The saved constrained
    % posterior remains the exact subset witness for the old certificate.
    assert(all(posterior.primitiveLower==-1) && all(posterior.primitiveUpper==1));
    extra=size(posterior.currentGenerator,2);count=numel(posterior.base.center);
    lo=zeros(count,1);hi=lo;options=optimoptions('linprog','Display','none');
    directions=[posterior.base.generator,zeros(count,extra)]';
    % E*xi=rhs fixes every direction in row(E). Compute all such projections
    % together, retaining the residual on |xi|<=1 instead of snapping it to
    % zero. Thus a nearly dependent direction still has a valid outer bound.
    multipliers=lsqminnorm(posterior.equal',directions);
    residual=sum(abs(directions-posterior.equal'*multipliers),1)';
    fixed=residual<=64*eps*max(1,sum(abs(directions),1)');
    projected=multipliers'*posterior.rhs;
    for j=1:count
        if fixed(j)
            lo(j)=posterior.base.center(j)+projected(j)-residual(j);
            hi(j)=posterior.base.center(j)+projected(j)+residual(j);
        else
            direction=directions(:,j);
            lo(j)=posterior.base.center(j)+localPosteriorSupport(direction,posterior,options);
            hi(j)=posterior.base.center(j)-localPosteriorSupport(-direction,posterior,options);
        end
    end
    set=nrmmSet.range(lo,hi);
end

function lower=localPosteriorSupport(direction,p,options)
    [~,~,flag,~,dual]=linprog(direction,p.a,p.b,p.equal,p.rhs, ...
        p.primitiveLower,p.primitiveUpper,options);
    assert(flag>0,'nrmmJointPrediction:posteriorSupportFailed', ...
        'A constrained posterior support solve did not finish.');
    inequality=max(0,dual.ineqlin);equality=dual.eqlin;
    % Any nonnegative inequality dual is valid after accounting for its
    % stationarity residual on |xi|<=1. A primal optimum alone is not a bound.
    lower=-p.b'*inequality-p.rhs'*equality ...
        -sum(abs(direction+p.a'*inequality+p.equal'*equality));
end

function result=localConditionMeasurements(set,contract,cfg)
    % Full-mode conditioning excludes states inconsistent with every allowed
    % tire parameter and sensor error. The engineering branch below uses an
    % affine IMU approximation and explicitly reports its narrower scope.
    [lo,hi]=bounds(set);before=radius(set(1:6));measurement=contract.currentMeasurement;
    lower=[measurement.gnssPosition;measurement.yawRate]-contract.noiseRadius([1,2,7]);
    upper=[measurement.gnssPosition;measurement.yawRate]+contract.noiseRadius([1,2,7]);
    indices=[1,2,6];lo(indices)=max(lo(indices),lower);hi(indices)=min(hi(indices),upper);
    assert(all(lo<=hi),'nrmmJointPrediction:inconsistentMeasurement', ...
        'Current position/gyro data contradict the retained physical family.');
    % The numerical observer and previous actuator command are exactly known.
    point=contract.point;point(3)=set.center(9)+atan2(sin(point(3)-set.center(9)),cos(point(3)-set.center(9)));
    lo(7:11)=point;hi(7:11)=point;
    lo(16:17)=measurement.heldInput;hi(16:17)=measurement.heldInput;
    if cfg.controller.reducedJointInformation && string(cfg.solver.method)=="rti" ...
            && ~cfg.controller.retainedTube && ~cfg.controller.jointNonlinearPrediction
        result=localAffineMeasurementConditioning(nrmmSet.range(lo,hi),contract,cfg,before);
        return
    end
    evaluations=0;unresolved=0;
    % First constrain lateral motion through the force balance, then speed
    % through the complete two-component IMU image. Neither scan assumes a
    % monotone tire inverse; saturated or unresolved cells are retained.
    for coordinate=[5,4]
        tolerance=5e-4;
        if coordinate==4,tolerance=max(tolerance,.02*(hi(coordinate)-lo(coordinate)));end
        endpoints=zeros(2,1);
        for edge=1:2
            [endpoints(edge),used,unknown,found]=localMeasurementEdge( ...
                lo,hi,coordinate,edge,tolerance,4096-evaluations,measurement,contract,cfg);
            evaluations=evaluations+used;unresolved=unresolved+unknown;
            assert(found,'nrmmJointPrediction:inconsistentMeasurement', ...
                'No state in the retained family can generate the current IMU data.');
        end
        lo(coordinate)=endpoints(1);hi(coordinate)=endpoints(2);
    end
    set=nrmmSet.range(lo,hi);
    diagnostics=struct('method',"nonlinearCurrentMeasurementIntersection", ...
        'priorPhysicalRadius',before,'posteriorPhysicalRadius',radius(set(1:6)), ...
        'imageEvaluations',evaluations,'unresolvedCells',unresolved, ...
        'noiseRadius',contract.noiseRadius,'parameterRadius',contract.parameterRadius, ...
        'realArithmeticEnclosure',true,'floatingPointCertified',false);
    result=struct('set',set,'diagnostics',diagnostics);
end

function result=localAffineMeasurementConditioning(set,contract,cfg,before)
    % Engineering measurement update: enclose the intersection of the current
    % publication and the AFFINE two-axis IMU strip in one matrix operation.
    % No statistical noise interpretation and no nonlinear remainder claim.
    set=promote(set);
    x=set.center(1:6);u=contract.currentMeasurement.heldInput;
    parameters=set.center(12:15);plant=localPlantConfiguration(cfg,parameters);
    ratio=1./(1+parameters);g=set.generator;
    [field,df]=localPlantFieldCore(x,u,plant,ratio,g(1:6,:),zeros(2,size(g,2)),g(12:15,:));
    value=[field(4)-x(6)*x(5);field(5)+x(6)*x(4)];
    h=[df(4,:)-x(5)*g(6,:)-x(6)*g(5,:); ...
        df(5,:)+x(4)*g(6,:)+x(6)*g(4,:)];
    noise=diag(contract.noiseRadius(5:6));
    gain=(g*h')*pinv(h*h'+noise*noise');
    innovation=contract.currentMeasurement.bodyAcceleration-value;
    proposal=nrmmSet(set.center+gain*innovation,[g-gain*h,gain*noise]);
    [lo,hi]=bounds(set);[pl,ph]=bounds(proposal);
    accepted=all(max(lo,pl)<=min(hi,ph));
    if accepted,set=nrmmSet.intersectRange(proposal,lo,hi);end
    diagnostics=struct('method',"affineCurrentMeasurementIntersection", ...
        'priorPhysicalRadius',before,'posteriorPhysicalRadius',radius(set(1:6)), ...
        'imageEvaluations',1,'unresolvedCells',double(~accepted), ...
        'noiseRadius',contract.noiseRadius,'parameterRadius',contract.parameterRadius, ...
        'realArithmeticEnclosure',false,'floatingPointCertified',false, ...
        'contractionAccepted',accepted,'scope',"affine IMU strip only; nonlinear remainder omitted");
    result=struct('set',set,'diagnostics',diagnostics);
end

function [endpoint,evaluations,unresolved,found]=localMeasurementEdge(lo,hi,coordinate,edge,tolerance,budget,measurement,contract,cfg)
    % Only the hull of surviving cells is used by the caller. Search its
    % two extremal cells, keeping the entire unsearched interior. Every
    % excluded exterior interval still has a disjoint nonlinear IMU image;
    % no monotonicity or heuristic inversion is assumed.
    queue=[lo(coordinate),hi(coordinate)];evaluations=0;unresolved=0;found=false;endpoint=NaN;
    while ~isempty(queue)
        interval=queue(end,:);queue(end,:)=[];
        if evaluations>=budget
            unresolved=size(queue,1)+1;found=true;endpoint=interval(edge);return;
        end
        state=nrmmSet.range(lo(1:6),hi(1:6));
        state(coordinate)=nrmmSet.range(interval(1),interval(2));
        evaluations=evaluations+1;
        try
            acceleration=nrmmJointEnclosure('acceleration',state,nrmmSet(measurement.heldInput), ...
                nrmmSet.range(lo(12:15),hi(12:15)),cfg);
            [al,ah]=bounds(acceleration);
            disjoint=any(ah<measurement.bodyAcceleration-contract.noiseRadius(5:6) ...
                | al>measurement.bodyAcceleration+contract.noiseRadius(5:6));
            contained=all(al>=measurement.bodyAcceleration-contract.noiseRadius(5:6) ...
                & ah<=measurement.bodyAcceleration+contract.noiseRadius(5:6));
        catch exception
            if ~startsWith(string(exception.identifier),["nrmmSet:","nrmmJointEnclosure:"]),rethrow(exception);end
            unresolved=1;found=true;endpoint=interval(edge);return;
        end
        if disjoint,continue;end
        if contained || diff(interval)<=tolerance
            found=true;endpoint=interval(edge);return;
        end
        middle=mean(interval);children=[interval(1),middle;middle,interval(2)];
        if edge==1,children=flipud(children);end
        queue=[queue;children]; %#ok<AGROW>
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
        'measurementHistory',runtime.measurementHistory, ...
        'positionVelocity',input.positionVelocity,'sensorContract',"position-only-GNSS", ...
        'heldInputAvailable',numel(input.heldInput)==2, ...
        'noiseRadius',[repmat(sensors.positionNoiseMaximum,2,1); ...
            zeros(2,1); ...
            repmat(sensors.accelerometerNoiseMaximum,2,1);sensors.gyroscopeNoiseMaximum], ...
        'parameterRadius',repmat(radius,4,1), ...
        'parameterOrder',["frontCornering";"rearCornering";"frontFriction";"rearFriction"], ...
         'targetObserver',struct('state',runtime.targetState, ...
            'predictor',runtime.targetOutputPredictor,'identifier',runtime.targetIdentifier, ...
            'noiseRadius',design.sensors.radarNoiseMaximum), ...
        'physicalModel',"declaredFialaWithPersistentTireParameters", ...
        'nonlinearCertificate',false);
end

function point=localObserverStep(point,input,contract)
    % Exactly the ego projection of the runtime's packed RK4 update. Target
    % states do not drive this projection. Retain the resetting GNSS predictor.
    design=contract.design;
    if ~isfield(input,'courseGeometry')
        input.courseGeometry=nrmmEgoCourseGeometry('pointCourse',input,design);
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
    nonlinear=isfield(options,'nonlinearEnclosure') && options.nonlinearEnclosure;
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
    tube=struct('kind',"joint-sampled-affine-tube-v3", ...
        'generator',{cell(1,records)},'observerErrorRadius',zeros(5,records), ...
        'physicalRadius',zeros(6,records),'mean',zeros(21,count+1), ...
        'inputRadius',zeros(2,count),'inputGenerator',{cell(1,count)}, ...
        'feedbackGain',gains,'transition',{cell(1,count)},'noiseMap',{cell(1,count)}, ...
        'currentInputUncertaintyRadius',zeros(2,1),'currentMeasurementConditioned',true, ...
        'futureObserverDynamicsIncluded',true,'futurePrimitiveBudgetsAssumed',false, ...
        'persistentParameterColumns',rootColumns+(1:4),'nonlinearCertificate',false, ...
        'scope',"joint affine maps along the RTI shooting reference; nonlinear remainders not certified", ...
        'mode',"causalObserverFeedback",'time',[0,cumsum(durations)], ...
        'predictionSensorSamples',round(durations/contract.samplePeriod), ...
        'mixedStopBranches',false(1,count));
    tube.generator{1}=initialGenerator;
    tube.physicalRadius(:,1)=sum(abs(initialGenerator),2);
    tube.observerErrorRadius(:,1)=abs(estimation*center)+sum(abs(estimation*generators),2);
    tube.mean(:,1)=center;
    tube.realArithmeticEnclosure=nonlinear;tube.floatingPointCertified=false;
    if nonlinear
        tube.kind="joint-nonlinear-outer-image-v1";
        tube.scope="declared sampled ego/observer map on the supplied decision domain; not a complete safety certificate";
    end
    for k=1:count
        if k>1
            estimateGenerators=generators([7:11,6],:);
            reserve=sum(abs(gains(2,:,k)*estimateGenerators))+abs(gains(2,6,k))*contract.noiseRadius(7);
            authority=min(cfg.actuation.brakingRatioMaximum-anchor.inputs(2,k), ...
                anchor.inputs(2,k)-cfg.actuation.brakingRatioMinimum);
            if nonlinear && isfield(options,'decisionLower')
                authority=min(cfg.actuation.brakingRatioMaximum-anchor.inputs(2,k)-options.decisionUpper(8,k), ...
                    anchor.inputs(2,k)+options.decisionLower(8,k)-cfg.actuation.brakingRatioMinimum);
            end
            % Keep computed ancillary authority just inside its boundary;
            % never trim the physical uncertainty set to repair roundoff.
            if nonlinear,authority=max(0,authority)*(1-32*eps);end
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
        if nonlinear
            domain=zeros(8,1);
            if isfield(options,'decisionLower')
                domain=struct('lower',options.decisionLower(:,k),'upper',options.decisionUpper(:,k));
                stage=options.stages(k);
                context.a=stage.a;context.b=stage.b;context.am=stage.am;context.bm=stage.bm;
                context.defect=stage.next-anchor.states(:,k+1);
            end
            try
                enclosure=nrmmJointEnclosure('hold',center,generators,context,domain);
            catch cause
                failure=MException('nrmmJointPrediction:uncertifiedImage', ...
                    'Nonlinear joint enclosure failed at hold %d (%.6g s): %s', ...
                    k,sum(durations(1:k-1)),cause.message);
                throw(addCause(failure,cause));
            end
            point=enclosure.point;nextGenerators=enclosure.generator;
            point(28:29)=point(28:29)+anchor.inputs(:,k);
            matrix=[];noise=[];mixed=false;
        else
            context.contract=localPredictionContract(contract,cfg,durations(k),k==1);
            tube.predictionSensorSamples(k)=round(durations(k)/context.contract.samplePeriod);
            [~,matrix,noise,branch]=localLinearize(center,context);
            beforeReset=[branch.matrix*generators,branch.noise];
            [point,nextGenerators,mixed]=localResetEnvelope(branch.point,beforeReset,cfg);
            if mixed
                matrix=branch.matrix;noise=branch.noise;
                matrix(4:6,:)=matrix(4:6,:)/2;noise(4:6,:)=noise(4:6,:)/2;
            end
        end
        tube.mixedStopBranches(k)=mixed;
        middle=point(22:27);next=point(1:21);input=point(28:29);
        if nonlinear
            middleReference=localReferenceMiddle(anchor,k,cfg,durations(k));
            tube.generator{2*k}=[nextGenerators(22:27,:),middle-middleReference];
            tube.generator{2*k+1}=[nextGenerators(1:6,:),next(1:6)-anchor.states(:,k+1)];
        else
            % RTI already carries F(xbar,ubar)-xbar_next in its dynamics
            % equality. It is a known shooting defect, not an independent
            % uncertain disturbance. Linearize every future joint hold at
            % the SAME shooting reference instead of accumulating a separate
            % open-loop nominal rollout and enclosing its drift as error.
            tube.generator{2*k}=nextGenerators(22:27,:);
            tube.generator{2*k+1}=nextGenerators(1:6,:);
            next=localReanchorJointMean(next,anchor.states(:,k+1));
        end
        tube.physicalRadius(:,2*k:2*k+1)=[sum(abs(tube.generator{2*k}),2),sum(abs(tube.generator{2*k+1}),2)];
        tube.observerErrorRadius(:,2*k)=NaN; % No fictitious midpoint posterior.
        tube.observerErrorRadius(:,2*k+1)=abs(estimation*next)+sum(abs(estimation*nextGenerators(1:21,:)),2);
        tube.inputGenerator{k}=nextGenerators(28:29,:);
        tube.inputRadius(:,k)=abs(input-anchor.inputs(:,k))+sum(abs(tube.inputGenerator{k}),2);
        if ~nonlinear,tube.transition{k}=matrix(1:21,:);tube.noiseMap{k}=noise(1:21,:);end
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

function joint=localReanchorJointMean(joint,reference)
    % Preserve nominal observer bias and frontend error in the new local
    % chart. Only the expansion point moves; shared uncertainty columns stay.
    old=joint(1:6);change=reference-old;
    change(3)=atan2(sin(change(3)),cos(change(3)));
    oldRotation=[cos(old(3)),-sin(old(3));sin(old(3)),cos(old(3))];
    newRotation=[cos(reference(3)),-sin(reference(3));sin(reference(3)),cos(reference(3))];
    joint(1:6)=reference;
    joint(7:11)=joint(7:11)+change(1:5);
    joint(18:19)=joint(18:19)+change(1:2);
    joint(20:21)=joint(20:21)+newRotation*reference(4:5)-oldRotation*old(4:5);
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
            [field,df]=localPlantFieldTangent(x,measuredInput,plant,cfg,dx,dmeasuredInput,dp);
            measurement=localMeasurement(x,measuredInput,zeros(7,1),plant,contract,field);
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
        if sample==1
            [f1,d1]=localPlantFieldTangent(x,u,plant,cfg,dx,du,dp);
        else
            f1=field;d1=df;
        end
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

function plant=localPlantConfiguration(cfg,parameters)
    % Only numerical model parameters cross the generated-kernel boundary.
    plant=struct('model',struct('scheduleSpeedFloor',cfg.model.scheduleSpeedFloor, ...
        'kinematicSpeedRange',cfg.model.kinematicSpeedRange, ...
        'kinematicLagSeconds',cfg.model.kinematicLagSeconds,'brakeHoldSpeed',cfg.model.brakeHoldSpeed), ...
        'vehicle',struct('m',cfg.vehicle.m,'Iz',cfg.vehicle.Iz,'lf',cfg.vehicle.lf, ...
        'lr',cfg.vehicle.lr,'gravity',cfg.vehicle.gravity), ...
        'tire',struct('corneringStiffness',cfg.tire.corneringStiffness.*(1+parameters(1:2)), ...
        'frictionCoefficient',cfg.tire.frictionCoefficient.*(1+parameters(3:4))), ...
        'roadLoad',cfg.roadLoad);
end

function [value,gradient]=localPlantFieldTangent(x,u,plant,nominal,dx,du,dp)
    % Fractional factors are relative to the root nominal parameters, even
    % when the mean parameter changes. Retain their identity at later times.
    ratio=[nominal.tire.corneringStiffness./plant.tire.corneringStiffness; ...
        nominal.tire.frictionCoefficient./plant.tire.frictionCoefficient];
    if exist('nrmmJointPlantKernelMex','file')==3
        result=nrmmJointPlantKernelMex(x,u,plant,ratio,dx,du,dp);
        value=result.point;gradient=result.gradient;
    else
        [value,gradient]=localPlantFieldCore(x,u,plant,ratio,dx,du,dp);
    end
end

function [value,gradient]=localPlantFieldCore(x,u,plant,ratio,dx,du,dp)
    value=nonlinearBicycleModel.derivative(x,u,plant);
    [a,b]=nonlinearBicycleModel.jacobian(x,u,plant);
    c=nonlinearBicycleModel.parameterJacobian(x,u,plant).*ratio.';
    gradient=a*dx+b*du+c*dp;
end

function [point,dpoint]=localObserverTangent(point,dpoint,input,dm,contract)
    design=contract.design;
    if isfield(contract,'predictionInverseTolerance')
        input.predictionInverseTolerance=contract.predictionInverseTolerance;
    end
    if any(dm(3:9,:),'all')
        [pseudo,jacobian]=nrmmEgoCourseGeometry('pointMeasurement',input,design);
        dpseudo=jacobian*dm(1:9,:);
    else
        pseudo=localPseudoMeasurement(input,design);dpseudo=zeros(4,size(dm,2));
    end
    state=[point(4:5);point(1:2);input.gnssPosition;point(3)];
    if numel(point)>5
        predictor=point(12:13);dpredictor=dpoint(12:13,:);
        if input.radarDetectionAvailable
            predictor=input.radarRelativePosition;dpredictor=dm(10:11,:);
        end
        state=[state;point(6:11);predictor];
        extra=[dpoint(6:11,:);dpredictor];
    end
    gradient=[dpoint(4:5,:);dpoint(1:2,:);dm(1:2,:);dpoint(3,:)];
    if numel(point)>5,gradient=[gradient;extra];end
    % A narrow numeric interface avoids marshalling the complete observer
    % design at every sensor hold. Parameters remain runtime inputs.
    domain=design.target.domain;
    minimum=domain.speedMinimum;
    if minimum==0,minimum=domain.regularizationSpeedMinimum;end
    kernelDesign=struct('velocity',struct('gain',design.velocity.gain), ...
        'position',struct('gain',design.position.gain), ...
        'yaw',struct('correctionBandwidth',design.yaw.correctionBandwidth), ...
        'target',struct('innovationGains',design.target.innovationGains(:), ...
        'domain',struct('speedMinimum',minimum,'speedMaximum',domain.speedMaximum, ...
        'scalarAccelerationMaximum',domain.scalarAccelerationMaximum, ...
        'yawRateMaximum',domain.yawRateMaximum,'accelerationNormBound',domain.accelerationNormBound, ...
        'regularizationSpeedMinimum',minimum)));
    measurement=struct('bodyAcceleration',input.bodyAcceleration,'yawRate',input.yawRate, ...
        'gnssVelocity',input.gnssVelocity,'radarDetectionAvailable',false);
    if numel(point)>5,measurement.radarDetectionAvailable=input.radarDetectionAvailable;end
    if exist('nrmmJointObserverKernelMex','file')==3
        result=nrmmJointObserverKernelMex(state,gradient,measurement,dm,pseudo,dpseudo, ...
            kernelDesign,contract.integrationStep,contract.integrationSubstepCount);
        state=result.point;gradient=result.gradient;
    else
        [state,gradient]=localObserverRk4(state,gradient,measurement,dm,pseudo,dpseudo, ...
            kernelDesign,contract.integrationStep,contract.integrationSubstepCount);
    end
    indices=[3,4,7,1,2];
    if numel(state)>7,indices=[indices,8:15];end
    point=state(indices);dpoint=gradient(indices,:);
end

function [state,gradient]=localObserverRk4(state,gradient,input,dm,pseudo,dpseudo,design,h,count)
    for j=1:count
        [v1,d1]=localObserverFieldTangent(state,gradient,input,dm,pseudo,dpseudo,design);
        [v2,d2]=localObserverFieldTangent(state+h*v1/2,gradient+h*d1/2,input,dm,pseudo,dpseudo,design);
        [v3,d3]=localObserverFieldTangent(state+h*v2/2,gradient+h*d2/2,input,dm,pseudo,dpseudo,design);
        [v4,d4]=localObserverFieldTangent(state+h*v3,gradient+h*d3,input,dm,pseudo,dpseudo,design);
        state=state+h*(v1+2*v2+2*v3+v4)/6;gradient=gradient+h*(d1+2*d2+2*d3+d4)/6;
        state(7)=atan2(sin(state(7)),cos(state(7)));
    end
end

function [value,gradient]=localObserverFieldTangent(state,gradient,input,dm,pseudo,dpseudo,design)
    cross=[0,-1;1,0];kv=design.velocity.gain;kp=design.position.gain;
    value=zeros(size(state));
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
    if numel(state)>7
        target=state(8:13);predictor=state(14:15);
        dt=gradient(8:13,:);dy=gradient(14:15,:);
        [jac,phi]=nrmmTargetJacobian('evaluate',target(3:4),target(5:6),design.target.domain);
        targetField=[target(3:4)-state(1:2)-input.yawRate*cross*target(1:2); ...
            target(5:6)-input.yawRate*cross*target(3:4);phi-input.yawRate*cross*target(5:6)];
        a=[-input.yawRate*cross,eye(2),zeros(2); ...
            zeros(2),-input.yawRate*cross,eye(2); ...
            zeros(2),jac(:,1:2),jac(:,3:4)-input.yawRate*cross];
        dtField=a*dt-[gradient(1:2,:);zeros(4,size(dm,2))] ...
            -[cross*target(1:2);cross*target(3:4);cross*target(5:6)]*dm(7,:);
        if input.radarDetectionAvailable
            gains=kron(design.target.innovationGains(:),eye(2));
            targetField=targetField+gains*(predictor-target(1:2));
            dtField=dtField+gains*(dy-dt(1:2,:));
        end
        value(8:13)=targetField;result(8:13,:)=dtField;
        value(14:15)=target(3:4)-state(1:2)-input.yawRate*cross*predictor;
        result(14:15,:)=dt(3:4,:)-gradient(1:2,:)-input.yawRate*cross*dy-cross*predictor*dm(7,:);
    end
    gradient=result;
end

function value=localPseudoMeasurement(input,design)
    if isfield(input,'courseGeometry'),geometry=input.courseGeometry;
    else,geometry=nrmmEgoCourseGeometry('pointCourse',input,design);end
    lateral=geometry.lateralMeasurement.center;
    if geometry.lateralMeasurement.source=="kinematic" || ~isfinite(lateral),lateral=NaN;end
    velocity=nrmmObserverVectorField('velocityMeasurement',input.gnssVelocity,input.yawRate,design,lateral);
    value=[velocity;geometry.correspondence.rotationVector];
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

function input=localMeasurement(x,u,noise,cfg,contract,derivative)
    if nargin<6,derivative=nonlinearBicycleModel.derivative(x,u,cfg);end
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

function root=localCoupledRoot(state,generator,contract,target,errorSet)
    % [ego/observer/parameters/input/frontend (21); physical target (6);
    % target observer rho,q,s (6); radar predictor (2)].
    assert(isfield(contract,'targetObserver'),'nrmmJointPrediction:missingTargetObserver', ...
        'Coupled prediction needs the actual target observer state and predictor.');
    n=size(generator,2);g=zeros(35,n+10);
    g(1:6,1:n)=generator;g(12:15,n+(1:4))=diag(contract.parameterRadius);
    r=[cos(state(3)),-sin(state(3));sin(state(3)),cos(state(3))];
    relative=r'*(target(1:2)-state(1:2));
    g(22:23,1:n)=generator(1:2,:)+r*[-relative(2);relative(1)]*generator(3,:);
    g(24,1:n)=generator(3,:);
    g(22:23,n+(5:6))=errorSet.positionRadius*r;
    g(24,n+7)=errorSet.courseRadius;
    g(25,n+8)=errorSet.speedRadius;g(26,n+9)=errorSet.accelerationRadius;
    g(27,n+10)=errorSet.sideslipRadius;g(24,n+10)=-errorSet.sideslipRadius;
    familyUsed=false;
    if isfield(errorSet,'family') && errorSet.family.available
        % Preserve the retained constant-parameter cover instead of expanding
        % it back into independent speed, acceleration and course balls.
        family=errorSet.family;age=contract.time-family.epochTime;
        padding=0;if isfield(errorSet,'familyYawRadius'),padding=errorSet.familyYawRadius;end
        image=nrmmConstantMotionFamily('image',family,contract.time,contract.time,padding);
        low=[max(0,family.lower(1,:)+age*family.lower(2,:));family.lower(2,:); ...
            asin(target(7)*family.lower(3,:));image.courseLower];
        high=[max(0,family.upper(1,:)+age*family.upper(2,:));family.upper(2,:); ...
            asin(target(7)*family.upper(3,:));image.courseUpper];
        if isfield(errorSet,'speedInterval')
            low(1,:)=max(low(1,:),errorSet.speedInterval(1));
            high(1,:)=min(high(1,:),errorSet.speedInterval(2));
        end
        keep=all(low<=high,1);low=low(:,keep);high=high(:,keep);
        assert(~isempty(low),'nrmmJointPrediction:emptyTargetFamily','No current target parameter cell remains.');
        midpoint=(low+high)/2;width=(high-low)/2;
        nominal=[target(4:6);target(3)+target(6)];
        scale=max(max(high,[],2)-min(low,[],2),1e-10);
        normalized=(midpoint-nominal)./scale;
        covariance=normalized*normalized'/size(normalized,2)+diag(mean((width./scale).^2,2)/3);
        [basis,~]=eig((covariance+covariance')/2);
        lower=min(basis'*normalized-abs(basis')*(width./scale),[],2);
        upper=max(basis'*normalized+abs(basis')*(width./scale),[],2);
        map=zeros(6,4);map(4,1)=1;map(5,2)=1;map(6,3)=1;map(3,3:4)=[-1,1];
        shape=map*(scale.*basis);offset=shape*((lower+upper)/2);
        % This cover is in WORLD course coordinates. Do not add the current
        % ego yaw again to the target's world-course uncertainty.
        g(24,1:n)=0;g(24:27,n+(7:10))=0;
        g(22:27,n+(7:10))=shape.*((upper-lower)/2)';
        if any(offset),g(:,end+1)=0;g(22:27,end)=offset;end
        familyUsed=true;
    end
    predictor=contract.targetObserver.predictor;
    if contract.currentMeasurement.radarDetectionAvailable
        predictor=contract.currentMeasurement.radarRelativePosition;
    end
    previous=contract.currentMeasurement.heldInput;
    assert(numel(previous)==2,'nrmmJointPrediction:missingHeldInput','Coupled prediction needs the applied input.');
    root=struct('center',[state;contract.point;zeros(4,1);previous;contract.positionVelocity.point; ...
        target(1:6);contract.targetObserver.state;predictor], ...
        'generator',g,'parameterColumns',n+(1:4),'targetParameterColumns',n+(7:10), ...
        'posteriorAvailable',false,'targetFamilyRetained',familyUsed,'nonlinearCertificate',false);
end

function tube=localCoupledPropagate(anchor,cfg,initialGenerator,contract,durations,target,errorSet,root,options)
    if nargin<8 || isempty(root)
        root=localCoupledRoot(anchor.states(:,1),initialGenerator,contract,target,errorSet);
    end
    count=size(anchor.inputs,2);center=root.center;generator=root.generator;
    inputRoom=inf(2,count);
    if nargin>=9 && isfield(options,'feedbackInputRadius'),inputRoom=options.feedbackInputRadius;end
    gains=localFeedback(anchor,cfg,durations);targetGains=zeros(size(gains));records=2*count+1;
    tube=struct('kind',"joint-ego-target-observer-affine-v1", ...
        'generator',{cell(1,records)},'relativeGenerator',{cell(1,records)}, ...
        'targetYawRadius',zeros(1,records),'targetGenerator',{cell(1,records)}, ...
        'jointPoseGenerator',{cell(1,records)}, ...
        'observerErrorRadius',zeros(5,count+1),'targetObserverErrorRadius',zeros(6,count+1), ...
        'physicalRadius',zeros(6,records),'mean',zeros(35,count+1), ...
        'inputRadius',zeros(2,count),'inputGenerator',{cell(1,count)}, ...
        'transition',{cell(1,count)},'noiseMap',{cell(1,count)}, ...
        'feedbackGain',gains,'feedbackInputRadius',inputRoom,'futureObserverDynamicsIncluded',true, ...
        'futureTargetObserverDynamicsIncluded',true,'targetFeedbackIncluded',true, ...
        'futurePrimitiveBudgetsAssumed',false,'nonlinearCertificate',false, ...
        'realArithmeticEnclosure',false,'floatingPointCertified',false, ...
        'persistentParameterColumns',root.parameterColumns,'mixedStopBranches',false(1,count), ...
        'time',[0,cumsum(durations)],'root',root, ...
        'predictionSensorSamples',round(durations/contract.samplePeriod), ...
        'scope',"shared nonlinear joint rollout and local tangents; no certified nonlinear remainder");
    tube=localCoupledRecord(tube,1,center(1:6),center(22:27),generator(1:6,:),generator(22:27,:));
    tube.mean(:,1)=center;
    tube.observerErrorRadius(:,1)=localEgoError(center,generator);
    tube.targetObserverErrorRadius(:,1)=localTargetError(center,generator,target(7));
    cache=[];tube.refreshedJointModel=false(1,count);
    for k=1:count
        [reference,policyMap]=localRelativeFeedback(center,anchor.states(3,k));
        gain=gains(:,:,k);targetGain=gain;
        egoMap=zeros(6,35);egoMap(1:5,7:11)=eye(5);egoMap(6,6)=1;
        if k>1
            reserve=sum(abs(gain*egoMap*generator),2)+abs(gain(:,6))*contract.noiseRadius(7);
            physicalRadius=sum(abs(generator(1:6,:)),2);
            course=atan2(abs(center(5)+cfg.vehicle.lf*center(6))+physicalRadius(5)+cfg.vehicle.lf*physicalRadius(6), ...
                max(0,center(4)-physicalRadius(4)));
            authority=[max(0,pi/2-course-abs(anchor.inputs(1,k))-cfg.solver.constraintTolerance); ...
                max(0,min(cfg.actuation.brakingRatioMaximum-anchor.inputs(2,k), ...
                anchor.inputs(2,k)-cfg.actuation.brakingRatioMinimum))];
            % Preserve the existing stabilizing ego feedback first. The
            % additional target response uses the remaining actuator room
            % and RTI's local correction budget. Scaling the entire ego gain
            % by target uncertainty would remove its stabilization.
            gain=gain.*min(1,authority./max(reserve,realmin));
            baseReserve=sum(abs(gain*egoMap*generator),2)+abs(gain(:,6))*contract.noiseRadius(7);
            targetReserve=sum(abs(gain*(policyMap-egoMap)*generator),2);
            targetRoom=min(inputRoom(:,k),max(0,authority-baseReserve));
            targetGain=gain.*min(1,targetRoom./max(targetReserve,realmin));
        end
        context=struct('cfg',cfg,'contract',contract,'input',anchor.inputs(:,k), ...
            'gain',gain,'targetGain',targetGain,'reference',reference, ...
            'egoReference',[center(7:11);center(6)],'referenceYaw',anchor.states(3,k), ...
            'duration',durations(k),'first',k==1, ...
            'targetRearAxleDistance',target(7));
        context.contract=localPredictionContract(contract,cfg,durations(k),k==1);
        tube.predictionSensorSamples(k)=round(durations(k)/context.contract.samplePeriod);
        reuse=localReuseJointModel(cache,center,context,k);
        if reuse
            [matrix,noise,branch]=localCachedJointModel(cache,center,context);
        else
            [~,matrix,noise,branch]=localCoupledLinearize(center,context);
            cache=struct('center',center,'context',context,'step',k,'branch',branch, ...
                'matrix',matrix,'noise',noise,'policy',localPolicyJacobian(center,context));
            tube.refreshedJointModel(k)=true;
        end
        raw=[branch.beforeResetMatrix*generator,branch.beforeResetNoise];
        [point,propagated,mixed]=localResetEnvelope(branch.beforeResetPoint,raw,cfg);
        tube.mixedStopBranches(k)=mixed;
        tube.transition{k}=matrix(1:35,:);tube.noiseMap{k}=noise(1:35,:);
        tube.inputGenerator{k}=propagated(48:49,:);
        tube.inputRadius(:,k)=sum(abs(tube.inputGenerator{k}),2);
        tube=localCoupledRecord(tube,2*k,point(36:41),point(42:47), ...
            propagated(36:41,:),propagated(42:47,:));
        tube=localCoupledRecord(tube,2*k+1,point(1:6),point(22:27), ...
            propagated(1:6,:),propagated(22:27,:));
        if k==1
            tube.firstSuccessor=struct('center',point(1:35),'generator',propagated(1:35,:), ...
                'measurementCenter',branch.measurementCenter, ...
                'measurementGenerator',[[branch.measurementMatrix*generator,branch.measurementNoise], ...
                    zeros(numel(branch.measurementCenter),size(propagated,2)-size(raw,2))], ...
                'mixedStopBranch',mixed, ...
                'time',contract.time+durations(k),'sampleTimes',contract.time+branch.sampleTimes, ...
                'contract',contract,'appliedInput',anchor.inputs(:,1), ...
                'parameterColumns',root.parameterColumns,'nonlinearCertificate',false);
        end
        % Transport the expansion point, not physical uncertainty, to the
        % RTI shooting reference. Keep target world motion and observer bias.
        next=point(1:35);
        [next,transport]=localCoupledReanchor(next,anchor.states(:,k+1));
        generator=transport*propagated(1:35,:);center=next;
        tube.mean(:,k+1)=center;gains(:,:,k)=gain;targetGains(:,:,k)=targetGain;
        tube.observerErrorRadius(:,k+1)=localEgoError(center,generator);
        tube.targetObserverErrorRadius(:,k+1)=localTargetError(center,generator,target(7));
    end
    tube.feedbackGain=gains;tube.targetFeedbackGain=targetGains;tube.finalJointGenerator=generator;
    tube.firstInputRadius=tube.inputRadius(:,1);tube.egoStateRadius=tube.physicalRadius;
    tube.observerErrorTime=tube.time;
    if any(~tube.refreshedJointModel)
        tube.scope="scheduled joint affine prediction with current causal feedback; no certified nonlinear remainder";
    end
end

function reuse=localReuseJointModel(cache,center,context,k)
    cfg=context.cfg;
    reuse=~isempty(cache) && k>2 && string(cfg.solver.method)=="rti" ...
        && ~cfg.controller.retainedTube && ~cfg.controller.jointNonlinearPrediction;
    if ~reuse,return;end
    old=cache.center;
    % Refresh across changes of vehicle regime or substantial maneuver/pose
    % changes. These are engineering scheduling thresholds, not error bounds.
    limits=[.35;max(1,.35*abs(old(4)));1;.6;.35;max(1,.35*abs(old(25)))];
    change=abs(center([3:6,24:25])-old([3:6,24:25]));
    speedChart=sum(center(4)>cfg.model.kinematicSpeedRange);
    oldChart=sum(old(4)>cfg.model.kinematicSpeedRange);
    oldRadar=norm(old(22:23)-old(1:2))<=cfg.collision.encounterRangeMeters;
    radar=norm(center(22:23)-center(1:2))<=cfg.collision.encounterRangeMeters;
    reuse=k-cache.step<cfg.controller.jointPredictionRefreshSteps ...
        && context.duration==cache.context.duration && speedChart==oldChart && radar==oldRadar ...
        && all(change<=limits) && all(abs(context.input-cache.context.input)<=[.25;.4]) ...
        && ((center(25)>0)==(old(25)>0));
end

function policy=localPolicyJacobian(center,context)
    policy=zeros(2,35);
    if context.first,return;end
    [~,map]=localRelativeFeedback(center,context.referenceYaw);
    ego=zeros(6,35);ego(1:5,7:11)=eye(5);ego(6,6)=1;
    policy=context.gain*ego+context.targetGain*(map-ego);
end

function [matrix,noise,branch]=localCachedJointModel(cache,center,context)
    % Remove the old feedback derivative and insert the current causal law.
    % The nominal command is recentered, so it uses the open-loop derivative.
    policy=localPolicyJacobian(center,context);delta=center-cache.center;
    inputDelta=context.input-cache.context.input;
    branch=cache.branch;
    b=branch.beforeResetInputMatrix;
    open=branch.beforeResetMatrix-b*cache.policy;
    branch.beforeResetPoint=branch.beforeResetPoint+open*delta+b*inputDelta;
    branch.beforeResetMatrix=open+b*policy;
    gyro=(context.gain(:,6)-cache.context.gain(:,6))*context.contract.noiseRadius(7);
    branch.beforeResetNoise(:,7)=branch.beforeResetNoise(:,7)+b*gyro;
    b=branch.inputMatrix;matrix=cache.matrix+b*(policy-cache.policy);
    noise=cache.noise;noise(:,7)=noise(:,7)+b*gyro;
    % Autonomous target motion stays analytic even between model refreshes.
    branch.beforeResetPoint(22:27)=localTargetPoint(center(22:27),context.duration,context.targetRearAxleDistance);
    branch.beforeResetPoint(42:47)=localTargetPoint(center(22:27),context.duration/2,context.targetRearAxleDistance);
end

function prediction=localPredictionContract(contract,cfg,duration,first)
    % The issued-command successor keeps the complete actual sensor block.
    % Only later engineering RTI holds use fewer virtual measurements. Keep
    % the stable internal observer integration step and all noise amplitudes.
    prediction=contract;
    requested=cfg.controller.jointPredictionSamplesPerHold;
    if first || requested==0 || string(cfg.solver.method)~="rti" ...
            || cfg.controller.retainedTube || cfg.controller.jointNonlinearPrediction
        return
    end
    samples=min(requested,round(duration/contract.samplePeriod));
    prediction.samplePeriod=duration/samples;
    prediction.integrationSubstepCount=ceil(prediction.samplePeriod/contract.integrationStep);
    prediction.integrationStep=prediction.samplePeriod/prediction.integrationSubstepCount;
    prediction.predictionInverseTolerance=cfg.controller.jointPredictionInverseTolerance;
end

function tube=localCoupledRecord(tube,index,~,~,ge,gt)
    % The same primitive columns drive both vehicles. Form the relative
    % displacement before taking absolute supports; retain yaw correlation.
    relative=[ge(1:2,:)-gt(1:2,:);ge(3,:)];
    tube.generator{index}=ge;tube.relativeGenerator{index}=relative;
    tube.jointPoseGenerator{index}=[ge(1:3,:);gt(1:3,:)];
    tube.targetGenerator{index}=gt;
    tube.targetYawRadius(index)=sum(abs(gt(3,:)));
    tube.physicalRadius(:,index)=sum(abs(ge),2);
end

function radius=localEgoError(center,generator)
    rows=7:11;e=center(rows)-center(1:5);e(3)=atan2(sin(e(3)),cos(e(3)));
    radius=abs(e)+sum(abs(generator(rows,:)-generator(1:5,:)),2);
end

function radius=localTargetError(center,generator,lr)
    [value,jac]=localPhysicalTargetCore(center,lr);
    radius=abs(center(28:33)-value)+sum(abs(generator(28:33,:)-jac*generator),2);
end

function [value,jac]=localPhysicalTargetCore(center,lr)
    yaw=center(3);r=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];j=[0,-1;1,0];
    t=center(22:27);angle=t(3)+t(6);e=[cos(angle);sin(angle)];
    k=sin(t(6))/lr;v=t(4)*e;a=t(5)*e+k*t(4)^2*j*e;
    value=[r'*(t(1:2)-center(1:2));r'*v;r'*a];jac=zeros(6,35);
    jac(1:2,1:2)=-r';jac(1:2,22:23)=r';jac(:,3)=-kron(eye(3),j)*value;
    jac(3:4,24)=r'*j*v;jac(3:4,25)=r'*e;jac(3:4,27)=r'*j*v;
    jac(5:6,24)=r'*j*a;jac(5:6,25)=r'*(2*k*t(4)*j*e);
    jac(5:6,26)=r'*e;jac(5:6,27)=r'*(j*a+cos(t(6))/lr*t(4)^2*j*e);
    if t(4)<=0 && t(5)<0
        value(3:6)=0;jac(3:6,:)=0;
    end
end

function [value,jac]=localRelativeFeedback(center,referenceYaw)
    % Track the nominal ego motion RELATIVE to the estimated target.
    % Target-dependent feedforward arises from the same state-feedback law.
    yaw=center(9);r=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];j=[0,-1;1,0];
    rho=center(28:29);q=center(30:31);
    referenceRotation=[cos(referenceYaw),-sin(referenceYaw);sin(referenceYaw),cos(referenceYaw)];
    transport=referenceRotation'*r;
    % Target velocity is expressed in the FROZEN reference frame. Subtracting
    % raw body-frame q instead would turn an ego yaw perturbation into a
    % fictitious target lateral acceleration and can reverse yaw feedback.
    value=[-r*rho;yaw;center(10:11)-transport*q;center(6)];
    jac=zeros(6,35);jac(1:2,9)=-r*j*rho;jac(1:2,28:29)=-r;
    jac(3,9)=1;jac(4:5,10:11)=eye(2);jac(4:5,30:31)=-transport;
    jac(4:5,9)=-transport*j*q;jac(6,6)=1;
end

function [joint,transport]=localCoupledReanchor(joint,reference)
    before=joint(1:6);r0=[cos(before(3)),-sin(before(3));sin(before(3)),cos(before(3))];
    d=reference(3)-before(3);rotation=[cos(d),sin(d);-sin(d),cos(d)];
    joint(1:21)=localReanchorJointMean(joint(1:21),reference);
    offset=r0'*(reference(1:2)-before(1:2));
    joint(28:29)=rotation*(joint(28:29)-offset);
    joint(30:31)=rotation*joint(30:31);joint(32:33)=rotation*joint(32:33);
    joint(34:35)=rotation*(joint(34:35)-offset);
    transport=eye(35);transport(28:35,28:35)=kron(eye(4),rotation);
end

function [point,matrix,noise,branch]=localCoupledLinearize(center,context,sensorNoise)
    contract=context.contract;cfg=context.cfg;n=35;
    samples=round(context.duration/contract.samplePeriod);channels=9;
    assert(samples>=2 && mod(samples,2)==0 && ...
        abs(samples*contract.samplePeriod-context.duration)<1e-10, ...
        'nrmmJointPrediction:unalignedSensors','A joint hold must resolve the sensor clock and midpoint.');
    if nargin<3,sensorNoise=zeros(channels,samples+1);end
    amplitude=[contract.noiseRadius;repmat(contract.targetObserver.noiseRadius,2,1)];
    dimension=n+channels*(samples+1)+2;inputColumns=dimension-1:dimension;
    gradient=[eye(n),zeros(n,dimension-n)];
    physical=center(1:6);dx=gradient(1:6,:);target=center(22:27);dt=gradient(22:27,:);
    observer=[center(7:11);center(28:35)];dobserver=[gradient(7:11,:);gradient(28:35,:)];
    parameters=center(12:15);dp=gradient(12:15,:);previous=center(16:17);dprevious=gradient(16:17,:);
    frontend=center(18:21);dfrontend=gradient(18:21,:);
    plant=localPlantConfiguration(cfg,parameters);input=context.input;du=zeros(2,dimension);
    if ~context.first
        [feature,map]=localRelativeFeedback(center,context.referenceYaw);delta=feature-context.reference;
        delta(3)=atan2(sin(delta(3)),cos(delta(3)));
        targetGain=context.gain;if isfield(context,'targetGain'),targetGain=context.targetGain;end
        egoReference=zeros(6,1);if isfield(context,'egoReference'),egoReference=context.egoReference;end
        egoDelta=[center(7:11);center(6)]-egoReference;
        egoDelta(3)=atan2(sin(egoDelta(3)),cos(egoDelta(3)));
        egoMap=zeros(6,n);egoMap(1:5,7:11)=eye(5);egoMap(6,6)=1;
        input=input+context.gain*(egoDelta+[zeros(5,1);sensorNoise(7,1)]) ...
            +targetGain*(delta-egoDelta);
        du=(context.gain*egoMap+targetGain*(map-egoMap))*gradient;
        du(:,n+7)=du(:,n+7)+context.gain(:,6)*amplitude(7);
    end
    du(:,inputColumns)=eye(2);
    assert(abs(input(2))<=1,'nrmmJointPrediction:inputDomain','The future feedback exceeds the Fiala input domain.');
    middle=zeros(12,1);dmiddle=zeros(12,dimension);preReset=zeros(6,1);dpreReset=zeros(6,dimension);
    measured=zeros(7,samples+1);dmeasured=zeros(7*(samples+1),dimension);
    for sample=1:samples+1
        dm=zeros(11,dimension);
        if context.first && sample==1
            measurement=contract.currentMeasurement;frontend=contract.positionVelocity.point;dfrontend(:)=0;
        else
            measuredInput=input;dmeasuredInput=du;
            if sample==1,measuredInput=previous;dmeasuredInput=dprevious;end
            [field,df]=localPlantFieldTangent(physical,measuredInput,plant,cfg,dx,dmeasuredInput,dp);
            measurement=localMeasurement(physical,measuredInput,sensorNoise(1:7,sample),plant,contract,field);
            dm(1:2,:)=dx(1:2,:);dm(5,:)=df(4,:)-physical(5)*dx(6,:)-physical(6)*dx(5,:);
            dm(6,:)=df(5,:)+physical(4)*dx(6,:)+physical(6)*dx(4,:);dm(7,:)=dx(6,:);
            if contract.heldInputAvailable,dm(8:9,:)=dmeasuredInput;end
            r=[cos(physical(3)),-sin(physical(3));sin(physical(3)),cos(physical(3))];
            relative=r'*(target(1:2)-physical(1:2));
            measurement.radarRelativePosition=relative+sensorNoise(8:9,sample);
            measurement.radarDetectionAvailable=norm(relative)<=cfg.collision.encounterRangeMeters;
            dm(10:11,:)=r'*(dt(1:2,:)-dx(1:2,:))-[-relative(2);relative(1)]*dx(3,:);
            columns=n+(sample-1)*channels+(1:channels);
            dm([1:7,10:11],columns)=dm([1:7,10:11],columns)+diag(amplitude);
        end
        if measurement.radarDetectionAvailable,radar=measurement.radarRelativePosition;else,radar=NaN(2,1);end
        measured(:,sample)=[measurement.gnssPosition;measurement.bodyAcceleration;measurement.yawRate;radar];
        dmeasured(7*(sample-1)+(1:7),:)=dm([1,2,5,6,7,10,11],:);
        if sample>samples,break;end
        if ~(context.first && sample==1)
            update=[contract.positionVelocity.gain(1)*eye(2);contract.positionVelocity.gain(2)*eye(2)];
            dfrontend=dfrontend+update*(dm(1:2,:)-dfrontend(1:2,:));
            frontend=nrmmPositionVelocity('pointMeasure',frontend,measurement.gnssPosition,contract.positionVelocity.gain);
            measurement.gnssVelocity=frontend(3:4);dm(3:4,:)=dfrontend(3:4,:);
        end
        h=contract.samplePeriod;r=[cos(observer(3)),-sin(observer(3));sin(observer(3)),cos(observer(3))];
        da=r*dm(5:6,:)+r*[-measurement.bodyAcceleration(2);measurement.bodyAcceleration(1)]*dobserver(3,:);
        frontend=nrmmPositionVelocity('pointPredict',frontend,measurement.bodyAcceleration,observer(3),h);
        dfrontend=[dfrontend(1:2,:)+h*dfrontend(3:4,:)+h^2*da/2;dfrontend(3:4,:)+h*da];
        [observer,dobserver]=localObserverTangent(observer,dobserver,measurement,dm,contract);
        if sample==1
            [f1,d1]=localPlantFieldTangent(physical,input,plant,cfg,dx,du,dp);
        else
            f1=field;d1=df;
        end
        [f2,d2]=localPlantFieldTangent(physical+h*f1/2,input,plant,cfg,dx+h*d1/2,du,dp);
        [f3,d3]=localPlantFieldTangent(physical+h*f2/2,input,plant,cfg,dx+h*d2/2,du,dp);
        [f4,d4]=localPlantFieldTangent(physical+h*f3,input,plant,cfg,dx+h*d3,du,dp);
        physical=physical+h*(f1+2*f2+2*f3+f4)/6;dx=dx+h*(d1+2*d2+2*d3+d4)/6;
        [target,a]=localTargetTransition(target,h,context.targetRearAxleDistance);dt=a*dt;
        if sample==samples/2,middle=[physical;target];dmiddle=[dx;dt];end
        if sample==samples
            preReset=physical;dpreReset=dx;
            [physical,stopped]=nonlinearBicycleModel.sampleEndState(physical,input,plant);
            if stopped,dx(4:6,:)=0;end
        end
    end
    joint=[physical;observer(1:5);parameters;input;frontend;target;observer(6:13)];
    gradient=[dx;dobserver(1:5,:);dp;du;dfrontend;dt;dobserver(6:13,:)];
    point=[joint;middle;input];gradient=[gradient;dmiddle;du];
    matrix=gradient(:,1:n);noise=gradient(:,n+1:end-2);
    before=point;before(1:6)=preReset;derivative=gradient;derivative(1:6,:)=dpreReset;
    branch=struct('measurementCenter',measured,'measurementMatrix',dmeasured(:,1:n), ...
        'measurementNoise',dmeasured(:,n+1:end-2),'sampleTimes',(0:samples)*contract.samplePeriod, ...
        'beforeResetPoint',before,'beforeResetMatrix',derivative(:,1:n), ...
        'beforeResetNoise',derivative(:,n+1:end-2), ...
        'inputMatrix',gradient(:,inputColumns),'beforeResetInputMatrix',derivative(:,inputColumns));
end

function [target,a]=localTargetTransition(initial,h,lr)
    % Differentiate the constant-curvature arc analytically, including the
    % stopping distance. Keep the previous directional secants at guards.
    target=localTargetPoint(initial,h,lr);
    if abs(initial(4)+h*initial(5))>1e-9
        [arc,~,rest]=predictiveSafetyGeometry.targetArc(initial,h);
        ds=[h,h^2/2];speedGradient=[1,h];
        if rest
            ds=[0,0];speedGradient=[0,0];
            if initial(4)*initial(5)<0
                ds=[-initial(4)/initial(5),initial(4)^2/(2*initial(5)^2)];
            end
        end
        curvature=sin(initial(6))/lr;dk=cos(initial(6))/lr;
        angle=curvature*arc/2;
        if abs(angle)<1e-4
            scale=1-angle^2/6+angle^4/120;
            slope=-angle/3+angle^3/30-angle^5/840;
        else
            scale=sin(angle)/angle;slope=(angle*cos(angle)-sin(angle))/angle^2;
        end
        direction=predictiveSafetyGeometry.direction(initial(3)+initial(6)+angle);
        tangent=[-direction(2);direction(1)];
        finish=predictiveSafetyGeometry.direction(target(3)+target(6));
        a=eye(6);a(1:2,3)=arc*scale*tangent;a(1:2,4:5)=finish*ds;
        a(1:2,6)=arc*scale*tangent+arc^2*dk/2*(slope*direction+scale*tangent);
        a(3,4:5)=curvature*ds;a(3,6)=dk*arc;
        a(4,4:5)=speedGradient;
        return
    end
    step=[1e-4;1e-4;1e-5;1e-3;1e-3;1e-5];a=eye(6);
    for j=1:6
        d=zeros(6,1);d(j)=step(j);
        if j==4 && initial(4)<step(j)
            % Differentiate the declared forward-motion side at rest; a
            % negative-speed perturbation would introduce a reverse target.
            a(:,j)=(localTargetPoint(initial+d,h,lr)-target)/step(j);
        else
            a(:,j)=(localTargetPoint(initial+d,h,lr)-localTargetPoint(initial-d,h,lr))/(2*step(j));
        end
    end
end

function target=localTargetPoint(initial,h,lr)
    q=[initial;lr;zeros(4,1)];
    result=predictiveSafetyGeometry.predictTarget(q,h);
    target=result(1:6);
    % Keep the latent acceleration parameter: zero speed plus A<0 is the
    % stopped mode; it must not be independently reinitialized next sample.
    target(5)=initial(5);
end

function result=localConditionCoupled(prior,current,contract)
    % Restrict the affine first-successor history with the ENTIRE sensor block.
    % The constrained primitive set is retained explicitly. Its compressed
    % outer proposal is numerical: no nonlinear containment is claimed.
    diagnostic=struct('method',"completeSensorBlockAffineIntersection", ...
        'accepted',false,'sampleCount',0,'usedAllSamples',false, ...
        'nonlinearCertificate',false,'reason',"missingPrior");
    result=struct('root',current,'diagnostics',diagnostic,'posterior',[]);
    if isempty(prior),return;end
    if ~isequaln(prior.contract.targetObserver.identifier,contract.targetObserver.identifier)
        result.diagnostics.reason="targetIdentityChanged";return;
    end
    if ~isequaln(prior.contract.design,contract.design) || prior.contract.samplePeriod~=contract.samplePeriod
        result.diagnostics.reason="observerContractChanged";return;
    end
    if prior.mixedStopBranch
        result.diagnostics.reason="mixedStopMeasurementMapUnavailable";return;
    end
    if abs(prior.time-contract.time)>1e-9 || ...
            ~isequal(prior.appliedInput,contract.currentMeasurement.heldInput)
        result.diagnostics.reason="timeOrAppliedInputChanged";return;
    end
    history=contract.measurementHistory;times=prior.sampleTimes(2:end);
    observations=zeros(7,numel(times));
    for k=1:numel(times)
        index=find(abs(history.time-times(k))<1e-9,1,'last');
        if isempty(index),result.diagnostics.reason="incompleteSensorBlock";return;end
        observations(:,k)=history.value(:,index);
    end
    result.diagnostics.sampleCount=numel(times);result.diagnostics.usedAllSamples=true;
    expected=prior.measurementCenter(:,2:end);
    if ~isequal(isfinite(expected),isfinite(observations))
        result.diagnostics.reason="detectionBranchChanged";return;
    end
    take=isfinite(observations(:));rows=reshape(8:numel(prior.measurementCenter),[],1);rows=rows(take);
    equal=prior.measurementGenerator(rows,:);rhs=observations(take)-expected(take);
    % Once the entire measured block is known, observer software is a
    % deterministic NONLINEAR update, not an uncertain affine variable.
    % Equating its tangent prediction to its exact update would reject valid
    % histories because of the observer's Taylor remainder alone.
    [knownObserver,knownFrontend]=localObservedBlock(prior,observations);
    received=[contract.point;contract.targetObserver.state];
    difference=knownObserver(1:11)-received;
    difference(3)=atan2(sin(difference(3)),cos(difference(3)));
    residual=max(abs([difference;knownFrontend-contract.positionVelocity.point]));
    result.diagnostics.observerReplayResidual=residual;
    if residual>1e-8
        result.diagnostics.reason="observerHistoryInconsistent";return;
    end
    n=size(prior.generator,2);lower=-ones(n,1);upper=ones(n,1);
    % A small arithmetic tolerance is not a model-error allowance.
    tolerance=1e-9;
    options=optimoptions('linprog','Display','none');
    scaling=max(1,max(abs(equal),[],2));scaled=equal./scaling;right=rhs./scaling;
    candidate=lsqminnorm(scaled,right,1e-12);
    flag=double(all(abs(scaled*candidate-right)<=tolerance) && all(abs(candidate)<=1));
    if ~flag
        try
            [~,~,flag]=linprog(zeros(n,1),[scaled;-scaled],[right;-right]+tolerance, ...
                [],[],lower,upper,options);
        catch exception
            if ~startsWith(string(exception.identifier),"optim:linprog:"),rethrow(exception);end
            result.diagnostics.reason="affineHistorySolverFailure";
            result.diagnostics.solverFailure=string(exception.identifier);return;
        end
    end
    if flag<=0
        result.diagnostics.reason="affineHistoryInconsistent";return;
    end
    for pass=1:4
        for row=1:size(equal,1)
            a=equal(row,:);middle=(lower+upper)/2;radius=(upper-lower)/2;
            residual=rhs(row)-a*middle;support=abs(a)*radius+tolerance*scaling(row);
            active=find(abs(a)>1e-12);
            other=max(0,support-abs(a(active)).*radius(active)');
            first=(residual-other)./a(active);second=(residual+other)./a(active);
            lower(active)=max(lower(active),middle(active)+min(first,second)');
            upper(active)=min(upper(active),middle(active)+max(first,second)');
        end
    end
    if any(lower>upper)
        result.diagnostics.reason="affineContractionInconsistent";return;
    end
    posterior=struct('baseCenter',prior.center,'baseGenerator',prior.generator, ...
        'measurementMatrix',equal,'measurementRhs',rhs,'arithmeticTolerance',tolerance*scaling, ...
        'primitiveLower',lower,'primitiveUpper',upper,'scope',"affineJointHistoryOnly");
    middle=(lower+upper)/2;radius=(upper-lower)/2;
    center=prior.center+prior.generator*middle;
    generator=prior.generator.*radius';
    conditionedMap=scaled.*radius';residual=right-scaled*middle;
    % Retain cancellation created by the observed block. Coefficient-wise
    % interval contraction alone forgets equalities between primitive columns.
    % On C*eta=b, c+G*eta == c+L*b+(G-L*C)*eta for ANY L. This outer image
    % keeps those relations cheaply, without introducing a new disturbance.
    gain=(generator*conditionedMap')*pinv(conditionedMap*conditionedMap');
    center=center+gain*residual;
    generator=[generator-gain*conditionedMap,tolerance*gain];
    % Express the physical proposal about the controller's current nominal.
    % Known software states are replaced after their deterministic history check.
    indices=[1:11,16:35];shift=zeros(35,1);
    shift([1:6,22:27])=center([1:6,22:27])-current.center([1:6,22:27]);
    for j=[3,24],shift(j)=atan2(sin(shift(j)),cos(shift(j)));end
    center(indices)=current.center(indices);
    generator([7:11,16:21,28:35],:)=0;
    generator=[generator,shift];
    % Compress tiny directions in relative coordinates, preserving shared
    % translation and the same persistent physical-parameter variables.
    transform=eye(35);transform(22:24,1:3)=-eye(3);
    outer=compress(nrmmSet(center,generator),.01,transform,[12:15,26:27]);
    root=current;root.center=outer.center;root.generator=outer.generator;
    root.parameterColumns=find(any(root.generator(12:15,:),1));
    root.targetParameterColumns=find(any(root.generator(26:27,:),1));
    root.posteriorAvailable=true;
    result.root=root;result.posterior=posterior;result.diagnostics.accepted=true;
    result.diagnostics.reason="affineHistoryConditioned";
end

function [observer,frontend]=localObservedBlock(prior,observations)
    contract=prior.contract;measurement=contract.currentMeasurement;
    predictor=contract.targetObserver.predictor;
    if measurement.radarDetectionAvailable,predictor=measurement.radarRelativePosition;end
    observer=[contract.point;contract.targetObserver.state;predictor];
    frontend=contract.positionVelocity.point;
    for k=1:size(observations,2)
        if k>1
            sample=observations(:,k-1);
            measurement.gnssPosition=sample(1:2);measurement.bodyAcceleration=sample(3:4);
            measurement.yawRate=sample(5);measurement.radarRelativePosition=sample(6:7);
            measurement.radarDetectionAvailable=all(isfinite(sample(6:7)));
            measurement.heldInput=prior.appliedInput;
            if isfield(measurement,'courseGeometry'),measurement=rmfield(measurement,'courseGeometry');end
            frontend=nrmmPositionVelocity('pointMeasure',frontend,measurement.gnssPosition,contract.positionVelocity.gain);
            measurement.gnssVelocity=frontend(3:4);
        end
        frontend=nrmmPositionVelocity('pointPredict',frontend,measurement.bodyAcceleration,observer(3),contract.samplePeriod);
        observer=localObserverTangent(observer,zeros(13,1),measurement,zeros(11,1),contract);
    end
    frontend=nrmmPositionVelocity('pointMeasure',frontend,observations(1:2,end),contract.positionVelocity.gain);
end
