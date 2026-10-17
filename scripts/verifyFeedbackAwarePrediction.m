function results = verifyFeedbackAwarePrediction(outputFile)
%verifyFeedbackAwarePrediction Research probe of feedback-aware uncertainty.
% Builds an invariant tube for the straight-trim sampled tangent model,
% replays bounded measurement feedback on the nonlinear vehicle, and shows
% target/ego correlation under target-relative feedback. This does not run
% the NRMM observer or certify a nonlinear collision/terminal controller.
    if nargin<1,outputFile="";end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    cfg=collisionAvoidanceControllerConfig();reference=nonlinearBicycleModel.cruise(cfg,0);
    [~,a,b]=nonlinearBicycleModel.sample(reference.state,reference.input,cfg);
    oldGain=reference.gain*[zeros(5,1),eye(5)];
    q=diag([1,4,20,1,1,2]);r=diag([20,1]);gain=-dlqr(a,b,q,r);
    closed=a+b*gain;
    % Six-dimensional current error box, illustrative declared inputs.
    radius=[.05;.05;.005;.02;.02;.005];generator=diag(radius);
    signs=2*(dec2bin(0:63,6)-'0')'-1;
    tube=localLinearTube(closed,b*gain*generator,generator,signs,gain);
    % Offline stress duration is independent of the online RTI work budget.
    count=512;h=cfg.controller.sampleTime;
    times=[0,1,3,5,count*h];steps=round(times/h);
    open=zeros(6,numel(steps));
    for index=1:numel(steps),open(:,index)=sum(abs(a^steps(index)*generator),2);end
    % Greedy bounded-noise stress on the linear model; not a worst-case solver.
    error=generator*signs(:,1);peak=abs(error);
    for index=1:count
        candidates=closed*error+b*gain*generator*signs;
        [~,chosen]=max(sum((candidates./tube.coordinateRadius).^2,1));
        error=candidates(:,chosen);peak=max(peak,abs(error));
    end
    assert(all(peak<=tube.coordinateRadius*(1+1e-9)));
    nonlinear=localNonlinearFeedback(cfg,reference,gain,generator,signs,count);
    target=localTargetRelative(cfg,reference,gain,count);
    relativeTube=localRelativeTube(cfg,reference,a,b,gain,generator,closed);
    bias=localVelocityBias(a,b,oldGain,gain,generator,h);
    results=struct('scope',"localLinearInvariantTubeAndNonlinearFeedbackProbes", ...
        'nonlinearRobustCertificate',false,'nrmmObserverExecuted',false, ...
        'sampleTimeSeconds',h,'horizonSteps',count,'measurementErrorRadius',radius, ...
        'lqrStateWeights',diag(q),'lqrInputWeights',diag(r), ...
        'oldFullStateSpectralRadius',max(abs(eig(a+b*oldGain))), ...
        'newFullStateSpectralRadius',max(abs(eig(closed))), ...
        'feedbackGain',gain,'linearTube',tube,'timeSeconds',times, ...
        'openLoopRadius',open,'linearStressPeak',peak, ...
        'nonlinearFeedback',nonlinear,'targetRelative',target,'relativeLinearTube',relativeTube, ...
        'constantVelocityBias',bias);
    if strlength(string(outputFile))>0
        file=fopen(outputFile,'w');assert(file>=0,'Cannot open result file.');
        cleaner=onCleanup(@()fclose(file));
        fprintf(file,'%s\n',jsonencode(results,PrettyPrint=true));
    end
    if nargout==0,disp(results);end
end

function result=localRelativeTube(cfg,reference,a,b,gain,generator,closed)
    tire=modifiedFialaTire.parameters(cfg);
    [~,slope]=nonlinearBicycleModel.roadLoad(reference.state(4),cfg);
    feedforward=[0;slope/sum(tire.longitudinalForceScale)];
    velocity=[0;0;0;1;0;0];position=[1;0;0;0;0;0];
    consistency=norm(a*velocity+b*feedforward-velocity-cfg.controller.sampleTime*position);
    assert(consistency<1e-10);
    % Nine shared measurement-error coordinates: ego(6), target p(2), V.
    targetMap=[-gain(:,1:2),feedforward-gain(:,4)];
    measurementMap=[gain*generator,targetMap*diag([.02,.02,.1])];
    initial=diag([.07,.07,.005,.12,.02,.005]);
    signs=2*(dec2bin(0:511,9)-'0')'-1;
    initialSigns=2*(dec2bin(0:63,6)-'0')'-1;
    % Same construction; initial support has six coordinates, noise has nine.
    tube=localLinearTube(closed,b*measurementMap,initial,signs,gain,initialSigns,measurementMap);
    inputLimit=abs(reference.input)+abs(feedforward)*.1+tube.inputCorrectionRadius;
    assert(inputLimit(2)<1);
    gap=6-tube.coordinateRadius(1)-cfg.vehicle.length ...
        -cfg.vehicle.width/2*tube.coordinateRadius(3);
    assert(all(cfg.vehicle.rectangleOffset==0));
    assert(gap>0);
    result=struct('model',"linearizedRelativeFollowingConstantUnknownTargetSpeed", ...
        'targetSpeedIntervalMetersPerSecond',reference.state(4)+[-.1,.1], ...
        'targetMeasurementRadius',[.02;.02;.1], ...
        'currentRelativeErrorRadius',diag(initial),'fixedRelativeCoordinateRadius',tube.coordinateRadius, ...
        'gainFeedforwardConsistencyResidual',consistency, ...
        'maximumAbsoluteInputFromLinearTube',inputLimit, ...
        'longitudinalRectangleGapLowerBoundMeters',gap,'requiredSafetyMarginMeters',0, ...
        'coversAllAdmittedLinearMeasurementSequences',true,'nonlinearCertificate',false);
end

function result=localVelocityBias(a,b,oldGain,gain,generator,h)
    measurementError=generator(:,4);old=a+b*oldGain;closed=a+b*gain;
    time=[5,25.6,100];steps=round(time/h);oldError=zeros(6,1);newError=zeros(6,1);
    oldPosition=zeros(size(time));newPosition=zeros(size(time));
    for index=1:max(steps)
        oldError=old*oldError+b*oldGain*measurementError;
        newError=closed*newError+b*gain*measurementError;
        at=find(index==steps);
        if ~isempty(at),oldPosition(at)=oldError(1);newPosition(at)=newError(1);end
    end
    result=struct('model',"sampledStraightTrimTangentOnly", ...
        'velocityMeasurementBiasMetersPerSecond',measurementError(4), ...
        'timeSeconds',time,'oldGainPositionErrorMeters',oldPosition, ...
        'fullGainPositionErrorMeters',newPosition, ...
        'fullGainSteadyPositionErrorMeters',((eye(6)-closed)\(b*gain*measurementError))'*[1;0;0;0;0;0]);
end

function tube=localLinearTube(closed,noise,initial,signs,gain,initialSigns,inputNoise)
    if nargin<6,initialSigns=signs;end
    if nargin<7,inputNoise=gain*initial;end
    metric=dlyap(closed',eye(6));factor=chol(metric);
    contraction=norm(factor*closed/factor,2);
    assert(contraction<1);
    initialSize=max(vecnorm(factor*initial*initialSigns));
    count=256;power=eye(6);coordinate=zeros(6,1);inputRadius=zeros(2,1);
    initialCoordinate=zeros(6,1);initialInput=zeros(2,1);
    for index=1:count
        coordinate=coordinate+sum(abs(power*noise),2);
        inputRadius=inputRadius+sum(abs(gain*power*noise),2);
        initialCoordinate=max(initialCoordinate,sum(abs(power*initial),2));
        initialInput=max(initialInput,sum(abs(gain*power*initial),2));power=closed*power;
    end
    tailNoise=max(vecnorm(factor*power*noise*signs));
    noiseTail=tailNoise/(1-contraction);
    initialTail=max(vecnorm(factor*power*initial*initialSigns));
    inverseMetric=metric\eye(6);
    shape=sqrt(diag(inverseMetric));inputShape=sqrt(diag(gain*inverseMetric*gain'));
    coordinate=coordinate+noiseTail*shape+max(initialCoordinate,initialTail*shape);
    inputRadius=inputRadius+noiseTail*inputShape+max(initialInput,initialTail*inputShape) ...
        +sum(abs(inputNoise),2);
    % R is a finite disturbance zonotope plus a noise-tail ellipsoid and
    % the convex hull of the initial box's first M images and its tail
    % ellipsoid. Both components are invariant (the first with added noise).
    % Its coordinate box is an outer bound, not itself asserted invariant.
    assert(tailNoise+contraction*noiseTail<=noiseTail*(1+1e-12));
    tube=struct('model',"sampledStraightTrimTangentOnly",'metric',metric, ...
        'contraction',contraction,'zonotopeTerms',count, ...
        'initialMetricRadius',initialSize,'lastNoiseMetricRadius',tailNoise, ...
        'initialHullTerms',count,'noiseTailEllipsoidRadius',noiseTail, ...
        'initialTailEllipsoidRadius',initialTail,'coordinateRadius',coordinate, ...
        'inputCorrectionRadius',inputRadius);
end

function result=localNonlinearFeedback(cfg,reference,gain,generator,signs,count)
    chosen=[1,9,17,25,33,41,49,64];h=cfg.controller.sampleTime;
    peak=zeros(6,1);braking=0;steering=0;inputAvailable=true;
    for world=chosen
        x=reference.state+generator*signs(:,world);peak=max(peak,abs(x-reference.state));
        for index=1:count
            nominal=reference.state;nominal(1)=nominal(1)+nominal(4)*(index-1)*h;
            % All worlds have the same first estimate. Future estimates obey
            % the illustrative bound, without revealing the true state exactly.
            estimate=nominal;
            if index>1,estimate=x+generator*sin((1:6)'*.7*index+world);end
            input=reference.input+gain*(estimate-nominal);
            inputAvailable=inputAvailable && abs(input(2))<1;
            assert(inputAvailable,'Feedback exceeded the declared braking domain.');
            braking=max(braking,abs(input(2)));steering=max(steering,abs(input(1)));
            x=nonlinearBicycleModel.sample(x,input,cfg);
            nominal(1)=nominal(1)+nominal(4)*h;peak=max(peak,abs(x-nominal));
        end
    end
    result=struct('worldCount',numel(chosen),'initialCornerIndices',chosen, ...
        'measurementSequence',"deterministicBoundedSines",'peakStateError',peak, ...
        'maximumAbsoluteSteeringRadians',steering,'maximumAbsoluteBrakingRatio',braking, ...
        'inputAvailable',inputAvailable,'nonlinearCertificate',false);
end

function result=localTargetRelative(cfg,reference,gain,count)
    h=cfg.controller.sampleTime;separation=6;offset=[-.1,.1];
    finalEgo=zeros(1,2);finalTarget=zeros(1,2);relative=zeros(1,2);
    peak=zeros(1,2);first=zeros(2,2);tire=modifiedFialaTire.parameters(cfg);
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    minimumDistance=Inf;
    for world=1:2
        x=reference.state;speed=reference.state(4)+offset(world);
        for index=1:count
            time=(index-1)*h;targetPosition=separation+speed*time;
            input=reference.input;
            if index>1
                targetSpeedEstimate=speed+.02*cos(.7*index);
                nominal=[targetPosition-separation+.02*sin(.6*index); ...
                    .02*sin(.3*index);0;targetSpeedEstimate;0;0];
                estimate=x+diag([.01;.01;.001;.01;.01;.001])*sin((1:6)'*.4*index);
                trim=[0;nonlinearBicycleModel.roadLoad(targetSpeedEstimate,cfg)/sum(tire.longitudinalForceScale)];
                input=trim+gain*(estimate-nominal);
            end
            if index==1,first(:,world)=input;end
            assert(abs(input(2))<1);
            x=nonlinearBicycleModel.sample(x,input,cfg);
            targetPosition=separation+speed*(time+h);
            peak(world)=max(peak(world),abs(x(1)-targetPosition+separation));
            distance=predictiveSafetyGeometry.rectangle(x(1:3),shape,[targetPosition;0;0],shape);
            minimumDistance=min(minimumDistance,distance);
        end
        finalEgo(world)=x(1);finalTarget(world)=targetPosition;
        relative(world)=x(1)-targetPosition;
    end
    assert(isequal(first(:,1),first(:,2)));
    result=struct('nominalTargetSpeedMetersPerSecond',reference.state(4), ...
        'trueSpeedOffsetsMetersPerSecond',offset,'separationMeters',separation, ...
        'currentSpeedErrorBoundMetersPerSecond',.1,'futureSpeedErrorBoundMetersPerSecond',.1, ...
        'injectedFutureSpeedErrorAmplitudeMetersPerSecond',.02, ...
        'futurePositionErrorBoundMeters',.02,'firstInputs',first, ...
        'terminalTimeSeconds',count*h,'targetPositionDiameterMeters',diff(finalTarget), ...
        'egoPositionDiameterMeters',diff(finalEgo),'relativePositionDiameterMeters',abs(diff(relative)), ...
        'peakRelativeTrackingErrorMeters',peak,'minimumSampledRectangleDistanceMeters',minimumDistance, ...
        'independentMarginalLongitudinalGapMeters',min(finalTarget)-max(finalEgo)-cfg.vehicle.length, ...
        'targetTrajectoryPhysicalUncertaintyRemoved',false);
end
