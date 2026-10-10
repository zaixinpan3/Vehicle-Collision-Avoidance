function result = alternativeCertificate(points,options)
%alternativeCertificate Terminal-controller certificate under alternative constructions.
% Study script of report/TERMINAL_SET_ALTERNATIVES_20261010. It repeats the
% robust-LMI synthesis of scripts/synthesizeClfMatrices.m (samples on the
% set, zonotope enclosure of the scaled Jacobian deviations, LMI with
% Petersen's lemma, iteration to self-consistency, eigenvalue verification,
% hold factor, independent sampled check) with options the production script
% does not have:
%   Enclosure   "tangent": pointwise one-hold Jacobians along the controller
%               (production). "secant": the ray-averaged Jacobians
%               Gbar(e) = int_0^1 G(t e, t u(e)) dt (Gauss-Legendre), which
%               are the matrices the mean-value identity g(e,u) = Gbar(e)[e;u]
%               uses; for a saturating tire their spread is that of the
%               secants, not of the tangents.
%   InputMap    "steering": deviations of [steering; braking] (production).
%               "force": the virtual input w = [front lateral force; braking]
%               deviations; the steering angle follows from the inverse Fiala
%               curve at the current slip geometry (feedback linearization of
%               the front tire). The input box is then [fraction of the front
%               capacity; braking ratio].
%   Lyapunov    "common": one quadratic V = e'Pe. "polyQuadratic": V =
%               e'P(theta(e))e, P affine in the two normalized axle slips
%               theta (ray-averaged in secant mode), with a common slack G and
%               a constant gain (de Oliveira-Bernussou-Geromel form), all
%               pairs (current vertex, next vertex).
%   Saturation  false: |K e| <= ubar on the set (production). true: the
%               controller is sat(K e) at ubar, certified through Hu-Lin's
%               convex hull with an auxiliary gain H, |H e| <= ubar on the set
%               and |K e| <= SaturationGainBound ubar.
%   points      a struct array (speed, curvature, optional configuration
%               override); several points give one certificate for all of
%               them (one P and K in the error coordinates of each trim).
% The result records the extents of the certified set (for polyQuadratic the
% extents of the inner set, the intersection of the vertex ellipsoids, and of
% their union), the certified contraction, the hold factor, the maxima of the
% inputs and slips, and the independent sampled check.
    arguments
        points struct
        options.Enclosure (1,1) string {mustBeMember(options.Enclosure,["tangent","secant"])} = "tangent"
        options.InputMap (1,1) string {mustBeMember(options.InputMap,["steering","force"])} = "steering"
        options.Lyapunov (1,1) string {mustBeMember(options.Lyapunov,["common","polyQuadratic"])} = "common"
        options.Saturation (1,1) logical = false
        options.SaturationGainBound (1,1) double {mustBeGreaterThanOrEqual(options.SaturationGainBound,1)} = 4
        options.TimeConstant (1,1) double = NaN
        options.Box (5,1) double = nan(5,1)
        options.InputBox (2,1) double = nan(2,1)
        options.Components (1,1) double {mustBePositive,mustBeInteger} = 6
        options.SchedulingComponents (1,1) double {mustBeNonnegative,mustBeInteger} = 3
        options.Margin (1,1) double {mustBeGreaterThanOrEqual(options.Margin,1)} = 1.25
        options.BoundarySamples (1,1) double {mustBePositive,mustBeInteger} = 1200
        options.InteriorSamples (1,1) double {mustBePositive,mustBeInteger} = 600
        options.Iterations (1,1) double {mustBeNonnegative,mustBeInteger} = 4
        options.HoldSubdivisions (1,1) double {mustBePositive,mustBeInteger} = 10
        options.Quadrature (1,1) double {mustBePositive,mustBeInteger} = 8
        options.Seed (1,1) double {mustBeNonnegative,mustBeInteger} = 20261010
        options.Verbose (1,1) logical = true
        options.Label (1,1) string = ""
        options.DesignMargin (1,1) double {mustBePositive} = 1e-3
        options.StartBox (5,1) double = [1;.15;.75;.15;.08]
        options.Growth (1,1) double {mustBeGreaterThan(options.Growth,1)} = 1.5
        options.GainStep (1,1) double {mustBeNonnegative} = .25
    end
    root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'));
    if isempty(which('sdpvar')),addpath(genpath(fullfile(root,'solver','YALMIP')));end
    if isempty(which('sedumi')),addpath(genpath(fullfile(root,'solver','sedumi')));end
    timer=tic;
    refs=localReferences(points,options);cfg=refs(1).cfg;
    h=cfg.controller.sampleTime;T=cfg.clf.convergenceTimeConstantSeconds;rho=exp(-2*h/T);design=rho*(1-options.DesignMargin);
    box=options.Box;
    if any(isnan(box))
        box=[cfg.clf.certificationLateralMeters;cfg.clf.certificationHeadingRadians;cfg.clf.certificationSpeedMetersPerSecond; ...
            cfg.clf.certificationLateralVelocityMetersPerSecond;cfg.clf.certificationYawRateRadiansPerSecond];
    end
    ubar=options.InputBox;
    if any(isnan(ubar)),ubar=[cfg.clf.certificationSteeringRadians;cfg.clf.certificationBrakingRatio];end
    if options.InputMap=="force"
        % ubar(1) is a fraction of the front lateral capacity at the trim.
        tire=modifiedFialaTire.parameters(cfg);
        if isnan(options.InputBox(1)),ubar(1)=.4;end
        fraction=ubar(1);ubar(1)=fraction*tire.longitudinalForceScale(1);
        for index=1:numel(refs)
            eta=sqrt(1-(abs(refs(index).input(2))+ubar(2))^2);
            if abs(refs(index).trimForce)+ubar(1)>=.98*tire.longitudinalForceScale(1)*eta
                error('alternativeCertificate:forceBox','The force box reaches the front capacity at speed %g.',refs(index).cfg.referenceSpeed);
            end
        end
    end
    for index=1:numel(refs)
        low=max(-1+1e-8,cfg.actuation.brakingRatioMinimum);high=min(1-1e-8,cfg.actuation.brakingRatioMaximum);
        if refs(index).input(2)-ubar(2)<low || refs(index).input(2)+ubar(2)>high
            error('alternativeCertificate:inputBox','The braking box leaves the actuator range at speed %g.',refs(index).cfg.referenceSpeed);
        end
    end
    scale=struct('state',diag(box),'input',diag(ubar));
    stream=RandStream('mt19937ar','Seed',options.Seed);
    % Nominal (trim) Jacobians of every point; the first point's is the centre
    % of the enclosure, the others enter the initial zonotope.
    g0=zeros(5,7,numel(refs));
    for index=1:numel(refs)
        [a0,b0]=localJacobians(refs(index),zeros(5,1),zeros(2,1),h,options);
        g0(:,:,index)=[scale.state\a0*scale.state,scale.state\b0*scale.input];
    end
    centre=g0(:,:,1);
    initial=localTrimEnclosure(g0,options);
    % 1. The plain LMI (no samples; for several points the zonotope of the
    % trims) gives the first gain and set. The iteration starts from a set
    % inside StartBox (a wide certification box would otherwise give a first
    % enclosure over the whole box).
    [sets,k,fill,ok,aux]=localLmi(centre,initial,design,[],[],options,min(1,options.StartBox./box),[]);
    if ~ok || fill<=1e-6,error('alternativeCertificate:infeasible','The plain LMI is infeasible.');end
    localReport(options,'  plain LMI: fill %.3f, extents %s\n',fill,localExtentsText(sets,box,options));
    % 2. Self-consistent growth. Each iteration encloses the Jacobians over
    % Growth times the current set and over inputs within GainStep (scaled)
    % of the current controller, and solves the LMI with the new set inside
    % the inflated set and the new gain within GainStep of the old one on it;
    % the new pair is therefore certified by the enclosure it was designed
    % with, and the set may grow by up to Growth per iteration.
    candidates=struct('sets',{},'k',{},'aux',{},'enclosure',{},'fill',{},'margin',{});
    for iteration=1:options.Iterations
        gamma=options.Growth;
        for attempt=1:3
            enclosure=localEnclosure(refs,scale,centre,gamma*sets,k,h,options,stream,[]);
            [sets2,k2,fill2,ok2,aux2]=localLmi(centre,enclosure,design,[],gamma*sets,options,ones(5,1),struct('k',k,'delta',options.GainStep));
            if ok2 && fill2>1e-6,break;end
            localReport(options,'  iteration %d: growth %.3f infeasible (bounds %s, residual %.4f)\n',iteration,gamma,mat2str(enclosure.coefficientBounds.',3),enclosure.residualNorm);
            gamma=1+(gamma-1)/2;
        end
        if ~ok2 || fill2<=1e-6,break;end
        margin=localVertexMargin(centre,enclosure,rho,k2,sets2,aux2,options);
        localReport(options,'  iteration %d: growth %.3f, bounds %s, residual %.4f, mean-value residual %.2e; fill %.3f, margin %.2e, extents %s\n', ...
            iteration,gamma,mat2str(enclosure.coefficientBounds.',3),enclosure.residualNorm,enclosure.meanValueResidual,fill2,margin,localExtentsText(sets2,box,options));
        candidates(end+1)=struct('sets',sets2,'k',k2,'aux',aux2,'enclosure',enclosure,'fill',fill2,'margin',margin); %#ok<AGROW>
        sets=sets2;k=k2;aux=aux2;
    end
    candidates=candidates([candidates.margin]>=-1e-6);
    if isempty(candidates),error('alternativeCertificate:infeasible','No iteration produced a verified certificate.');end
    [~,order]=max([candidates.fill]);best=candidates(order);
    sets=best.sets;k=best.k;aux=best.aux;enclosure=best.enclosure;fill=best.fill;
    % 3. Bisect the certified contraction against the certificate's own enclosure.
    [minimumEigenvalue,certified]=localVerify(centre,enclosure,rho,k,sets,aux,options);
    % 4. Hold factor from the partial-hold maps on the same samples.
    holdFactor=1;residualHold=0;
    for step=1:options.HoldSubdivisions-1
        duration=h*step/options.HoldSubdivisions;
        [a0s,b0s]=localJacobians(refs(1),zeros(5,1),zeros(2,1),duration,options);
        centreStep=[scale.state\a0s*scale.state,scale.state\b0s*scale.input];
        partial=localEnclosure(refs,scale,centreStep,sets,k,duration,options,stream,enclosure.samples);
        holdFactor=max(holdFactor,exp(duration/T)*localVertexNorm(centreStep,partial,k,sets,aux,options));
        residualHold=max(residualHold,partial.residualNorm);
    end
    holdFactor=max(holdFactor,exp(h/T)*localVertexNorm(centre,enclosure,k,sets,aux,options));
    % 6. Unscale and check independently.
    gain=scale.input*k/scale.state;
    matrices=zeros(5,5,size(sets,3));
    for index=1:size(sets,3)
        p=scale.state\inv(sets(:,:,index))/scale.state;matrices(:,:,index)=(p+p.')/2;
    end
    certificate=struct('gain',gain,'matrices',matrices,'thetaBox',enclosure.thetaBox,'thetaTrim',enclosure.thetaTrim, ...
        'ubar',ubar,'contraction',rho,'box',box);
    check=localCheck(refs,certificate,options,options.Seed+1);
    [inner,outer]=localExtents(sets,box,options);
    result=struct('label',options.Label,'options',options,'speeds',arrayfun(@(r)r.cfg.referenceSpeed,refs), ...
        'curvatures',arrayfun(@(r)r.curvature,refs),'box',box,'inputBox',ubar,'gain',gain,'matrices',matrices, ...
        'extents',inner,'unionExtents',outer,'regionFill',fill,'certifiedContraction',certified,'requiredContraction',rho, ...
        'certifiedTimeConstantSeconds',-2*h/log(certified),'requiredTimeConstantSeconds',T,'holdFactor',holdFactor, ...
        'lmiMinimumEigenvalue',minimumEigenvalue,'coefficientBounds',enclosure.coefficientBounds,'residualNorm',enclosure.residualNorm, ...
        'residualNormWithinHold',residualHold,'thetaBox',enclosure.thetaBox,'meanValueResidual',enclosure.meanValueResidual,'quadratureResidual',enclosure.quadratureResidual, ...
        'maximumInput',max(abs(enclosure.samples.inputs),[],2),'maximumSlipRadians',enclosure.maximumSlip, ...
        'sampledWorstContraction',max(check.contraction),'sampledHoldFactor',max(check.holdFactor), ...
        'sampledRowsSatisfied',all(check.rows),'sampledMaximumSlip',max(check.slip,[],2),'sampledDomainFailures',sum(isnan(check.contraction)), ...
        'scaledGain',k,'scaledSets',sets,'scaledCentre',centre,'enclosureDirections',enclosure.directions, ...
        'thetaDirections',enclosure.thetaDirections,'iterationsUsed',numel(candidates),'elapsedSeconds',toc(timer));
    fprintf(['%s speeds %s curvatures %s: extents [%.3f m %.4f rad %.3f m/s %.3f m/s %.4f rad/s]%s, fill %.3f; ' ...
        'certified T %.2f s (required %.2f), hold factor %.4f, slips [%.3f %.3f] rad; sampled worst %.4f, rows %d; %.0f s\n'], ...
        options.Label,mat2str(result.speeds),mat2str(result.curvatures),inner,localUnionText(outer,options),fill, ...
        result.certifiedTimeConstantSeconds,T,holdFactor,enclosure.maximumSlip,result.sampledWorstContraction,result.sampledRowsSatisfied,result.elapsedSeconds);
end

function refs=localReferences(points,options)
    refs=struct('cfg',{},'state',{},'input',{},'curvature',{},'trimForce',{});
    for index=1:numel(points)
        override=struct('referenceSpeed',points(index).speed);
        if isfield(points(index),'configuration') && ~isempty(points(index).configuration)
            names=fieldnames(points(index).configuration);
            for n=1:numel(names),override.(names{n})=points(index).configuration.(names{n});end
        end
        if ~isnan(options.TimeConstant)
            if ~isfield(override,'clf'),override.clf=struct();end
            override.clf.convergenceTimeConstantSeconds=options.TimeConstant;
        end
        cfg=collisionAvoidanceControllerConfig(override);
        point=nonlinearBicycleModel.operatingPoint(cfg,points(index).curvature);
        x=point.state;u=point.input;
        slip=atan2(x(5)+cfg.vehicle.lf*x(6),x(4))-u(1);
        force=modifiedFialaTire.evaluate([slip;0],u(2),cfg);
        refs(end+1)=struct('cfg',cfg,'state',x,'input',u,'curvature',points(index).curvature,'trimForce',force(1)); %#ok<AGROW>
    end
end

function [a,b,slip,theta,plus]=localJacobians(ref,errors,inputs,duration,options)
    % Jacobians of the sampled error map g(e, w) over 'duration' seconds at
    % the states x(e) on the trim's lane with the input U(e, w) of the input
    % map (steering: u* + w; force: inverse Fiala), by the chain rule on the
    % RK4 variational equations; the start slips, the normalized slips theta
    % and the error after the hold. NaN where the model's domain is left.
    cfg=ref.cfg;
    curve=struct('origin',[0;0],'heading',0,'curvature',ref.curvature,'length',200);
    lane=struct('referenceCurve',curve);
    [~,heading]=laneGeometry.referencePose(50,0,curve);
    chart=[[-sin(heading);cos(heading)],zeros(2,4);zeros(4,1),eye(4)];
    n=size(errors,2);a=nan(5,5,n);b=nan(5,2,n);slip=nan(2,n);theta=nan(2,n);plus=nan(5,n);
    for j=1:n
        e=errors(:,j);w=inputs(:,j);
        [position,yaw]=laneGeometry.referencePose(50,e(1),curve);
        x=[position;yaw+ref.state(3)+e(2);ref.state(4:6)+e(3:5)];
        [u,due,duw]=localInput(ref,x,w,options);
        if any(~isfinite(u)),continue;end
        try
            [next,ax,bx]=nonlinearBicycleModel.sample(x,u,cfg,[],duration);
        catch exception
            if startsWith(string(exception.identifier),"collisionAvoidanceController:"),continue;end
            rethrow(exception);
        end
        [plus(:,j),jacobian]=nonlinearBicycleModel.errorLinearization(next,lane,ref);
        a(:,:,j)=jacobian*(ax*chart+bx*due);b(:,:,j)=jacobian*bx*duw;
        [slip(:,j),theta(:,j)]=localSlip(cfg,x,u);
    end
end

function [slip,theta]=localSlip(cfg,x,u)
    tire=modifiedFialaTire.parameters(cfg);
    slip=atan2([x(5)+cfg.vehicle.lf*x(6);x(5)-cfg.vehicle.lr*x(6)],x(4))-[u(1);0];
    eta=sqrt(max(0,1-u(2)^2));
    theta=min(1,tire.corneringStiffness.*abs(tan(slip))./(3*tire.longitudinalForceScale*eta));
end

function [u,due,duw]=localInput(ref,x,w,options)
    % Physical input of the controller's virtual input w at state x, with
    % du/de (2x5, e the transverse error) and du/dw (2x2).
    if options.InputMap=="steering"
        u=ref.input+w;due=zeros(2,5);duw=eye(2);return;
    end
    u=[localInverseSteering(ref,x,w);ref.input(2)+w(2)];
    if ~isfinite(u(1)),due=nan(2,5);duw=nan(2);return;end
    due=zeros(2,5);duw=[0,0;0,1];
    % Central differences of the steering inverse in the speeds and in w.
    for index=3:5
        step=1e-6*max(1,abs(x(index+1)));xp=x;xm=x;xp(index+1)=xp(index+1)+step;xm(index+1)=xm(index+1)-step;
        due(1,index)=(localInverseSteering(ref,xp,w)-localInverseSteering(ref,xm,w))/(2*step);
    end
    steps=[1e-3*max(1,abs(w(1)));1e-7];
    for index=1:2
        wp=w;wm=w;wp(index)=wp(index)+steps(index);wm(index)=wm(index)-steps(index);
        duw(1,index)=(localInverseSteering(ref,x,wp)-localInverseSteering(ref,x,wm))/(2*steps(index));
    end
end

function delta=localInverseSteering(ref,x,w)
    % Steering angle whose front lateral force is trimForce + w(1) at braking
    % ratio b* + w(2): Fiala's curve inverted below its peak.
    cfg=ref.cfg;tire=modifiedFialaTire.parameters(cfg);
    b=ref.input(2)+w(2);eta=sqrt(max(0,1-b^2));capacity=tire.longitudinalForceScale(1)*eta;
    force=ref.trimForce+w(1);
    if abs(force)>=capacity,delta=NaN;return;end
    q=1-(1-abs(force)/capacity)^(1/3);
    alpha=-sign(force)*atan(3*capacity*q/tire.corneringStiffness(1));
    delta=atan2(x(5)+cfg.vehicle.lf*x(6),x(4))-alpha;
end

function inputs=localController(k,scaled,scale,options)
    % Physical virtual inputs of the controller at the scaled errors.
    inputs=scale.input*k*scaled;
    if options.Saturation
        limit=diag(scale.input);inputs=min(limit,max(-limit,inputs));
    end
end

function [nodes,weights]=localGauss(n)
    beta=.5./sqrt(1-(2*(1:n-1)).^(-2));t=diag(beta,1)+diag(beta,-1);
    [v,d]=eig(t);[nodes,order]=sort(diag(d));weights=2*v(1,order).^2;
    nodes=(nodes.'+1)/2;weights=weights/2;
end

function enclosure=localTrimEnclosure(g0,options)
    % Zonotope of the trim Jacobians of several points about the first (empty
    % for one point).
    x=reshape(g0-g0(:,:,1),35,[]);x=x(:,2:end);
    enclosure=struct('directions',zeros(5,7,0),'coefficientBounds',zeros(0,1),'residualNorm',0, ...
        'thetaDirections',zeros(5,7,0),'thetaBox',zeros(2,2),'thetaTrim',zeros(2,1));
    if isempty(x),return;end
    [u,~,~]=svd(x,'econ');m=min(options.Components,size(u,2));u=u(:,1:m);
    coefficients=u.'*x;bounds=options.Margin*max(abs(coefficients),[],2);
    residual=x-u*coefficients;eps=0;
    for j=1:size(x,2),eps=max(eps,norm(reshape(residual(:,j),5,7)));end
    enclosure.residualNorm=options.Margin*eps;enclosure.coefficientBounds=bounds;
    enclosure.directions=zeros(5,7,m);
    for index=1:m,enclosure.directions(:,:,index)=bounds(index)*reshape(u(:,index),5,7);end
    if options.Lyapunov=="polyQuadratic"
        enclosure.thetaDirections=zeros(5,7,2);
    end
end

function enclosure=localEnclosure(refs,scale,centre,sets,k,duration,options,stream,samples)
    % Zonotope enclosure of the scaled Jacobian deviations along the
    % controller over the set(s): boundary-heavy samples (per ellipsoid, per
    % point), tangent or ray-averaged Jacobians, the physical scheduling
    % regression (polyQuadratic), the leading principal directions of the
    % remainder with their bounds and the residual norm, widened by Margin.
    if isempty(samples)
        count=size(sets,3);total=options.BoundarySamples+options.InteriorSamples;
        scaled=zeros(5,0);
        for index=1:count
            lower=chol(sets(:,:,index),'lower');
            nb=ceil(options.BoundarySamples/count);ni=ceil(options.InteriorSamples/count);
            d=randn(stream,5,nb+ni);radius=[ones(1,nb),sqrt(rand(stream,1,ni))];
            scaled=[scaled,lower*(d./vecnorm(d)).*radius]; %#ok<AGROW>
        end
        scaled=scaled(:,1:min(end,max(total,size(scaled,2))));
        refIndex=randi(stream,numel(refs),1,size(scaled,2));
        % Inputs within GainStep (scaled) of the controller's, clipped to the
        % input box when the controller saturates.
        inputs=localController(k,scaled,scale,options)+scale.input*(options.GainStep*(2*rand(stream,2,size(scaled,2))-1));
        if options.Saturation,limit=diag(scale.input);inputs=min(limit,max(-limit,inputs));end
        samples=struct('scaled',scaled,'refIndex',refIndex,'inputs',inputs);
    end
    scaled=samples.scaled;n=size(scaled,2);errors=scale.state*scaled;inputs=samples.inputs;
    if options.Enclosure=="secant",[nodes,weights]=localGauss(options.Quadrature);else,nodes=1;weights=1;end
    x=zeros(35,n);theta=zeros(2,n);slip=zeros(2,n);meanValue=0;quadrature=0;
    for r=1:numel(refs)
        columns=find(samples.refIndex==r);if isempty(columns),continue;end
        e=errors(:,columns);w=inputs(:,columns);m=numel(columns);
        abar=zeros(5,5,m);bbar=zeros(5,2,m);tbar=zeros(2,m);
        for q=1:numel(nodes)
            [a,b,s,t]=localJacobians(refs(r),nodes(q)*e,nodes(q)*w,duration,options);
            assert(all(isfinite(a(:))) && all(isfinite(b(:))),'alternativeCertificate:enclosure','A sample left the model domain.');
            abar=abar+weights(q)*a;bbar=bbar+weights(q)*b;tbar=tbar+weights(q)*t;
            if nodes(q)==1,slip(:,columns)=s;else,slip(:,columns)=max(slip(:,columns),abs(s));end
        end
        if options.Enclosure=="secant"
            % The mean-value identity g(e,w) = Gbar [e; w]: its relative residual
            % measures the quadrature (an exact identity otherwise).
            [~,~,s1,~,plus]=localJacobians(refs(r),e,w,duration,options);slip(:,columns)=max(slip(:,columns),abs(s1));
            for j=1:m
                predicted=abar(:,:,j)*e(:,j)+bbar(:,:,j)*w(:,j);gap=scale.state\(plus(:,j)-predicted);
                meanValue=max(meanValue,norm(gap)/max(1e-12,norm(scale.state\plus(:,j))));
                % The rank-one matrix that makes the identity exact for this
                % sample has this norm; it joins the residual below.
                quadrature=max(quadrature,norm(gap)/norm([scale.state\e(:,j);scale.input\w(:,j)]));
            end
        end
        for j=1:m
            x(:,columns(j))=reshape([scale.state\abar(:,:,j)*scale.state,scale.state\bbar(:,:,j)*scale.input]-centre,[],1);
        end
        theta(:,columns)=tbar;
    end
    slip=abs(slip);
    [~,thetaTrim]=localSlip(refs(1).cfg,[0;0;refs(1).state(3:6)],refs(1).input);
    enclosure=struct('samples',samples,'maximumSlip',max(slip,[],2),'meanValueResidual',meanValue,'quadratureResidual',quadrature,'theta',theta, ...
        'thetaTrim',thetaTrim,'thetaDirections',zeros(5,7,0),'thetaBox',zeros(2,2));
    remainder=x;
    if options.Lyapunov=="polyQuadratic"
        % Regress the deviations on the normalized slips (relative to the
        % trim's); the scheduling box is the sampled range widened by Margin.
        regressors=theta-thetaTrim;
        d=remainder*regressors.'*pinv(regressors*regressors.');
        remainder=remainder-d*regressors;
        enclosure.thetaDirections=reshape(d,5,7,2);
        enclosure.thetaBox=[min(0,options.Margin*min(regressors,[],2)),max(0,options.Margin*max(regressors,[],2))];
        components=options.SchedulingComponents;
    else
        components=options.Components;
    end
    if components>0
        [u,~,~]=svd(remainder,'econ');m=min(components,size(u,2));u=u(:,1:m);
        coefficients=u.'*remainder;bounds=options.Margin*max(abs(coefficients),[],2);
        residual=remainder-u*coefficients;
    else
        u=zeros(35,0);bounds=zeros(0,1);residual=remainder;m=0;
    end
    eps=0;for j=1:n,eps=max(eps,norm(reshape(residual(:,j),5,7)));end
    enclosure.residualNorm=options.Margin*(eps+quadrature);enclosure.coefficientBounds=bounds;enclosure.basis=u;
    enclosure.directions=zeros(5,7,m);
    for index=1:m,enclosure.directions(:,:,index)=bounds(index)*reshape(u(:,index),5,7);end
end

function [vertices,current]=localVertices(centre,enclosure,options)
    % Vertex matrices of the zonotope (5x7xV) and, for polyQuadratic, the
    % index of the scheduling vertex each belongs to.
    m=size(enclosure.directions,3);signs=(dec2bin(0:2^m-1)-'0')*2-1;if m==0,signs=zeros(1,0);end
    base=zeros(5,7,size(signs,1));
    for v=1:size(signs,1)
        ab=centre;for index=1:m,ab=ab+signs(v,index)*enclosure.directions(:,:,index);end
        base(:,:,v)=ab;
    end
    if options.Lyapunov=="polyQuadratic" && ~isempty(enclosure.thetaDirections)
        corners=localCorners(enclosure.thetaBox);
        vertices=zeros(5,7,0);current=zeros(1,0);
        for i=1:4
            shift=enclosure.thetaDirections(:,:,1)*corners(1,i)+enclosure.thetaDirections(:,:,2)*corners(2,i);
            vertices=cat(3,vertices,base+shift);current=[current,i*ones(1,size(base,3))]; %#ok<AGROW>
        end
    else
        vertices=base;current=ones(1,size(base,3));
    end
end

function corners=localCorners(thetaBox)
    % Corners (lo,lo), (hi,lo), (lo,hi), (hi,hi) of the scheduling box.
    corners=[thetaBox(1,1),thetaBox(1,2),thetaBox(1,1),thetaBox(1,2);thetaBox(2,1),thetaBox(2,1),thetaBox(2,2),thetaBox(2,2)];
end

function patterns=localPatterns(options)
    if options.Saturation,patterns=cat(3,zeros(2),diag([1,0]),diag([0,1]),eye(2));else,patterns=eye(2);end
end

function [sets,k,fill,ok,aux]=localLmi(centre,enclosure,rho,fixed,contain,options,limit,previous)
    % maximize t subject to the vertex inequalities (Petersen's lemma for the
    % residual), the input rows, the box rows and S >= t I, in scaled
    % coordinates (unit box, unit input square). fixed: struct with the gain
    % k (and aux.h) to keep; contain: S' <= contain.
    if nargin<7,limit=ones(5,1);end
    if nargin<8,previous=[];end
    [vertices,current]=localVertices(centre,enclosure,options);count=size(vertices,3);eps=enclosure.residualNorm;
    patterns=localPatterns(options);np=size(patterns,3);
    t=sdpvar(1);constraints=[t>=0];
    if options.Lyapunov=="common"
        s=sdpvar(5,5);
        if isempty(fixed),y=sdpvar(2,5,'full');else,y=fixed.k*s;end
        if options.Saturation
            if isempty(fixed),z=sdpvar(2,5,'full');else,z=fixed.aux.h*s;end
        else
            z=y;
        end
        constraints=[constraints,s>=t*eye(5)];
        for i=1:5,constraints=[constraints,s(i,i)<=limit(i)^2];end %#ok<AGROW>
        for j=1:2
            constraints=[constraints,[1,z(j,:);z(j,:).',s]>=0]; %#ok<AGROW>
            if options.Saturation,constraints=[constraints,[options.SaturationGainBound^2,y(j,:);y(j,:).',s]>=0];end %#ok<AGROW>
        end
        if ~isempty(contain),constraints=[constraints,s<=contain];end
        if ~isempty(previous) && isempty(fixed)
            % |(K - K_old)_j e| <= delta on the set: the enclosure's input band.
            for j=1:2,constraints=[constraints,[previous.delta^2,y(j,:)-previous.k(j,:)*s;(y(j,:)-previous.k(j,:)*s).',s]>=0];end %#ok<AGROW>
        end
        lambda=sdpvar(count*np,1);
        for v=1:count
            for q=1:np
                d=patterns(:,:,q);f=d*y+(eye(2)-d)*z;
                n=vertices(:,1:5,v)*s+vertices(:,6:7,v)*f;zr=eps*[s;f];index=(v-1)*np+q;
                if eps>0
                    constraints=[constraints,[rho*s,n.',zr.';n,s-lambda(index)*eye(5),zeros(5,7);zr,zeros(7,5),lambda(index)*eye(7)]>=0,lambda(index)>=0]; %#ok<AGROW>
                else
                    constraints=[constraints,[rho*s,n.';n,s]>=0]; %#ok<AGROW>
                end
            end
        end
        diagnostics=optimize(constraints,-t,sdpsettings('solver','sedumi','verbose',0));
        ok=localSolved(diagnostics,constraints,options);s=value(s);s=(s+s.')/2;sets=s;fill=value(t);
        if isempty(fixed),k=value(y)/s;else,k=fixed.k;end
        aux=struct('h',[]);
        if options.Saturation
            if isempty(fixed),aux.h=value(z)/s;else,aux.h=fixed.aux.h;end
        end
    else
        % Poly-quadratic: S_i per scheduling corner, common G and R = K G.
        if options.Saturation,error('alternativeCertificate:unsupported','Saturation with polyQuadratic is not implemented.');end
        s=cell(1,4);for i=1:4,s{i}=sdpvar(5,5);end
        g=sdpvar(5,5,'full');
        if isempty(fixed),r=sdpvar(2,5,'full');else,r=fixed.k*g;end
        for i=1:4
            constraints=[constraints,s{i}>=t*eye(5)]; %#ok<AGROW>
            for d=1:5,constraints=[constraints,s{i}(d,d)<=limit(d)^2];end %#ok<AGROW>
            for j=1:2,constraints=[constraints,[1,r(j,:);r(j,:).',g+g.'-s{i}]>=0];end %#ok<AGROW>
            if ~isempty(contain),constraints=[constraints,s{i}<=contain(:,:,i)];end %#ok<AGROW>
            if ~isempty(previous) && isempty(fixed)
                for j=1:2,constraints=[constraints,[previous.delta^2,r(j,:)-previous.k(j,:)*g;(r(j,:)-previous.k(j,:)*g).',g+g.'-s{i}]>=0];end %#ok<AGROW>
            end
        end
        lambda=sdpvar(count*4,1);zr=eps*[g;r];
        for v=1:count
            i=current(v);n=vertices(:,1:5,v)*g+vertices(:,6:7,v)*r;x=rho*(g+g.'-s{i});
            for j=1:4
                index=(v-1)*4+j;
                if eps>0
                    constraints=[constraints,[x,n.',zr.';n,s{j}-lambda(index)*eye(5),zeros(5,7);zr,zeros(7,5),lambda(index)*eye(7)]>=0,lambda(index)>=0]; %#ok<AGROW>
                else
                    constraints=[constraints,[x,n.';n,s{j}]>=0]; %#ok<AGROW>
                end
            end
        end
        diagnostics=optimize(constraints,-t,sdpsettings('solver','sedumi','verbose',0));
        ok=localSolved(diagnostics,constraints,options);fill=value(t);sets=zeros(5,5,4);
        for i=1:4,si=value(s{i});sets(:,:,i)=(si+si.')/2;end
        if isempty(fixed),k=value(r)/value(g);else,k=fixed.k;end
        aux=struct('h',[]);
    end
end

function ok=localSolved(diagnostics,constraints,options)
    % Solved, or flagged for numerical problems (the slack G of the
    % poly-quadratic form leaves a flat direction) with every constraint
    % satisfied to 1e-7 at the returned point.
    ok=diagnostics.problem==0;
    if ~ok && diagnostics.problem==4
        residuals=check(constraints);ok=all(isfinite(residuals)) && min(residuals)>=-1e-7;words=["rejected","accepted"];
        localReport(options,'    LMI flagged numerical problems; least constraint residual %.2e, %s\n',min(residuals),words(ok+1));
    elseif ~ok
        localReport(options,'    LMI: %s\n',diagnostics.info);
    end
end

function margin=localVertexMargin(centre,enclosure,rho,k,sets,aux,options)
    % Least relative eigenvalue over the vertex blocks (Schur-reduced, best
    % multiplier on a grid), with the final sets and gain.
    [vertices,current]=localVertices(centre,enclosure,options);eps=enclosure.residualNorm;
    patterns=localPatterns(options);margin=Inf;
    for v=1:size(vertices,3)
        for q=1:size(patterns,3)
            d=patterns(:,:,q);
            if options.Saturation,f=d*k+(eye(2)-d)*aux.h;else,f=k;end
            for j=1:size(sets,3)
                si=sets(:,:,current(v));sj=sets(:,:,j);
                n=(vertices(:,1:5,v)+vertices(:,6:7,v)*f)*si;z=eps*[si;f*si];
                grid=logspace(-6,0,60)*min(eig(sj));best=-Inf;
                for lambda=grid
                    block=[rho*si-(z.'*z)/lambda,n.';n,sj-lambda*eye(5)];
                    best=max(best,min(eig((block+block.')/2))/norm(block));
                    if eps==0,break;end
                end
                margin=min(margin,best);
            end
        end
    end
end

function [minimumEigenvalue,certified]=localVerify(centre,enclosure,rho,k,sets,aux,options)
    feasible=@(r)localVertexMargin(centre,enclosure,r,k,sets,aux,options);
    minimumEigenvalue=feasible(rho);
    if minimumEigenvalue<-1e-6
        error('alternativeCertificate:certificate','The final certificate fails its own vertex check (relative margin %.3g).',minimumEigenvalue);
    end
    low=0;high=rho;
    for step=1:24
        middle=(low+high)/2;
        if feasible(middle)>=-1e-6,high=middle;else,low=middle;end
    end
    certified=high;
end

function bound=localVertexNorm(centre,enclosure,k,sets,aux,options)
    % max over vertices, patterns and set pairs of ||F_j (A + B F) F_i^-1||
    % plus the residual's share.
    [vertices,current]=localVertices(centre,enclosure,options);eps=enclosure.residualNorm;
    patterns=localPatterns(options);factors=cell(1,size(sets,3));
    for i=1:size(sets,3),factors{i}=chol(inv(sets(:,:,i)));end
    bound=0;
    for v=1:size(vertices,3)
        for q=1:size(patterns,3)
            d=patterns(:,:,q);
            if options.Saturation,f=d*k+(eye(2)-d)*aux.h;else,f=k;end
            closed=vertices(:,1:5,v)+vertices(:,6:7,v)*f;
            for j=1:size(sets,3)
                fi=factors{current(v)};fj=factors{j};
                bound=max(bound,norm(fj*closed/fi)+eps*norm(fj)*norm([eye(5);f]/fi));
            end
        end
    end
end

function check=localCheck(refs,certificate,options,seed)
    % Independent sampled check on the final set: contraction V(x+)/V(x) under
    % the actual controller, hold factor, rows, slips (800 samples per point).
    stream=RandStream('mt19937ar','Seed',seed);
    count=size(certificate.matrices,3);total=800;
    check=struct('contraction',[],'holdFactor',[],'rows',logical([]),'slip',zeros(2,0));
    for r=1:numel(refs)
        ref=refs(r);cfg=ref.cfg;h=cfg.controller.sampleTime;T=cfg.clf.convergenceTimeConstantSeconds;
        curve=struct('origin',[0;0],'heading',0,'curvature',ref.curvature,'length',200);lane=struct('referenceCurve',curve);
        for i=1:count
            factor=chol(certificate.matrices(:,:,i));per=ceil(total/count);
            d=randn(stream,5,per);unit=factor\(d./vecnorm(d));
            errors=unit.*sqrt([ones(1,ceil(per/2)),rand(stream,1,per-ceil(per/2))]);
            for j=1:per
                e=errors(:,j);
                if localValue(e,certificate,ref,options)>1+1e-9,continue;end
                w=certificate.gain*e;
                if options.Saturation,w=min(certificate.ubar,max(-certificate.ubar,w));end
                [position,heading]=laneGeometry.referencePose(50,e(1),curve);
                x=[position;heading+ref.state(3)+e(2);ref.state(4:6)+e(3:5)];
                u=localInput(ref,x,w,options);
                if any(~isfinite(u)),check.contraction(end+1)=NaN;check.holdFactor(end+1)=NaN;check.rows(end+1)=false;check.slip(:,end+1)=NaN;continue;end
                value=localValue(e,certificate,ref,options);
                try
                    [middle,next]=terminalSafeSet.holdStates(x,u,cfg);
                    hold=0;
                    for s=h*(1:options.HoldSubdivisions)/options.HoldSubdivisions
                        y=nonlinearBicycleModel.sample(x,u,cfg,[],s);
                        ey=nonlinearBicycleModel.error(y,lane,ref);
                        hold=max(hold,exp(s/T)*sqrt(localValue(ey,certificate,ref,options)/value));
                    end
                catch exception
                    if startsWith(string(exception.identifier),"collisionAvoidanceController:")
                        check.contraction(end+1)=NaN;check.holdFactor(end+1)=NaN;check.rows(end+1)=false;check.slip(:,end+1)=NaN;continue;
                    end
                    rethrow(exception);
                end
                plus=nonlinearBicycleModel.error(next,lane,ref);
                check.contraction(end+1)=localValue(plus,certificate,ref,options)/value;
                check.holdFactor(end+1)=hold;
                check.rows(end+1)=terminalSafeSet.stateRows(next,cfg) && terminalSafeSet.stateRows(middle,cfg) ...
                    && terminalSafeSet.handlingRows(x,u,cfg) && terminalSafeSet.handlingRows(next,u,cfg);
                slip=localSlip(cfg,x,u);check.slip(:,end+1)=abs(slip);
            end
        end
    end
end

function value=localValue(e,certificate,ref,options)
    % V(e): e'Pe, or e'P(theta)e with P interpolated bilinearly at the
    % (ray-averaged) normalized slips along the controller.
    if options.Lyapunov=="common",value=e.'*certificate.matrices(:,:,1)*e;return;end
    w=certificate.gain*e;
    if options.Enclosure=="secant",[nodes,weights]=localGauss(options.Quadrature);else,nodes=1;weights=1;end
    theta=zeros(2,1);
    for q=1:numel(nodes)
        x=[0;0;ref.state(3:6)]+[0;0;0;nodes(q)*e(3:5)];x(3)=x(3)+nodes(q)*e(2);
        u=localInput(ref,x,nodes(q)*w,options);
        [~,t]=localSlip(ref.cfg,x,u);theta=theta+weights(q)*t;
    end
    weightsCorner=localBilinear(theta-certificate.thetaTrim,certificate.thetaBox);
    p=zeros(5);for i=1:4,p=p+weightsCorner(i)*certificate.matrices(:,:,i);end
    value=e.'*p*e;
end

function weights=localBilinear(theta,thetaBox)
    a=(theta(1)-thetaBox(1,1))/max(1e-12,thetaBox(1,2)-thetaBox(1,1));
    b=(theta(2)-thetaBox(2,1))/max(1e-12,thetaBox(2,2)-thetaBox(2,1));
    a=min(1,max(0,a));b=min(1,max(0,b));
    weights=[(1-a)*(1-b),a*(1-b),(1-a)*b,a*b];
end

function [inner,outer]=localExtents(sets,box,options)
    % Extents of the certified set: common: sqrt(diag(S)) box. polyQuadratic:
    % inner = extents of the intersection of the vertex ellipsoids (the set
    % certified for every scheduling), outer = of their union.
    count=size(sets,3);
    if count==1,inner=sqrt(diag(sets)).*box;outer=inner;return;end
    outer=zeros(5,1);for i=1:count,outer=max(outer,sqrt(diag(sets(:,:,i))));end
    inner=zeros(5,1);
    for d=1:5
        e=sdpvar(5,1);constraints=[];
        for i=1:count,constraints=[constraints,e.'*(sets(:,:,i)\e)<=1];end %#ok<AGROW>
        optimize(constraints,-e(d),sdpsettings('solver','sedumi','verbose',0));inner(d)=value(e(d));
    end
    inner=inner.*box;outer=outer.*box;
    if options.Lyapunov=="common",inner=outer;end
end

function text=localExtentsText(sets,box,options)
    [inner,outer]=localExtents(sets,box,options);
    text=mat2str(inner.',3);
    if size(sets,3)>1,text=[text,' (union ',mat2str(outer.',3),')'];end
end

function text=localUnionText(outer,options)
    if options.Lyapunov=="common",text='';else,text=sprintf(' (union [%.3f %.4f %.3f %.3f %.4f])',outer);end
end

function localReport(options,varargin)
    if options.Verbose,fprintf(varargin{:});end
end
