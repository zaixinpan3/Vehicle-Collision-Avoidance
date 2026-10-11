function entries = synthesizeClfMatrices(operatingPoints,options)
%synthesizeClfMatrices Certify the CLF and the terminal controller offline, before experiments.
% The controller never computes P. For each operating point (controller
% configuration overrides and path curvature) this script certifies, on the
% sampled nonlinear bicycle model itself, a quadratic CLF V = e'Pe of the
% transverse error e = [lateral; heading; vx - vx*; vy - vy*; r - r*] and the
% gain K of the terminal controller u = u* + K e
% (terminalSafeSet.terminalInput), such that on the sublevel set
% Omega = {V <= 1}
%
%   V(x+) <= rho V(x),   rho = exp(-2h/T), T = clf.convergenceTimeConstantSeconds,
%   |K_j e| <= ubar_j,   ubar = [clf.certificationSteeringRadians; clf.certificationBrakingRatio],
%   Omega lies inside the certification box |e_i| <= box_i
%                        (clf.certificationLateralMeters, ...HeadingRadians,
%                         ...SpeedMetersPerSecond, ...LateralVelocityMetersPerSecond,
%                         ...YawRateRadiansPerSecond).
%
% Method (TERMINAL_SAFE_SET.md, Section 2; NOMINAL_CLF.md). Let g(e, du) be the
% sampled error map of one hold from x(e) with input u* + du. The trim is a
% fixed point, g(0, 0) = 0, so along the ray from 0 to (e, du)
%
%   g(e, du) = Gbar(e, du) [e; du],   Gbar = integral_0^1 G(t e, t du) dt,
%
% with G = [dg/de, dg/ddu] the Jacobian of the sampled map. The certificate
% encloses the ray averages Gbar themselves (terminalSafeSet.rayJacobians,
% Gauss-Legendre quadrature; the quadrature's residual in the identity joins
% the enclosure's residual as a rank-one term), not the pointwise Jacobians:
% for the saturating tire the averages spread like the secants of its force
% curve, the pointwise Jacobians like its tangents, and the secant spread is
% several times smaller (report/TERMINAL_SET_ALTERNATIVES_20261010.tex). The
% enclosure is a zonotope in box-scaled coordinates,
%
%   G0 + sum_k delta_k D_k + R,   |delta_k| <= 1,   ||R|| <= eps,
%
% G0 the Jacobian at the trim, D_k the leading principal directions of the
% sampled deviations with their coefficient bounds, R the residual, all
% widened by MarginFactor. The discrete Lyapunov inequality is imposed at every
% vertex with the residual absorbed by Petersen's lemma (one multiplier per
% vertex), as an LMI in S = inv(P), Y = K S:
%
%   [rho S, N_v', Z'; N_v, S - lambda_v I, 0; Z, 0, lambda_v I] >= 0,
%   N_v = A_v S + B_v Y,  Z = eps [S; Y],
%   [ubar_j^2, Y_j; Y_j', S] >= 0,   S_ii <= box_i^2,   S >= t diag(box)^2,
%
% maximizing the fill t. The enclosure depends on the set and on the gain, so
% the design grows self-consistently: starting from the plain LMI inside
% StartFraction of the box, each iteration encloses the ray averages over
% Growth times the current set and over inputs within GainStep (in units of
% ubar) of the current controller, and solves the LMI with the new set inside
% that inflated set (S' <= Growth S) and the new gain within GainStep of the
% old one on it (a round with no design is retried with half and a quarter of
% the growth, then with a band two and four times as wide; the entry records
% the growth and band of the kept round). Every iteration's pair is therefore
% certified by the enclosure it was designed with; the pair with the largest
% fill is kept, re-verified by
% eigenvalues, and the smallest contraction it certifies is recorded. The same
% enclosures of the partial-hold maps give the hold factor, the largest
% exp(s/T) sqrt(V(x(s)) / V(x)) inside a hold, by which the CLF tube is
% inflated between samples. The one non-algebraic step is the enclosure: it is
% built from samples (boundary-heavy, with a fixed seed) and widened by
% MarginFactor; the entry records the samples, the slips they reach and an
% independent sampled check (terminalSafeSet.certificate).
%
% Entries are merged into config/clfMatrices.json, keyed by
% nonlinearBicycleModel.clfKey. With RefreshExisting (default true) every entry
% already in the file is synthesized again from the configuration its key
% records (entries whose key no longer reproduces are replaced). YALMIP and
% SeDuMi from solver/ are required here only.

    arguments
        operatingPoints struct = localDefaultPoints()
        options.OutputFile (1,1) string = ""
        options.RefreshExisting (1,1) logical = true
        options.EnclosureComponents (1,1) double {mustBePositive,mustBeInteger} = 6
        options.MarginFactor (1,1) double {mustBeGreaterThanOrEqual(options.MarginFactor,1)} = 1.25
        options.BoundarySamples (1,1) double {mustBePositive,mustBeInteger} = 1200
        options.InteriorSamples (1,1) double {mustBePositive,mustBeInteger} = 600
        options.Iterations (1,1) double {mustBePositive,mustBeInteger} = 10
        options.Growth (1,1) double {mustBeGreaterThan(options.Growth,1)} = 1.3
        options.GainStep (1,1) double {mustBeNonnegative} = .1
        options.StartFraction (1,1) double {mustBeInRange(options.StartFraction,0.05,1)} = .5
        options.QuadratureNodes (1,1) double {mustBePositive,mustBeInteger} = 8
        options.DesignMargin (1,1) double {mustBePositive} = 1e-3
        options.HoldSubdivisions (1,1) double {mustBePositive,mustBeInteger} = 10
        options.CertificateSeed (1,1) double {mustBeNonnegative,mustBeInteger} = 20261010
        options.Verbose (1,1) logical = true
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    localPrepareSolver(root);
    file=options.OutputFile;if strlength(file)==0,file=fullfile(root,'config','clfMatrices.json');end
    existing={};
    if isfile(file)
        decoded=jsondecode(fileread(file));
        if iscell(decoded),existing=decoded(:).';else,existing=num2cell(decoded(:).');end
    end
    entries=existing;
    jobs=struct('cfg',{},'curvature',{},'key',{});stale=strings(1,0);
    for index=1:numel(operatingPoints)
        cfg=collisionAvoidanceControllerConfig(operatingPoints(index).configuration);
        curvature=operatingPoints(index).curvature;
        jobs(end+1)=struct('cfg',cfg,'curvature',curvature,'key',nonlinearBicycleModel.clfKey(cfg,curvature)); %#ok<AGROW>
    end
    if options.RefreshExisting
        % Every stored entry is synthesized again from its key; a key whose
        % settings have been retired maps to the current key and is dropped.
        for index=1:numel(existing)
            old=string(existing{index}.key);
            [cfg,curvature]=localConfigurationFromKey(old);
            key=nonlinearBicycleModel.clfKey(cfg,curvature);
            if key~=old,stale(end+1)=old;end %#ok<AGROW>
            if any(arrayfun(@(j)j.key==key,jobs)),continue;end
            jobs(end+1)=struct('cfg',cfg,'curvature',curvature,'key',key); %#ok<AGROW>
        end
    end
    for index=1:numel(jobs)
        cfg=jobs(index).cfg;curvature=jobs(index).curvature;key=jobs(index).key;
        entry=localSynthesize(cfg,curvature,key,options);
        entries(cellfun(@(e)string(e.key)==key,entries))=[];
        entries{end+1}=entry; %#ok<AGROW>
        fprintf(['CLF speed %5.2f curvature %6.4f: extents [%.2f m %.3f rad %.2f m/s %.2f m/s %.3f rad/s], fill %.3f; ' ...
            'certified time constant %.2f s (required %.2f s), hold factor %.4f; sampled worst contraction %.4f\n'], ...
            cfg.referenceSpeed,curvature,entry.extents,entry.regionFill,entry.certifiedTimeConstantSeconds, ...
            entry.requiredTimeConstantSeconds,entry.holdFactor,entry.certificate.sampledWorstContraction);
    end
    entries(cellfun(@(e)any(string(e.key)==stale) || ~isfield(e,'gain'),entries))=[];
    [~,order]=sortrows([cellfun(@(e)e.referenceSpeed,entries).',cellfun(@(e)e.curvature,entries).']);entries=entries(order);
    fid=fopen(file,'w');assert(fid>=0,'Cannot write %s.',file);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(entries,'PrettyPrint',true));
    entries=[entries{:}];
end

function entry=localSynthesize(cfg,curvature,key,options)
    point=nonlinearBicycleModel.operatingPoint(cfg,curvature);
    reference=struct('state',point.state,'input',point.input,'curvature',curvature);
    h=cfg.controller.sampleTime;T=cfg.clf.convergenceTimeConstantSeconds;rho=exp(-2*h/T);
    % The LMIs ask for a contraction DesignMargin stricter than rho, so that
    % the eigenvalue re-verification at rho has a margin above solver tolerance.
    design=rho*(1-options.DesignMargin);
    box=[cfg.clf.certificationLateralMeters;cfg.clf.certificationHeadingRadians;cfg.clf.certificationSpeedMetersPerSecond; ...
        cfg.clf.certificationLateralVelocityMetersPerSecond;cfg.clf.certificationYawRateRadiansPerSecond];
    ubar=[cfg.clf.certificationSteeringRadians;cfg.clf.certificationBrakingRatio];
    low=max(-1+1e-8,cfg.actuation.brakingRatioMinimum);high=min(1-1e-8,cfg.actuation.brakingRatioMaximum);
    if point.input(2)-ubar(2)<low || point.input(2)+ubar(2)>high
        error('synthesizeClfMatrices:inputBox','The certification braking box leaves the actuator range at speed %g.',cfg.referenceSpeed);
    end
    scale=struct('state',diag(box),'input',diag(ubar));
    [a0,b0]=terminalSafeSet.jacobians(reference,cfg,zeros(5,1),zeros(2,1),h);
    at=scale.state\a0*scale.state;bt=scale.state\b0*scale.input;
    stream=RandStream('mt19937ar','Seed',options.CertificateSeed);
    % 1. The plain LMI inside StartFraction of the box gives the first gain and set.
    [s,y,fill,ok]=localLmi(at,bt,zeros(5,7,0),0,design,[],options.StartFraction*ones(5,1),[]);
    if ~ok || fill<=1e-6
        error('synthesizeClfMatrices:infeasible', ...
            'No CLF contracts with the requested time constant and input bound at speed %g, curvature %g.',cfg.referenceSpeed,curvature);
    end
    localReport(options,'  plain LMI: fill %.3f, extents %s\n',fill,mat2str((sqrt(diag(s)).*box).',3));
    % 2. Self-consistent growth: enclose over Growth times the current set and
    % the input band around the current gain, design inside both.
    candidates=struct('s',{},'k',{},'enclosure',{},'fill',{},'margin',{},'band',{},'growth',{});
    for iteration=1:options.Iterations
        k=y/s;ok2=false;fill2=0;
        % A round that admits no design at the nominal growth is retried with
        % half and a quarter of the growth, then with a gain band twice and
        % four times as wide (the enclosure is sampled again for each band).
        for band=options.GainStep*[1,2,4]
            gamma=options.Growth;
            for attempt=1:3
                enclosure=localEnclosure(reference,cfg,gamma*s,k,scale,a0,b0,h,options,stream,band);
                [s2,y2,fill2,ok2]=localLmi(at,bt,enclosure.directions,enclosure.residualNorm,design,gamma*s,ones(5,1),struct('k',k,'delta',band));
                if ok2 && fill2>1e-6,break;end
                localReport(options,'  iteration %d: growth %.3f, band %.2f infeasible (bounds %s, residual %.4f)\n', ...
                    iteration,gamma,band,mat2str(enclosure.coefficientBounds.',3),enclosure.residualNorm);
                gamma=1+(gamma-1)/2;
            end
            if ok2 && fill2>1e-6,break;end
        end
        if ~ok2 || fill2<=1e-6,break;end
        k2=y2/s2;margin=localVertexMargin(at,bt,enclosure,rho,k2,s2);
        localReport(options,'  iteration %d: growth %.3f, band %.2f, coefficient bounds %s, residual %.4f (quadrature %.1e); fill %.3f, margin %.1e, extents %s\n', ...
            iteration,gamma,band,mat2str(enclosure.coefficientBounds.',3),enclosure.residualNorm,enclosure.quadratureResidual, ...
            fill2,margin,mat2str((sqrt(diag(s2)).*box).',3));
        candidates(end+1)=struct('s',s2,'k',k2,'enclosure',enclosure,'fill',fill2,'margin',margin,'band',band,'growth',gamma); %#ok<AGROW>
        s=s2;y=y2;
    end
    candidates=candidates([candidates.margin]>=-1e-6);
    if isempty(candidates)
        error('synthesizeClfMatrices:infeasible', ...
            'No iteration of the robust LMI produced a verified certificate at speed %g, curvature %g.',cfg.referenceSpeed,curvature);
    end
    [~,order]=max([candidates.fill]);best=candidates(order);
    s2=best.s;k=best.k;enclosure=best.enclosure;fill=best.fill;
    % 3. Verify the vertex inequalities with the final (P, K) against the
    % enclosure they were designed with, and bisect the certified contraction.
    [minimumEigenvalue,certified]=localVerify(at,bt,enclosure,rho,k,s2);
    % 4. Hold factor from the enclosures of the partial-hold maps over the same samples.
    factorScaled=chol(inv(s2));holdFactor=1;residualHold=0;
    for step=1:options.HoldSubdivisions-1
        duration=h*step/options.HoldSubdivisions;
        [a0s,b0s]=terminalSafeSet.jacobians(reference,cfg,zeros(5,1),zeros(2,1),duration);
        partial=localEnclosure(reference,cfg,s2,k,scale,a0s,b0s,duration,options,stream,best.band,enclosure.errors,enclosure.inputs);
        bound=localVertexNorm(scale.state\a0s*scale.state,scale.state\b0s*scale.input,partial,k,factorScaled);
        holdFactor=max(holdFactor,exp(duration/T)*bound);residualHold=max(residualHold,partial.residualNorm);
    end
    holdFactor=max(holdFactor,exp(h/T)*localVertexNorm(at,bt,enclosure,k,factorScaled));
    % 5. Unscale and check independently on the final set.
    p=scale.state\inv(s2)/scale.state;p=(p+p.')/2;gain=scale.input*k/scale.state;
    full=struct('state',point.state,'input',point.input,'curvature',curvature,'matrix',p,'factor',chol(p), ...
        'gain',gain,'nominalA',a0,'nominalB',b0);
    check=RandStream('mt19937ar','Seed',options.CertificateSeed+1);
    directions=randn(check,5,800);unit=full.factor\(directions./vecnorm(directions));
    sampled=terminalSafeSet.certificate(full,cfg,unit.*sqrt([ones(1,400),rand(check,1,400)]),options.HoldSubdivisions);
    assert(all(isfinite(sampled.contraction)),'synthesizeClfMatrices:certificate','A sample of the certified set left the model domain.');
    entry=struct('key',key,'referenceSpeed',cfg.referenceSpeed,'curvature',curvature,'matrix',p,'gain',gain, ...
        'nominalA',a0,'nominalB',b0,'certifiedContraction',certified,'requiredContraction',rho, ...
        'certifiedTimeConstantSeconds',-2*h/log(certified),'requiredTimeConstantSeconds',T, ...
        'certifiedLevel',1,'holdFactor',holdFactor,'regionFill',fill,'extents',sqrt(diag(inv(p))));
    entry.certificate=struct('box',box,'inputBox',ubar,'components',size(enclosure.directions,3), ...
        'directions',enclosure.basis,'coefficientBounds',enclosure.coefficientBounds,'residualNorm',enclosure.residualNorm, ...
        'residualNormWithinHold',residualHold,'quadratureNodes',options.QuadratureNodes, ...
        'quadratureResidual',enclosure.quadratureResidual,'meanValueResidual',enclosure.meanValueResidual, ...
        'marginFactor',options.MarginFactor,'growth',best.growth,'inputBand',best.band, ...
        'iterationsUsed',numel(candidates),'boundarySamples',options.BoundarySamples,'interiorSamples',options.InteriorSamples, ...
        'seed',options.CertificateSeed,'iterations',options.Iterations,'holdSubdivisions',options.HoldSubdivisions, ...
        'lmiMinimumEigenvalue',minimumEigenvalue,'maximumInput',max(abs(enclosure.inputs),[],2), ...
        'maximumSlipRadians',enclosure.maximumSlip,'sampledWorstContraction',max(sampled.contraction), ...
        'sampledHoldFactor',max(sampled.holdFactor),'sampledRowsSatisfied',all(sampled.rows));
end

function enclosure=localEnclosure(reference,cfg,s,k,scale,a0,b0,duration,options,stream,band,errors,inputs)
    % Zonotope enclosure of the scaled deviations of the ray-averaged
    % Jacobians over the set {e' S^-1 e <= 1} (scaled coordinates) and over
    % inputs within band*ubar of u = K e: boundary-heavy samples, the
    % leading principal directions with their coefficient bounds, and the
    % residual norm (the principal residual plus the rank-one quadrature
    % correction of the mean-value identity), all widened by MarginFactor.
    if nargin<12
        lower=chol(s,'lower');
        d=randn(stream,5,options.BoundarySamples+options.InteriorSamples);
        radius=[ones(1,options.BoundarySamples),sqrt(rand(stream,1,options.InteriorSamples))];
        scaled=lower*(d./vecnorm(d)).*radius;
        errors=scale.state*scaled;
        inputs=scale.input*(k*scaled+band*(2*rand(stream,2,size(scaled,2))-1));
    end
    [a,b,gap]=terminalSafeSet.rayJacobians(reference,cfg,errors,inputs,duration,options.QuadratureNodes);
    assert(all(isfinite(a(:))) && all(isfinite(b(:))),'synthesizeClfMatrices:enclosure','A sample left the model domain.');
    n=size(errors,2);x=zeros(35,n);quadrature=0;meanValue=0;
    for j=1:n
        x(:,j)=reshape(localScaled(a(:,:,j),b(:,:,j),a0,b0,scale),[],1);
        scaledGap=scale.state\gap(:,j);
        quadrature=max(quadrature,norm(scaledGap)/norm([scale.state\errors(:,j);scale.input\inputs(:,j)]));
        plus=a(:,:,j)*errors(:,j)+b(:,:,j)*inputs(:,j)+gap(:,j);
        meanValue=max(meanValue,norm(scaledGap)/max(1e-12,norm(scale.state\plus)));
    end
    [u,~,~]=svd(x,'econ');m=min(options.EnclosureComponents,size(u,2));u=u(:,1:m);
    coefficients=u.'*x;bounds=options.MarginFactor*max(abs(coefficients),[],2);
    residual=x-u*coefficients;
    eps=options.MarginFactor*(max(arrayfun(@(j)norm(reshape(residual(:,j),5,7)),1:n))+quadrature);
    directions=zeros(5,7,m);for index=1:m,directions(:,:,index)=bounds(index)*reshape(u(:,index),5,7);end
    front=abs(atan2(reference.state(5)+errors(4,:)+cfg.vehicle.lf*(reference.state(6)+errors(5,:)), ...
        reference.state(4)+errors(3,:))-(reference.input(1)+inputs(1,:)));
    rear=abs(atan2(reference.state(5)+errors(4,:)-cfg.vehicle.lr*(reference.state(6)+errors(5,:)),reference.state(4)+errors(3,:)));
    enclosure=struct('directions',directions,'basis',u,'coefficientBounds',bounds,'residualNorm',eps, ...
        'quadratureResidual',quadrature,'meanValueResidual',meanValue, ...
        'errors',errors,'inputs',inputs,'maximumSlip',[max(front);max(rear)]);
end

function deviation=localScaled(a,b,a0,b0,scale)
    deviation=[scale.state\(a-a0)*scale.state,scale.state\(b-b0)*scale.input];
end

function [s,y,fill,ok]=localLmi(at,bt,directions,eps,rho,sContain,limit,previous)
    % maximize t  s.t.  S >= t I, S_ii <= limit_i^2, the input rows, the
    % containment S <= sContain, the gain band |(K - K_old)_j e| <= delta on
    % the set, and the vertex inequalities with Petersen's lemma for the
    % residual (scaled coordinates, where the box is the unit cube and the
    % input box the unit square).
    m=size(directions,3);signs=(dec2bin(0:2^m-1)-'0')*2-1;count=2^m;
    s=sdpvar(5,5);t=sdpvar(1);lambda=sdpvar(count,1);y=sdpvar(2,5,'full');
    constraints=[s>=t*eye(5),t>=0];
    for i=1:5,constraints=[constraints,s(i,i)<=limit(i)^2];end %#ok<AGROW>
    for j=1:2,constraints=[constraints,[1,y(j,:);y(j,:).',s]>=0];end %#ok<AGROW>
    if ~isempty(sContain),constraints=[constraints,s<=sContain];end
    if ~isempty(previous)
        for j=1:2
            band=y(j,:)-previous.k(j,:)*s;
            constraints=[constraints,[previous.delta^2,band;band.',s]>=0]; %#ok<AGROW>
        end
    end
    z=eps*[s;y];
    for v=1:count
        ab=[at,bt];for index=1:m,ab=ab+signs(v,index)*directions(:,:,index);end
        n=ab(:,1:5)*s+ab(:,6:7)*y;
        if eps>0
            constraints=[constraints,[rho*s,n.',z.';n,s-lambda(v)*eye(5),zeros(5,7);z,zeros(7,5),lambda(v)*eye(7)]>=0,lambda(v)>=0]; %#ok<AGROW>
        else
            constraints=[constraints,[rho*s,n.';n,s]>=0]; %#ok<AGROW>
        end
    end
    diagnostics=optimize(constraints,-t,sdpsettings('solver','sedumi','verbose',0));
    ok=diagnostics.problem==0;s=value(s);s=(s+s.')/2;y=value(y);fill=value(t);
end

function [minimumEigenvalue,certified]=localVerify(at,bt,enclosure,rho,k,s)
    % The vertex inequalities with the final (S, K): the least eigenvalue of
    % the Schur-reduced blocks at rho (relative to the block's norm, with the
    % best multiplier on a grid), and the smallest contraction they certify.
    feasible=@(r)localVertexMargin(at,bt,enclosure,r,k,s);
    minimumEigenvalue=feasible(rho);
    if minimumEigenvalue<-1e-6
        error('synthesizeClfMatrices:certificate','The final certificate fails its own vertex check (relative margin %.3g).',minimumEigenvalue);
    end
    low=0;high=rho;
    for step=1:24
        middle=(low+high)/2;
        if feasible(middle)>=-1e-6,high=middle;else,low=middle;end
    end
    certified=high;
end

function margin=localVertexMargin(at,bt,enclosure,rho,k,s)
    m=size(enclosure.directions,3);signs=(dec2bin(0:2^m-1)-'0')*2-1;
    z=enclosure.residualNorm*[s;k*s];grid=logspace(-6,0,60)*min(eig(s));margin=Inf;
    for v=1:2^m
        ab=[at,bt];for index=1:m,ab=ab+signs(v,index)*enclosure.directions(:,:,index);end
        n=(ab(:,1:5)+ab(:,6:7)*k)*s;best=-Inf;
        for lambda=grid
            block=[rho*s-(z.'*z)/lambda,n.';n,s-lambda*eye(5)];
            best=max(best,min(eig((block+block.')/2))/norm(block));
            if enclosure.residualNorm==0,break;end
        end
        margin=min(margin,best);
    end
end

function bound=localVertexNorm(at,bt,enclosure,k,factor)
    % max over the zonotope of ||F (A + B K) F^-1|| (convex, so at a vertex),
    % plus the residual's share.
    m=size(enclosure.directions,3);signs=(dec2bin(0:2^m-1)-'0')*2-1;bound=0;
    for v=1:2^m
        ab=[at,bt];for index=1:m,ab=ab+signs(v,index)*enclosure.directions(:,:,index);end
        bound=max(bound,norm(factor*(ab(:,1:5)+ab(:,6:7)*k)/factor));
    end
    bound=bound+enclosure.residualNorm*norm(factor)*norm([eye(5);k]/factor);
end

function localReport(options,varargin)
    if options.Verbose,fprintf(varargin{:});end
end

function [cfg,curvature]=localConfigurationFromKey(key)
    % The configuration a stored key records (nonlinearBicycleModel.clfKey);
    % settings that no longer exist are dropped.
    data=jsondecode(char(key));defaults=collisionAvoidanceControllerConfig();
    override=struct('vehicle',localShaped(data.vehicle,defaults.vehicle),'tire',localShaped(data.tire,defaults.tire), ...
        'roadLoad',localShaped(data.roadLoad,defaults.roadLoad),'referenceSpeed',data.referenceSpeed, ...
        'controller',struct('sampleTime',data.sampleTime),'nonlinear',struct('integrationStep',data.integrationStep), ...
        'clf',localShaped(data.clf,defaults.clf));
    cfg=collisionAvoidanceControllerConfig(override);curvature=data.curvature;
end

function group=localShaped(group,defaults)
    % jsondecode returns column vectors; restore the configuration's shapes and
    % drop settings the configuration no longer has.
    for name=string(fieldnames(group)).'
        if ~isfield(defaults,name)
            group=rmfield(group,name);
        elseif isnumeric(group.(name)) && numel(group.(name))==numel(defaults.(name))
            group.(name)=reshape(group.(name),size(defaults.(name)));
        end
    end
end

function points=localDefaultPoints()
    % The collision-threat campaign and the configuration default.
    points=struct('configuration',{},'curvature',{});
    for speed=[8,15]
        for curvature=[0,.005]
            points(end+1)=struct('configuration',struct('referenceSpeed',speed),'curvature',curvature); %#ok<AGROW>
        end
    end
end

function localPrepareSolver(root)
    if ~isempty(which('sdpvar')) && ~isempty(which('sedumi')),return;end
    if ~isfolder(fullfile(root,'solver','YALMIP')) || ~isfolder(fullfile(root,'solver','sedumi'))
        error('synthesizeClfMatrices:missingLmiSolver','CLF synthesis requires YALMIP and SeDuMi in %s.',fullfile(root,'solver'));
    end
    addpath(genpath(fullfile(root,'solver','YALMIP')));addpath(genpath(fullfile(root,'solver','sedumi')));
end
