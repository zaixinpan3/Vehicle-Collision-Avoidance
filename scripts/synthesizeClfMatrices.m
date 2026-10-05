function entries = synthesizeClfMatrices(operatingPoints,options)
%synthesizeClfMatrices Compute the CLF matrices offline, before experiments.
% The controller never computes P. For each operating point (controller
% configuration overrides and path curvature) this script builds the sampled
% local transverse model e+ = A e + B (u - u_ref) at the cruise trim and
% solves, by bisection on rho, the generalized eigenvalue problem
%
%   minimize rho  over P > 0 and a certificate gain K
%   subject to (A + B K)' P (A + B K) <= rho P,
%              |K_j e| <= ubar_j for every e with e' P e <= 1,
%              {e : sum((e./scales).^2) <= R^2} lies in {e : e' P e <= 1},
%              P >= Q / (R^2 * shapeRatio),
%
% with S = inv(P) and Y = K S as LMIs. ubar = [clf.certificationSteeringRadians;
% clf.certificationBrakingRatio], R is clf.certificationRegionScale and
% Q = diag(1./scales.^2). The
% result is the CLF with the fastest contraction attainable with admissible
% inputs near the path. K only certifies that contraction; it is discarded.
% The requested contraction rho_req = exp(-2 h / clf.convergenceTimeConstantSeconds)
% must be slower than the certified one, otherwise the requested time
% constant is not attainable and the operating point is rejected.
%
% Entries are merged into config/clfMatrices.json, keyed by
% nonlinearBicycleModel.clfKey. YALMIP and SeDuMi from solver/ are required
% here only.

    arguments
        operatingPoints struct = localDefaultPoints()
        options.OutputFile (1,1) string = ""
        options.BisectionSteps (1,1) double {mustBePositive,mustBeInteger} = 30
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    localPrepareSolver(root);
    file=options.OutputFile;if strlength(file)==0,file=fullfile(root,'config','clfMatrices.json');end
    existing=struct('key',{},'referenceSpeed',{},'curvature',{},'matrix',{},'certifiedContraction',{}, ...
        'requiredContraction',{},'certifiedTimeConstantSeconds',{},'requiredTimeConstantSeconds',{});
    if isfile(file),existing=jsondecode(fileread(file));existing=existing(:).';end
    entries=existing;
    for index=1:numel(operatingPoints)
        cfg=collisionAvoidanceControllerConfig(operatingPoints(index).configuration);
        curvature=operatingPoints(index).curvature;
        key=nonlinearBicycleModel.clfKey(cfg,curvature);
        entry=localSynthesize(cfg,curvature,key,options.BisectionSteps);
        match=find(arrayfun(@(e)string(e.key)==key,entries),1);
        if isempty(match),entries(end+1)=entry;else,entries(match)=entry;end %#ok<AGROW>
        fprintf('CLF speed %5.2f curvature %6.4f: certified time constant %.2f s, required %.2f s\n', ...
            cfg.referenceSpeed,curvature,entry.certifiedTimeConstantSeconds,entry.requiredTimeConstantSeconds);
    end
    [~,order]=sortrows([[entries.referenceSpeed].',[entries.curvature].']);entries=entries(order);
    fid=fopen(file,'w');assert(fid>=0,'Cannot write %s.',file);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(entries,'PrettyPrint',true));
end

function entry=localSynthesize(cfg,curvature,key,steps)
    point=nonlinearBicycleModel.operatingPoint(cfg,curvature);a=point.a;b=point.b;
    scales=[cfg.clf.lateralPositionErrorScale,cfg.clf.headingErrorScale,cfg.clf.speedErrorScale, ...
        cfg.clf.lateralVelocityErrorScale,cfg.clf.yawRateErrorScale];
    region=cfg.clf.certificationRegionScale;spread=cfg.clf.shapeRatio;
    bound=[cfg.clf.certificationSteeringRadians;cfg.clf.certificationBrakingRatio];
    h=cfg.controller.sampleTime;required=exp(-2*h/cfg.clf.convergenceTimeConstantSeconds);
    low=0;high=1;solution=[];
    for step=1:steps
        rho=(low+high)/2;[feasible,s,y]=localFeasible(a,b,rho,scales,region,spread,bound);
        if feasible,high=rho;solution={s,y};else,low=rho;end
    end
    if isempty(solution)
        error('synthesizeClfMatrices:infeasible','No CLF contracts with admissible inputs at speed %g, curvature %g.', ...
            cfg.referenceSpeed,curvature);
    end
    p=inv(solution{1});p=(p+p.')/2;k=solution{2}/solution{1};
    % Check the certificate itself, not only the solver status.
    residual=max(eig((a+b*k).'*p*(a+b*k)-high*p))/max(eig(p));
    assert(min(eig(p))>0 && residual<=1e-6,'synthesizeClfMatrices:certificate', ...
        'The certificate fails its own contraction check (relative residual %.3g).',residual);
    if high>required
        error('synthesizeClfMatrices:requestedConvergenceTooFast', ...
            ['clf.convergenceTimeConstantSeconds = %.3g s is faster than the certified %.3g s ' ...
            'at speed %g, curvature %g.'],cfg.clf.convergenceTimeConstantSeconds,-2*h/log(high), ...
            cfg.referenceSpeed,curvature);
    end
    entry=struct('key',key,'referenceSpeed',cfg.referenceSpeed,'curvature',curvature,'matrix',p, ...
        'certifiedContraction',high,'requiredContraction',required, ...
        'certifiedTimeConstantSeconds',-2*h/log(high),'requiredTimeConstantSeconds',cfg.clf.convergenceTimeConstantSeconds);
end

function [feasible,s,y]=localFeasible(a,b,rho,scales,region,spread,bound)
    n=size(a,1);m=size(b,2);d=diag(scales.^2);
    s=sdpvar(n,n);y=sdpvar(m,n,'full');closed=a*s+b*y;
    constraints=[[rho*s,closed.';closed,s]>=0, s>=region^2*d, s<=region^2*spread*d];
    for j=1:m,constraints=[constraints,[bound(j)^2,y(j,:);y(j,:).',s]>=0];end %#ok<AGROW>
    diagnostics=optimize(constraints,0,sdpsettings('solver','sedumi','verbose',0));
    feasible=diagnostics.problem==0;s=value(s);y=value(y);
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
    if ~isempty(which('optimize')) && ~isempty(which('sedumi')),return;end
    if ~isfolder(fullfile(root,'solver','YALMIP')) || ~isfolder(fullfile(root,'solver','sedumi'))
        error('synthesizeClfMatrices:missingLmiSolver','CLF synthesis requires YALMIP and SeDuMi in %s.',fullfile(root,'solver'));
    end
    addpath(genpath(fullfile(root,'solver','YALMIP')));addpath(genpath(fullfile(root,'solver','sedumi')));
end
