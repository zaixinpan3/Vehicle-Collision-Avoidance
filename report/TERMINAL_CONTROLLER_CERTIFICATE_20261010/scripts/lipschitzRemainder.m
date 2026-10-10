root='/home/zai/Downloads/ResearchProjects/collisionAvoidance';
addpath(fullfile(root,'controller'),fullfile(root,'config'));
addpath(genpath(fullfile(root,'solver','YALMIP')));addpath(genpath(fullfile(root,'solver','sedumi')));
entries=jsondecode(fileread(fullfile(root,'config','clfMatrices.json')));
for speed=[8 15]
for curvature=[0 .005]
cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed));
key=nonlinearBicycleModel.clfKey(cfg,curvature);
entry=entries(arrayfun(@(e)string(e.key)==key,entries));
p=(entry.matrix+entry.matrix')/2;
point=nonlinearBicycleModel.operatingPoint(cfg,curvature);a=point.a;b=point.b;
s=inv(p);y=sdpvar(2,5,'full');closed=a*s+b*y;rho=entry.certifiedContraction*(1+1e-6);
bound=[cfg.clf.certificationSteeringRadians;cfg.clf.certificationBrakingRatio];
constraints=[[rho*s,closed';closed,s]>=0];
for j=1:2,constraints=[constraints,[bound(j)^2,y(j,:);y(j,:)',s]>=0];end
diagnostics=optimize(constraints,0,sdpsettings('solver','sedumi','verbose',0));
fprintf('LMI status %d\n',diagnostics.problem);
k=value(y)/s;
disp(k)
reference=struct('state',point.state,'input',point.input,'curvature',curvature,'matrix',p,'factor',chol(p),'contraction',exp(-2*cfg.controller.sampleTime/cfg.clf.convergenceTimeConstantSeconds),'gain',k,'sampledA',a,'sampledB',b);
budget=sqrt(reference.contraction)-sqrt(entry.certifiedContraction);
stream=RandStream('mt19937ar','Seed',1);d=randn(stream,5,2000);unit=reference.factor\(d./vecnorm(d));
for c=[1 .4 .1 .025]
 tic;r=terminalSafeSet.certificate(reference,cfg,sqrt(c)*unit,10);t=toc;
 fprintf('speed %g curv %g c=%.3f: L=%.5f (budget %.5f) worstContr=%.5f (rho %.5f, rho* %.5f) holdFactor=%.5f rows %d/%d nan %d time %.1fs\n',speed,curvature,c,max(r.remainder),budget,max(r.contraction),reference.contraction,entry.certifiedContraction,max(r.holdFactor),sum(r.rows),numel(r.rows),sum(isnan(r.remainder)),t);
end
% sampling convergence at c=1
for n=[200 2000 20000]
 d=randn(stream,5,n);u2=reference.factor\(d./vecnorm(d));r=terminalSafeSet.certificate(reference,cfg,u2,0);
 fprintf('  n=%d: L(1)=%.5f\n',n,max(r.remainder));
end
end
end
