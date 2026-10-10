root='/home/zai/Downloads/ResearchProjects/collisionAvoidance';
addpath(fullfile(root,'controller'),fullfile(root,'config'));
addpath(genpath(fullfile(root,'solver','YALMIP')));addpath(genpath(fullfile(root,'solver','sedumi')));
entries=jsondecode(fileread(fullfile(root,'config','clfMatrices.json')));
for speed=[8 15]
curvature=0;
cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed));
key=nonlinearBicycleModel.clfKey(cfg,curvature);
entry=entries(arrayfun(@(e)string(e.key)==key,entries));
p=(entry.matrix+entry.matrix')/2;
point=nonlinearBicycleModel.operatingPoint(cfg,curvature);a=point.a;b=point.b;
curve=struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200);lane=struct('referenceCurve',curve);
reference=struct('state',point.state,'input',point.input,'curvature',curvature,'matrix',p,'factor',chol(p));
Xe=[zeros(1,5);eye(5)];
% RK4 variational Jacobian at the trim vs expm
[pos,hd]=laneGeometry.referencePose(50,0,curve);x0=[pos;hd+point.state(3);point.state(4:6)];
[n0,a0,b0]=nonlinearBicycleModel.sample(x0,point.input,cfg);[~,J0]=nonlinearBicycleModel.errorLinearization(n0,lane,reference);
G0=J0*a0*Xe;H0=J0*b0;
fprintf('\n=== speed %g: trim RK4 Jacobian minus expm A (max abs %.3g), B (max abs %.3g)\n',speed,max(abs(G0-a),[],'all'),max(abs(H0-b),[],'all'));
disp(a);disp(G0-a);
scale=sqrt(diag(inv(p)));
for c=[.4 .1 .025]
  stream=RandStream('mt19937ar','Seed',3);N=3000;
  % points uniformly in the box |e_i|<=a_i sqrt(c) and on the ellipsoid
  box=(2*rand(stream,5,N)-1).*(scale*sqrt(c));
  d=randn(stream,5,N);ell=(reference.factor\(d./vecnorm(d)))*sqrt(c);
  E=[box,ell];Ge=zeros(5,5,size(E,2));Gu=zeros(5,2,size(E,2));
  for j=1:size(E,2)
    e=E(:,j);[pos,hd]=laneGeometry.referencePose(50,e(1),curve);x=[pos;hd+point.state(3)+e(2);point.state(4:6)+e(3:5)];
    du=[cfg.clf.certificationSteeringRadians;cfg.clf.certificationBrakingRatio]*sqrt(c).*(2*rand(stream,2,1)-1);
    u=point.input+du;
    [nx,ax,bx]=nonlinearBicycleModel.sample(x,u,cfg);[~,Jx]=nonlinearBicycleModel.errorLinearization(nx,lane,reference);
    Ge(:,:,j)=Jx*ax*Xe;Gu(:,:,j)=Jx*bx;
  end
  lo=min(Ge,[],3);hi=max(Ge,[],3);lou=min(Gu,[],3);hiu=max(Gu,[],3);
  fprintf('--- c=%.3f box half-widths |e|<= %s\n',c,mat2str((scale*sqrt(c))',3));
  fprintf('A entry half-width (hi-lo)/2:\n');disp((hi-lo)/2);
  fprintf('A entry half-width relative to |A0|+1e-3:\n');disp(((hi-lo)/2)./(abs(G0)+1e-3));
  fprintf('B half-width:\n');disp((hiu-lou)/2);disp(H0);
  % weighted: in P metric, F*(dA)*inv(F)
  F=reference.factor;W=zeros(5,5);
  for j=1:size(E,2),W=max(W,abs(F*(Ge(:,:,j)-G0)/F));end
  fprintf('max |F dA F^-1| entrywise:\n');disp(W);
  fprintf('max ||F dA F^-1||_2 over samples: %.4f ; sqrt(rho*) %.4f, sqrt(rho_req) %.4f\n', ...
    max(arrayfun(@(j)norm(F*(Ge(:,:,j)-G0)/F),1:size(E,2))),sqrt(entry.certifiedContraction),exp(-cfg.controller.sampleTime/4));
end
end
