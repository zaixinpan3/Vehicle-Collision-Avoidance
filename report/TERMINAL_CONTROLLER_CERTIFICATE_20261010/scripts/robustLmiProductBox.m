% Prototype: robust discrete-time LMI over a Jacobian enclosure of the sampled bicycle.
root='/home/zai/Downloads/ResearchProjects/collisionAvoidance';
addpath(fullfile(root,'controller'),fullfile(root,'config'));
addpath(genpath(fullfile(root,'solver','YALMIP')));addpath(genpath(fullfile(root,'solver','sedumi')));
Xe=[zeros(1,5);eye(5)];
for speed=[8 15]
for curvature=[0 .005]
cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed));
point=nonlinearBicycleModel.operatingPoint(cfg,curvature);
curve=struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200);lane=struct('referenceCurve',curve);
reference=struct('state',point.state,'input',point.input,'curvature',curvature);
h=cfg.controller.sampleTime;T=cfg.clf.convergenceTimeConstantSeconds;rho=exp(-2*h/T);
ubar=[cfg.clf.certificationSteeringRadians;cfg.clf.certificationBrakingRatio];
[pos0,hd0]=laneGeometry.referencePose(50,0,curve);
normal=[-sin(hd0);cos(hd0)];Xe=[normal,zeros(2,4);zeros(4,1),eye(4)];
jac=@(e,du,s)localJacobian(e,du,s,point,cfg,curve,lane,reference,Xe);
[A0,B0]=jac(zeros(5,1),[0;0],h);
for boxes=[[1.5;.2;1;.3;.15],[1.5;.2;1;.2;.1],[1.5;.2;1;.15;.08],[1;.15;.75;.1;.06]]
  beta=boxes;margin=1.25;m=5;
  stream=RandStream('mt19937ar','Seed',11);N=1500;
  corners=(dec2bin(0:31)-'0')'*2-1;ic=(dec2bin(0:3)-'0')'*2-1;
  E=[];U=[];
  for k=1:size(corners,2),for l=1:size(ic,2),E=[E,corners(:,k).*beta];U=[U,ic(:,l).*ubar];end,end
  E=[E,(2*rand(stream,5,N)-1).*beta];U=[U,(2*rand(stream,2,N)-1).*ubar];
  n=size(E,2);X=zeros(35,n);
  for j=1:n,[Aj,Bj]=jac(E(:,j),U(:,j),h);X(:,j)=reshape([Aj-A0,Bj-B0],[],1);end
  [Uu,Ss,~]=svd(X,'econ');coef=Uu(:,1:m)'*X;w=margin*max(abs(coef),[],2);
  resid=X-Uu(:,1:m)*(Uu(:,1:m)'*X);eps=margin*max(arrayfun(@(j)norm(reshape(resid(:,j),5,7)),1:n));
  fprintf('\n=== speed %g curv %g box %s: singular values %s, eps %.4f\n',speed,curvature,mat2str(beta',3),mat2str(diag(Ss(1:min(7,end),1:min(7,end)))',3),eps);
  D=zeros(5,7,m);for k=1:m,D(:,:,k)=w(k)*reshape(Uu(:,k),5,7);end
  signs=(dec2bin(0:2^m-1)-'0')*2-1;
  S=sdpvar(5,5);Y=sdpvar(2,5,'full');t=sdpvar(1);lam=sdpvar(2^m,1);
  C=[S>=t*diag(beta.^2),t>=0];
  for i=1:5,C=[C,S(i,i)<=beta(i)^2];end
  for j=1:2,C=[C,[ubar(j)^2,Y(j,:);Y(j,:)',S]>=0];end
  Z=eps*[S;Y];
  for v=1:2^m
    AB=[A0,B0];for k=1:m,AB=AB+signs(v,k)*D(:,:,k);end
    Nv=AB(:,1:5)*S+AB(:,6:7)*Y;
    C=[C,[rho*S,Nv',Z';Nv,S-lam(v)*eye(5),zeros(5,7);Z,zeros(7,5),lam(v)*eye(7)]>=0,lam(v)>=0];
  end
  diag_=optimize(C,-t,sdpsettings('solver','sedumi','verbose',0));
  fprintf('LMI status %d (%s), fill t=%.3f\n',diag_.problem,diag_.info,value(t));
  if diag_.problem~=0,continue;end
  Sv=value(S);P=inv(Sv);P=(P+P')/2;K=value(Y)/Sv;F=chol(P);
  fprintf('extents sqrt(S_ii) = %s ; K =\n',mat2str(sqrt(diag(Sv))',3));disp(K);
  % post hoc robust contraction over vertices
  worst=0;for v=1:2^m,AB=[A0,B0];for k=1:m,AB=AB+signs(v,k)*D(:,:,k);end;worst=max(worst,norm(F*(AB(:,1:5)+AB(:,6:7)*K)/F));end
  fprintf('vertex max ||F(A+BK)F^-1|| = %.4f (+eps term %.4f) vs sqrt(rho) %.4f\n',worst,eps*norm(F)*norm([eye(5);K]/F),sqrt(rho));
  % independent sampled verification on the ellipsoid boundary and interior
  stream2=RandStream('mt19937ar','Seed',99);d=randn(stream2,5,2000);unit=F\(d./vecnorm(d));
  lev=[ones(1,1000),rand(stream2,1,1000)];Ev=unit.*sqrt(lev);worstC=0;worstMu=0;bad=0;
  for j=1:size(Ev,2)
    e=Ev(:,j);u=point.input+K*e;[pos,hd]=laneGeometry.referencePose(50,e(1),curve);x=[pos;hd+point.state(3)+e(2);point.state(4:6)+e(3:5)];
    try
      nx=nonlinearBicycleModel.sample(x,u,cfg);ep=nonlinearBicycleModel.error(nx,lane,reference);
      worstC=max(worstC,(ep'*P*ep)/(e'*P*e));
      for s=h*(1:10)/10,y=nonlinearBicycleModel.sample(x,u,cfg,[],s);ey=nonlinearBicycleModel.error(y,lane,reference);worstMu=max(worstMu,exp(s/T)*sqrt((ey'*P*ey)/(e'*P*e)));end
    catch,bad=bad+1;end
  end
  fprintf('sampled worst V+/V = %.5f (rho %.5f), sampled hold factor %.4f, domain failures %d, max|K e| = %s\n',worstC,rho,worstMu,bad,mat2str(max(abs(K*unit),[],2)',3));
  % slips at the box corners
  sl=[max(abs((beta(4)+cfg.vehicle.lf*beta(5))/(point.state(4)-beta(3)))),max(abs((beta(4)+cfg.vehicle.lr*beta(5))/(point.state(4)-beta(3))))];
  fprintf('slip bounds front/rear ~ %.3f %.3f rad\n',sl);
end
end
end
function [A,B]=localJacobian(e,du,s,point,cfg,curve,lane,reference,Xe)
  [pos,hd]=laneGeometry.referencePose(50,e(1),curve);x=[pos;hd+point.state(3)+e(2);point.state(4:6)+e(3:5)];
  u=point.input+du;
  [nx,a,b]=nonlinearBicycleModel.sample(x,u,cfg,[],s);[~,J]=nonlinearBicycleModel.errorLinearization(nx,lane,reference);
  A=J*a*Xe;B=J*b;
end
