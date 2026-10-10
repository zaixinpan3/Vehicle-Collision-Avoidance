% Prototype 3: nominal LMI first, then robust LMIs over enclosures along u = K e on the ellipsoid.
root='/home/zai/Downloads/ResearchProjects/collisionAvoidance';
addpath(fullfile(root,'controller'),fullfile(root,'config'));
addpath(genpath(fullfile(root,'solver','YALMIP')));addpath(genpath(fullfile(root,'solver','sedumi')));
for speed=[8 15]
for curvature=[0 .005]
cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed));
point=nonlinearBicycleModel.operatingPoint(cfg,curvature);
reference=struct('state',point.state,'input',point.input,'curvature',curvature);
curve=struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200);lane=struct('referenceCurve',curve);
h=cfg.controller.sampleTime;T=cfg.clf.convergenceTimeConstantSeconds;rho=exp(-2*h/T);
ubar=[cfg.clf.certificationSteeringRadians;cfg.clf.certificationBrakingRatio];
[A0,B0]=terminalSafeSet.jacobians(reference,cfg,zeros(5,1),zeros(2,1),h);
tire=modifiedFialaTire.parameters(cfg);q=tire.corneringStiffness(1)/(3*tire.frictionCoefficient(1)*cfg.vehicle.m*cfg.vehicle.gravity*cfg.vehicle.lr/cfg.vehicle.wheelbase);
for boxes=[[1.5;.2;1;.3;.15],[1;.15;.75;.15;.08],[.75;.1;.5;.1;.05]]
  beta=boxes;margin=1.25;m=5;Dm=diag(beta);Du=diag(ubar);At=Dm\A0*Dm;Bt=Dm\B0*Du;
  fprintf('\n=== speed %g curv %g box %s\n',speed,curvature,mat2str(beta',3));
  signs=(dec2bin(0:2^m-1)-'0')*2-1;
  stream=RandStream('mt19937ar','Seed',11);
  % nominal LMI (no uncertainty) in scaled coordinates, maximal fill
  [St,Yt,fill,status]=localLmi(At,Bt,zeros(5,7,0),0,rho,0);
  fprintf('nominal LMI %d fill %.3f extents %s\n',status,fill,mat2str(sqrt(diag(St))'.*beta',3));
  if status~=0,continue;end
  for iter=1:4
    Kt=Yt/St;K=Du*Kt/Dm;P=Dm\inv(St)/Dm;P=(P+P')/2;F=chol(P);
    % enclosure over the ellipsoid {V<=1} along u = K e (boundary-heavy)
    d=randn(stream,5,1600);unit=F\(d./vecnorm(d));E=unit.*sqrt([ones(1,1100),rand(stream,1,500)]);
    Uin=K*E;
    [Aj,Bj]=terminalSafeSet.jacobians(reference,cfg,E,Uin,h);
    X=zeros(35,size(E,2));
    for j=1:size(E,2),X(:,j)=reshape([Dm\(Aj(:,:,j)-A0)*Dm,Dm\(Bj(:,:,j)-B0)*Du],[],1);end
    [Uu,~,~]=svd(X,'econ');coef=Uu(:,1:m)'*X;w=margin*max(abs(coef),[],2);
    resid=X-Uu(:,1:m)*coef;eps=margin*max(arrayfun(@(j)norm(reshape(resid(:,j),5,7)),1:size(E,2)));
    Dk=zeros(5,7,m);for k=1:m,Dk(:,:,k)=w(k)*reshape(Uu(:,k),5,7);end
    % slips and tire stiffness factor along the samples
    slipF=abs(atan2(point.state(5)+E(4,:)+cfg.vehicle.lf*(point.state(6)+E(5,:)),point.state(4)+E(3,:))-(point.input(1)+Uin(1,:)));
    fprintf(' iter %d: max|Ke| %s, max front slip %.3f rad (stiffness factor >= %.2f), coef bounds %s, eps %.4f\n',iter,mat2str(max(abs(Uin),[],2)',3),max(slipF),1-2*q*max(slipF)+q^2*max(slipF)^2,mat2str(w',3),eps);
    % bound with the current (P,K) over this enclosure
    Ft=chol(Dm*P*Dm);worst=0;
    for v=1:2^m,AB=[At,Bt];for k=1:m,AB=AB+signs(v,k)*Dk(:,:,k);end;worst=max(worst,norm(Ft*(AB(:,1:5)+AB(:,6:7)*Kt)/Ft));end
    bound=worst+eps*norm(Ft)*norm([eye(5);Kt]/Ft);
    fprintf('   current (P,K): vertex %.4f + residual %.4f = %.4f vs sqrt(rho) %.4f\n',worst,bound-worst,bound,sqrt(rho));
    % robust LMI over this enclosure
    [St2,Yt2,fill,status]=localLmi(At,Bt,Dk,eps,rho,0);
    fprintf('   robust LMI %d fill %.3f extents %s\n',status,fill,mat2str(sqrt(diag(St2))'.*beta',3));
    if status~=0 || fill<1e-3
      % fall back: P only with K fixed
      [St2,Yt2,fill,status]=localLmi(At,Bt,Dk,eps,rho,Kt);
      fprintf('   P-only LMI (K fixed) %d fill %.3f extents %s\n',status,fill,mat2str(sqrt(diag(St2))'.*beta',3));
      if status~=0 || fill<1e-3,break;end
    end
    St=St2;Yt=Yt2;
  end
  Kt=Yt/St;K=Du*Kt/Dm;P=Dm\inv(St)/Dm;P=(P+P')/2;F=chol(P);
  fprintf(' final extents %s, K=%s\n',mat2str(sqrt(diag(inv(P)))',3),mat2str(K,3));
  % independent sampled verification
  stream2=RandStream('mt19937ar','Seed',99);d=randn(stream2,5,1500);unit=F\(d./vecnorm(d));
  Ev=unit.*sqrt([ones(1,750),rand(stream2,1,750)]);worstC=0;worstMu=0;bad=0;
  for j=1:size(Ev,2)
    e=Ev(:,j);u=point.input+K*e;[pos,hd]=laneGeometry.referencePose(50,e(1),curve);x=[pos;hd+point.state(3)+e(2);point.state(4:6)+e(3:5)];
    try
      nx=nonlinearBicycleModel.sample(x,u,cfg);ep=nonlinearBicycleModel.error(nx,lane,reference);
      worstC=max(worstC,(ep'*P*ep)/(e'*P*e));
      for s=h*(1:10)/10,y=nonlinearBicycleModel.sample(x,u,cfg,[],s);ey=nonlinearBicycleModel.error(y,lane,reference);worstMu=max(worstMu,exp(s/T)*sqrt((ey'*P*ey)/(e'*P*e)));end
    catch,bad=bad+1;end
  end
  fprintf(' sampled worst V+/V = %.5f (rho %.5f), hold factor %.4f, domain failures %d; stateLevel %.3f\n',worstC,rho,worstMu,bad,terminalSafeSet.stateLevel(struct('state',point.state,'input',point.input,'matrix',P),cfg));
end
end
end
function [S,Y,fill,status]=localLmi(At,Bt,Dk,eps,rho,Kfixed)
  m=size(Dk,3);signs=(dec2bin(0:2^m-1)-'0')*2-1;if m==0,signs=zeros(1,0);end
  S=sdpvar(5,5);t=sdpvar(1);lam=sdpvar(max(1,2^m),1);
  if isequal(Kfixed,0),Y=sdpvar(2,5,'full');else,Y=Kfixed*S;end
  C=[S>=t*eye(5),t>=0];
  for i=1:5,C=[C,S(i,i)<=1];end
  for j=1:2,C=[C,[1,Y(j,:);Y(j,:)',S]>=0];end
  Z=eps*[S;Y];
  for v=1:max(1,2^m)
    AB=[At,Bt];for k=1:m,AB=AB+signs(v,k)*Dk(:,:,k);end
    Nv=AB(:,1:5)*S+AB(:,6:7)*Y;
    if eps>0
      C=[C,[rho*S,Nv',Z';Nv,S-lam(v)*eye(5),zeros(5,7);Z,zeros(7,5),lam(v)*eye(7)]>=0,lam(v)>=0];
    else
      C=[C,[rho*S,Nv';Nv,S]>=0];
    end
  end
  dg=optimize(C,-t,sdpsettings('solver','sedumi','verbose',0));
  status=dg.problem;S=value(S);Y=value(Y);fill=value(t);
end
