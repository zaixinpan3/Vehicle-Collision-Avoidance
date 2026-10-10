% Prototype 2: scaled coordinates, enclosure along u = K e, iterated.
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
for boxes=[[1.5;.2;1;.3;.15],[1;.15;.75;.15;.08],[.75;.1;.5;.1;.05]]
  beta=boxes;margin=1.25;m=5;Dm=diag(beta);Du=diag(ubar);
  stream=RandStream('mt19937ar','Seed',11);N=1500;
  corners=(dec2bin(0:31)-'0')'*2-1;ic=(dec2bin(0:3)-'0')'*2-1;
  E=[kron(corners,ones(1,4)).*beta,(2*rand(stream,5,N)-1).*beta];
  Ufull=[repmat(ic,1,32).*ubar,(2*rand(stream,2,N)-1).*ubar];
  fprintf('\n=== speed %g curv %g box %s\n',speed,curvature,mat2str(beta',3));
  K=[];Uin=Ufull;
  for iter=0:3
    [Aj,Bj]=terminalSafeSet.jacobians(reference,cfg,E,Uin,h);
    % scaled deviations
    X=zeros(35,size(E,2));
    for j=1:size(E,2),X(:,j)=reshape([Dm\(Aj(:,:,j)-A0)*Dm,Dm\(Bj(:,:,j)-B0)*Du],[],1);end
    [Uu,Ss,~]=svd(X,'econ');coef=Uu(:,1:m)'*X;w=margin*max(abs(coef),[],2);
    resid=X-Uu(:,1:m)*coef;eps=margin*max(arrayfun(@(j)norm(reshape(resid(:,j),5,7)),1:size(E,2)));
    Dk=zeros(5,7,m);for k=1:m,Dk(:,:,k)=w(k)*reshape(Uu(:,k),5,7);end
    At=Dm\A0*Dm;Bt=Dm\B0*Du;
    signs=(dec2bin(0:2^m-1)-'0')*2-1;
    S=sdpvar(5,5);Y=sdpvar(2,5,'full');t=sdpvar(1);lam=sdpvar(2^m,1);
    C=[S>=t*eye(5),t>=0];
    for i=1:5,C=[C,S(i,i)<=1];end
    for j=1:2,C=[C,[1,Y(j,:);Y(j,:)',S]>=0];end
    Z=eps*[S;Y];
    for v=1:2^m
      AB=[At,Bt];for k=1:m,AB=AB+signs(v,k)*Dk(:,:,k);end
      Nv=AB(:,1:5)*S+AB(:,6:7)*Y;
      C=[C,[rho*S,Nv',Z';Nv,S-lam(v)*eye(5),zeros(5,7);Z,zeros(7,5),lam(v)*eye(7)]>=0,lam(v)>=0];
    end
    dg=optimize(C,-t,sdpsettings('solver','sedumi','verbose',0));
    St=value(S);Yt=value(Y);
    fprintf('iter %d: coef bounds %s eps %.4f | LMI %d fill %.3f extents %s\n',iter,mat2str(w',3),eps,dg.problem,value(t),mat2str(sqrt(diag(St))'.*beta',3));
    if dg.problem~=0 || value(t)<1e-3,break;end
    Kt=Yt/St;K=Du*Kt/Dm;Pt=inv(St);P=Dm\Pt/Dm;P=(P+P')/2;F=chol(P);
    % enclosure for the next iteration along u = K e (clipped to the box)
    Uin=max(-ubar,min(ubar,K*E));
    % robust bound with this (P,K) over the current enclosure
    Ft=chol((Pt+Pt')/2);worst=0;
    for v=1:2^m,AB=[At,Bt];for k=1:m,AB=AB+signs(v,k)*Dk(:,:,k);end;worst=max(worst,norm(Ft*(AB(:,1:5)+AB(:,6:7)*Kt)/Ft));end
    fprintf('   vertex bound %.4f + eps term %.4f = %.4f vs sqrt(rho) %.4f; K=%s\n',worst,eps*norm(Ft)*norm([eye(5);Kt]/Ft),worst+eps*norm(Ft)*norm([eye(5);Kt]/Ft),sqrt(rho),mat2str(K,3));
  end
  if isempty(K),continue;end
  % sampled verification on the final ellipsoid (boundary + interior), with hold factor
  stream2=RandStream('mt19937ar','Seed',99);d=randn(stream2,5,1500);unit=F\(d./vecnorm(d));
  Ev=unit.*sqrt([ones(1,750),rand(stream2,1,750)]);worstC=0;worstMu=0;bad=0;maxu=[0;0];
  for j=1:size(Ev,2)
    e=Ev(:,j);u=point.input+K*e;maxu=max(maxu,abs(K*e));[pos,hd]=laneGeometry.referencePose(50,e(1),curve);x=[pos;hd+point.state(3)+e(2);point.state(4:6)+e(3:5)];
    try
      nx=nonlinearBicycleModel.sample(x,u,cfg);ep=nonlinearBicycleModel.error(nx,lane,reference);
      worstC=max(worstC,(ep'*P*ep)/(e'*P*e));
      for s=h*(1:10)/10,y=nonlinearBicycleModel.sample(x,u,cfg,[],s);ey=nonlinearBicycleModel.error(y,lane,reference);worstMu=max(worstMu,exp(s/T)*sqrt((ey'*P*ey)/(e'*P*e)));end
    catch,bad=bad+1;end
  end
  fprintf('   sampled worst V+/V = %.5f (rho %.5f), hold factor %.4f, domain failures %d, max|Ke| %s\n',worstC,rho,worstMu,bad,mat2str(maxu',3));
  fprintf('   stateLevel of this P: %.3f\n',terminalSafeSet.stateLevel(struct('state',point.state,'input',point.input,'matrix',P),cfg));
end
end
end
