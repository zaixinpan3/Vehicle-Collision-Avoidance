cd('/home/zai/Downloads/ResearchProjects/collisionAvoidance');addpath('controller','config','scripts');
% Mean-value identity g(e,u) = Gbar [e;u] with Gauss-Legendre ray averages of n nodes, 8 m/s, curvature 0.005, default-box boundary samples.
cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8));curvature=.005;
point=nonlinearBicycleModel.operatingPoint(cfg,curvature);ref=struct('state',point.state,'input',point.input,'curvature',curvature);
box=[1;.15;.75;.15;.08];ubar=[.075;.125];h=.05;
stream=RandStream('mt19937ar','Seed',1);d=randn(stream,5,40);e=box.*(d./vecnorm(d));
k=[-0.05,-0.5,0,-0.1,-0.2;0,0,0.3,0,0];w=ubar.*max(-1,min(1,k*e./ubar));  % a plausible controller within the box
curve=struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200);lane=struct('referenceCurve',curve);
for n=[4 6 8 12 24]
  beta=.5./sqrt(1-(2*(1:n-1)).^(-2));t=diag(beta,1)+diag(beta,-1);[v,dd]=eig(t);[nodes,order]=sort(diag(dd));weights=2*v(1,order).^2;nodes=(nodes.'+1)/2;weights=weights/2;
  worst=0;worstAbs=0;
  for j=1:size(e,2)
    gbar=zeros(5,7);
    for q=1:n
      [a,b]=terminalSafeSet.jacobians(ref,cfg,nodes(q)*e(:,j),nodes(q)*w(:,j),h);gbar=gbar+weights(q)*[a,b];
    end
    [position,yaw]=laneGeometry.referencePose(50,e(1,j),curve);x=[position;yaw+ref.state(3)+e(2,j);ref.state(4:6)+e(3:5,j)];
    next=nonlinearBicycleModel.sample(x,ref.input+w(:,j),cfg,[],h);plus=nonlinearBicycleModel.error(next,lane,ref);
    predicted=gbar*[e(:,j);w(:,j)];
    worst=max(worst,norm((plus-predicted)./box)/norm(plus./box));worstAbs=max(worstAbs,norm((plus-predicted)./box));
  end
  fprintf('nodes %2d: worst relative mean-value residual %.2e, absolute (scaled) %.2e\n',n,worst,worstAbs);
end
