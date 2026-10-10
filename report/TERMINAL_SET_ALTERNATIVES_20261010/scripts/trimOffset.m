cd('/home/zai/Downloads/ResearchProjects/collisionAvoidance');addpath('controller','config','scripts');
for speed=[8 15],for curvature=[0 .005]
cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed));
point=nonlinearBicycleModel.operatingPoint(cfg,curvature);ref=struct('state',point.state,'input',point.input,'curvature',curvature);
curve=struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200);lane=struct('referenceCurve',curve);
[position,yaw]=laneGeometry.referencePose(50,0,curve);x=[position;yaw+ref.state(3);ref.state(4:6)];
next=nonlinearBicycleModel.sample(x,ref.input,cfg,[],cfg.controller.sampleTime);
e=nonlinearBicycleModel.error(next,lane,ref);
fprintf('speed %g curvature %g: g(0,0) = %s (scaled norm %.2e)\n',speed,curvature,mat2str(e.',3),norm(e./[1;.15;.75;.15;.08]));
end,end
