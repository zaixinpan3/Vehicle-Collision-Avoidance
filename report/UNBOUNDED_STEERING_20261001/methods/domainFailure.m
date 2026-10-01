cd('/home/zai/Downloads/ResearchProjects/collisionAvoidance');addpath('controller','config');
data=load('/home/zai/.cache/collisionAvoidance/eleven-failure-causes-20261001/early-problems.mat');c=data.cases{5};assert(c.scenario=="headOn" && abs(c.time-1)<1e-12);
m=c.capture.model;m.cfg.model=rmfield(m.cfg.model,{'frontWheelSteeringAngleMaximum','frontWheelSteeringRateMaximum'});
[solution,search]=solvePredictiveControl(m,c.prior,tic);u=solution.inputs(:,1);x=m.initialState;
slip=atan2([x(5)+m.cfg.vehicle.lf*x(6);x(5)-m.cfg.vehicle.lr*x(6)],x(4))-[u(1);0];
entry=struct('time',c.time,'input',u,'slipAngle',slip,'flags',[search.stages.exitFlag],'failure',"");
try,modifiedFialaTire.evaluate(slip,u(2),m.cfg);catch e,entry.failure=string(e.identifier);end
assert(entry.failure=="collisionAvoidanceController:invalidTireOperatingPoint");
f=fopen('/home/zai/.cache/collisionAvoidance/unbounded-steering-20261001/domain-failure.json','w');fprintf(f,'%s\n',jsonencode(entry));fclose(f);
fprintf('DOMAIN input=[%s] slip=[%s] flags=[%s] failure=%s\n',num2str(u.'),num2str(slip.'),num2str(entry.flags),entry.failure);
