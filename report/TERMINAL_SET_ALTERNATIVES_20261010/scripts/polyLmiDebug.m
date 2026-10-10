cd('/home/zai/Downloads/ResearchProjects/collisionAvoidance');addpath('controller','config','scripts');addpath(genpath('solver/YALMIP'));addpath(genpath('solver/sedumi'));
cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8));
point=nonlinearBicycleModel.operatingPoint(cfg,0);reference=struct('state',point.state,'input',point.input,'curvature',0);
h=.05;T=4;rho=exp(-2*h/T)*(1-1e-3);
box=[1;.15;.75;.15;.08];ubar=[.075;.125];
[a0,b0]=terminalSafeSet.jacobians(reference,cfg,zeros(5,1),zeros(2,1),h);
at=diag(box)\a0*diag(box);bt=diag(box)\b0*diag(ubar);
% (a) common form
s=sdpvar(5,5);y=sdpvar(2,5,'full');t=sdpvar(1);c=[s>=t*eye(5),t>=0];for i=1:5,c=[c,s(i,i)<=1];end
for j=1:2,c=[c,[1,y(j,:);y(j,:).',s]>=0];end
n=at*s+bt*y;c=[c,[rho*s,n.';n,s]>=0];
d=optimize(c,-t,sdpsettings('solver','sedumi','verbose',0));fprintf('common: %s fill %.4f\n',d.info,value(t));
% (b) poly form with one S (G-slack)
for variant=1:3
s=sdpvar(5,5);g=sdpvar(5,5,'full');r=sdpvar(2,5,'full');t=sdpvar(1);c=[s>=t*eye(5),t>=0];for i=1:5,c=[c,s(i,i)<=1];end
for j=1:2,c=[c,[1,r(j,:);r(j,:).',g+g.'-s]>=0];end
n=at*g+bt*r;c=[c,[rho*(g+g.'-s),n.';n,s]>=0];
if variant==2,c=[c,g==g.'];end
if variant==3,c=[c,norm(g,'fro')<=10];end
d=optimize(c,-t,sdpsettings('solver','sedumi','verbose',0));fprintf('poly variant %d: %s fill %.4f\n',variant,d.info,value(t));
end
% (c) poly form with 4 S's identical structure (as in the script), eps 0
s=cell(1,4);for i=1:4,s{i}=sdpvar(5,5);end;g=sdpvar(5,5,'full');r=sdpvar(2,5,'full');t=sdpvar(1);c=[t>=0];
for i=1:4,c=[c,s{i}>=t*eye(5)];for dd=1:5,c=[c,s{i}(dd,dd)<=1];end;for j=1:2,c=[c,[1,r(j,:);r(j,:).',g+g.'-s{i}]>=0];end;end
n=at*g+bt*r;x=rho*(g+g.'-s{1});
for j=1:4,c=[c,[x,n.';n,s{j}]>=0];end
d=optimize(c,-t,sdpsettings('solver','sedumi','verbose',0));fprintf('poly 4 sets: %s fill %.4f\n',d.info,value(t));
d=optimize(c,-t,sdpsettings('solver','sedumi','verbose',0,'sedumi.eps',1e-10));fprintf('poly 4 sets eps1e-10: %s fill %.4f\n',d.info,value(t));
