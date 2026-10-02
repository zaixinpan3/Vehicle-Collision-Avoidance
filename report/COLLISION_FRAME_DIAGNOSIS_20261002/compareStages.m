source='/home/zai/.cache/collisionAvoidance/unified-clf-implementation-20261001/assessment/source';
output='/home/zai/.cache/collisionAvoidance/collision-diagnosis-20261002';
cd(source);addpath('controller','config','scripts');
raw=jsondecode(fileread('/home/zai/.cache/collisionAvoidance/unified-clf-implementation-20261001/assessment/campaign/speed15/turningCrossing.json'));r=raw.results;
rows=[];
for i=[16,17,18,19,33,34]
    d=load(fullfile(output,sprintf('primary-%02d.mat',i)));e=load(fullfile(output,sprintf('frame-%02d.mat',i)));
    m=d.model;cfg=m.cfg;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    for stage=1:2
        if stage==1
            controls=d.anchor.inputs+reshape(d.point(d.problem.inputIndices),size(d.anchor.inputs));
            states=d.anchor.states+reshape(d.point(d.problem.stateIndices),size(d.anchor.states));
        else
            controls=e.pred.solution.inputs;states=e.pred.solution.states;
        end
        x=m.initialState;maxIntegrationError=0;
        peakForce=0;firstStepError=NaN;
        for j=1:round((1.65-(i-1)*.05)/.05)+1
            tangent=modifiedFialaTire.affineModel(d.anchor.states(:,j),d.anchor.inputs(:,j),cfg);
            fy=tangent.force+tangent.state*(states(:,j)-d.anchor.states(:,j))+tangent.input*(controls(:,j)-d.anchor.inputs(:,j));
            tire=modifiedFialaTire.parameters(cfg);peakForce=max(peakForce,max(hypot(fy./tire.longitudinalForceScale,controls(2,j))));
            if j==round((1.65-(i-1)*.05)/.05)+1
                q=predictiveSafetyGeometry.targetFlow(m.targetEpoch,1.65);
                ga=predictiveSafetyGeometry.rectangle(states(1:3,j),shape,q(1:3),q(8:11));
                gn=predictiveSafetyGeometry.rectangle(x(1:3),shape,q(1:3),q(8:11));
                poserr=norm(states(1:2,j)-x(1:2));break;
            end
            next=nonlinearBicycleModel.sample(x,controls(:,j),cfg);
            [~,ode]=ode45(@(~,s)nonlinearBicycleModel.derivative(s,controls(:,j),cfg),[0,.05],x,odeset('RelTol',1e-11,'AbsTol',1e-12));
            maxIntegrationError=max(maxIntegrationError,norm(next(1:2)-ode(end,1:2).'));
            if j==1,firstStepError=norm(states(1:2,2)-next(1:2));end
            x=next;
        end
        rows=[rows;(i-1)*.05,stage,ga,gn,poserr,peakForce,firstStepError,maxIntegrationError,controls(:,1).'];
    end
end
writematrix(rows,fullfile(output,'stages.csv'));disp(rows);
