source='/home/zai/.cache/collisionAvoidance/unified-clf-implementation-20261001/assessment/source';
output='/home/zai/.cache/collisionAvoidance/turning-root-cause-20261002';
cd(source);addpath('controller','config','scripts');
d=load('/home/zai/.cache/collisionAvoidance/collision-diagnosis-20261002/primary-18.mat');
m=d.model;cfg=m.cfg;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
q=predictiveSafetyGeometry.targetFlow(m.targetEpoch,1.65);old= predictiveSafetyGeometry.dualLinearization(d.anchor.states(1:3,17),shape,q(1:3),q(8:11),[0;1]);
comparisons=struct([]);
for k=[1,2,3]
    v=load(fullfile(output,sprintf('secondary-%02d.mat',k)));x=m.initialState;gap=Inf;where=NaN;trace=[];
    for j=1:size(v.solution.inputs,2)
        sol=ode45(@(~,s)nonlinearBicycleModel.derivative(s,v.solution.inputs(:,j),cfg),[0,.05],x,odeset('RelTol',1e-11,'AbsTol',1e-12));
        states=deval(sol,0:.005:.05);
        for l=1:11
            t=.85+(j-1)*.05+(l-1)*.005;qt=predictiveSafetyGeometry.targetFlow(m.targetEpoch,t);
            g=predictiveSafetyGeometry.rectangle(states(1:3,l),shape,qt(1:3),qt(8:11));
            if g<gap,gap=g;where=t;end
            trace=[trace;t,states(:,l).',qt(1:3).',g];
        end
        x=states(:,end);
    end
    item=v.entry;item.ode45MinimumGap=gap;item.ode45GapTime=where;
    at=find(abs(trace(:,1)-1.65)<1e-9,1);xx=trace(at,2:7).';
    dual=predictiveSafetyGeometry.dualLinearization(xx(1:3),shape,q(1:3),q(8:11),[0;1]);
    [oldGap,cornerGaps]=fixedMargin(xx,q,old.normal,shape);
    item.actualPoseAt165=xx(1:3);item.oldNormalGapAt165=oldGap;
    item.oldNormalCornerGapsAt165=cornerGaps;item.actualBestNormalAt165=dual.normal;item.actualGapAt165=trace(at,11);
    item.minimumSpeed=min(trace(:,5));item.maximumSpeed=max(trace(:,5));item.maximumLateralSpeed=max(abs(trace(:,6)));item.maximumYawRate=max(abs(trace(:,7)));
    if isempty(comparisons),comparisons=item;else,comparisons(end+1)=item;end
    writematrix(trace,fullfile(output,sprintf('counterfactual-%02d.csv',k)));
end
export=struct('shape',shape,'targetPose',q(1:3),'targetShape',q(8:11), ...
    'seedPose',d.anchor.states(1:3,17),'oldNormal',old.normal,'comparisons',comparisons);
fid=fopen(fullfile(output,'comparison.json'),'w');fprintf(fid,'%s\n',jsonencode(export));fclose(fid);
function [gap,values]=fixedMargin(x,q,n,shape)
    r=[cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];rt=[cos(q(3)),-sin(q(3));sin(q(3)),cos(q(3))];
    signs=[-1,1,1,-1;-1,-1,1,1];ego=x(1:2)+r*(shape(3:4)+shape(1:2).*signs);target=q(1:2)+rt*(q(10:11)+q(8:9).*signs);
    values=(n.'*ego-max(n.'*target)).';gap=min(values);
end
