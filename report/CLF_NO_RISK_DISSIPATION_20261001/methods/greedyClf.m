function result = greedyClf(speed,road,x0,u0,policy,frames)
%greedyClf Closed loop of an idealized one-step CLF policy, without RTI or anchors.
% Each hold, every input on a grid (then a local refinement) is propagated one
% hold with the controller's own nonlinear model (one RK4 step) and scored with
% the controller's CLF V = |F e|^2 and decay 1%. The plant uses the same model
% with a 5-ms RK4 mesh. Policies:
%   greedy            u = argmin V(x1)
%   greedySlew        greedy, with |du| <= (0.075,0.125) about the applied input
%   indifferent       among inputs with minimal slack rho = max(0,V1-0.99V0),
%                     the one closest to the applied input (models "any input
%                     meeting the 1% decrease is optimal")
%   indifferentSlew   indifferent, within the same per-hold change box
% No target and no road constraints: the no-collision-risk phase only.
    source='/home/zai/.cache/collisionAvoidance/margin-5cm-20261001/source';
    addpath(fullfile(source,'controller'),fullfile(source,'config'));
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed));
    plant=cfg;plant.nonlinear.integrationStep=0.005;
    ego=struct('position',x0(1:2),'yaw',x0(3),'speed',x0(4),'lateralVelocity',x0(5),'yawRate',x0(6));
    [~,lane]=readControllerInputs(ego,[],road,cfg);
    frame=predictiveSafetyGeometry.roadFrame(lane,[]);reference=nonlinearBicycleModel.cruise(cfg,frame(4));
    decay=1-cfg.nonlinear.clfDecay;tolerance=[.1;pi/180;.1;.05;.01];h=cfg.controller.sampleTime;
    slewBox=cfg.nonlinear.trustRadius*[.15;.25];
    x=x0;u=u0;n=frames;X=zeros(6,n+1);U=zeros(2,n);V=zeros(1,n+1);R=zeros(1,n);E=zeros(5,n+1);X(:,1)=x;
    e=nonlinearBicycleModel.error(x,lane,reference);E(:,1)=e;V(1)=norm(reference.factor*e)^2;
    inside=0;recovered=NaN;failure="";
    for k=1:n
        V0=V(k);slew=contains(policy,"Slew");
        if slew
            dGrid=u(1)+linspace(-slewBox(1),slewBox(1),31);bGrid=min(.999,max(-.999,u(2)+linspace(-slewBox(2),slewBox(2),21)));
        else
            dGrid=linspace(-0.7,0.7,71);bGrid=linspace(-.999,.999,21);
        end
        [cand,V1]=localScore(x,dGrid,bGrid,cfg,lane,reference);
        if isempty(cand),failure="no admissible input";break;end
        [~,i]=min(V1);
        % Local refinement around the best grid point (same box limits).
        dFine=cand(1,i)+linspace(-(dGrid(2)-dGrid(1)),dGrid(2)-dGrid(1),11);
        bFine=min(.999,max(-.999,cand(2,i)+linspace(-(bGrid(2)-bGrid(1)),bGrid(2)-bGrid(1),11)));
        if slew
            dFine=min(u(1)+slewBox(1),max(u(1)-slewBox(1),dFine));bFine=min(u(2)+slewBox(2),max(u(2)-slewBox(2),bFine));
        else
            dFine=min(0.7,max(-0.7,dFine));
        end
        [c2,V2]=localScore(x,dFine,bFine,cfg,lane,reference);cand=[cand,c2];V1=[V1,V2]; %#ok<AGROW>
        rho=max(0,V1-decay*V0);
        if startsWith(policy,"greedy")
            [~,i]=min(V1);
        else
            ok=find(rho<=min(rho)+1e-9*max(1,V0));
            [~,j]=min(((cand(1,ok)-u(1))/slewBox(1)).^2+((cand(2,ok)-u(2))/slewBox(2)).^2);i=ok(j);
        end
        u=cand(:,i);U(:,k)=u;R(k)=rho(i);
        try
            x=nonlinearBicycleModel.sample(x,u,plant);
        catch exception
            failure="plant: "+string(exception.identifier);break;
        end
        X(:,k+1)=x;e=nonlinearBicycleModel.error(x,lane,reference);E(:,k+1)=e;V(k+1)=norm(reference.factor*e)^2;
        if all(abs(e)<=tolerance),inside=inside+1;else,inside=0;end
        if inside*h>=5-1e-10,recovered=k*h;break;end
    end
    last=find(any(X~=0,1),1,'last');
    result=struct('speed',speed,'policy',policy,'x0',x0,'u0',u0,'steps',k,'recoveredAfter',recovered,'failure',failure, ...
        'X',X(:,1:last),'U',U(:,1:min(k,n)),'V',V(1:last),'E',E(:,1:last),'rho',R(1:min(k,n)), ...
        'maxAbsLateral',max(abs(E(1,1:last))),'positiveSlackSteps',nnz(R(1:min(k,n))>0));
end

function [cand,V1]=localScore(x,dGrid,bGrid,cfg,lane,reference)
    [D,B]=ndgrid(dGrid,bGrid);cand=[D(:).';B(:).'];V1=Inf(1,size(cand,2));
    for i=1:size(cand,2)
        try
            x1=nonlinearBicycleModel.sample(x,cand(:,i),cfg);
            V1(i)=norm(reference.factor*nonlinearBicycleModel.error(x1,lane,reference))^2;
        catch
        end
    end
    keep=isfinite(V1);cand=cand(:,keep);V1=V1(keep);
end
