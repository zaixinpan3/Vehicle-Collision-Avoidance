root='/home/zai/.cache/collisionAvoidance/eleven-failure-causes-20261001';source='/home/zai/.cache/collisionAvoidance/direct-two-stage-scenarios-20261001/source';addpath(root);cd(source);addpath('controller','config','scripts');data=load(fullfile(root,'early-problems.mat'));results=struct([]);
for j=1:numel(data.cases)
    c=data.cases{j};p=c.capture.problem;m=c.capture.model;cfg=m.cfg;anchor=c.capture.anchor;
    for mode=["base","zeroSlack","noFirstInputTrust","noInputTrust","rerollAnchor","coldSeed"]
        a=anchor;q=p;
        if mode=="zeroSlack",q.upper(q.slackIndices)=0;end
        if any(mode==["noInputTrust","noFirstInputTrust"])
            lo=[-cfg.model.frontWheelSteeringAngleMaximum;max(-1+1e-8,cfg.actuation.brakingRatioMinimum)];hi=[cfg.model.frontWheelSteeringAngleMaximum;min(1-1e-8,cfg.actuation.brakingRatioMaximum)];
            inds=1:size(a.inputs,2);if mode=="noFirstInputTrust",inds=1;end
            q.lower(q.inputIndices(:,inds))=lo-a.inputs(:,inds);q.upper(q.inputIndices(:,inds))=hi-a.inputs(:,inds);
        end
        valid=true;
        if mode=="rerollAnchor"
            try
                a.states(:,1)=m.initialState;for k=1:size(a.inputs,2),a.states(:,k+1)=nonlinearBicycleModel.sample(a.states(:,k),a.inputs(:,k),cfg);end
                q=reformulate(a,m);
            catch,valid=false;end
        end
        if mode=="coldSeed"
            try
                captured=[];captureController(c.ego,c.target,c.road,cfg,[]);
            catch e,valid=strcmp(e.identifier,'diagnostic:captured');end
            if valid,a=captured.anchor;q=captured.problem;mCold=captured.model;end
        end
        entry=struct('speed',c.speed,'scenario',c.scenario,'time',c.time,'mode',mode,'valid',valid, ...
            'primary',[],'secondary',[],'input',[],'v0',p.initialClfValue,'affineV',NaN,'actualV',NaN, ...
            'anchorInput',a.inputs(:,1),'affineNextState',[],'actualNextState',[],'clearance',[], ...
            'firstSlack',NaN,'maxInputChange',NaN);
        if valid
            [stats,z]=runProbe(q,q.safetyObjective);entry.primary=stats;
            if ~isempty(z)
                q.a=[q.a;q.safetyObjective.'];q.b=[q.b;sum(max(0,z(q.slackIndices)))+1e-6];o=zeros(size(q.lower));o(q.clfIndex)=1;[s,z2]=runProbe(q,o);entry.secondary=s;
                if ~isempty(z2)
                    u=a.inputs(:,1)+z2(q.inputIndices(:,1));entry.input=u;entry.firstSlack=z2(q.slackIndices(1));entry.maxInputChange=norm(u-c.trace.input,inf);
                    entry.affineV=norm(q.clfMap*z2+q.clfOffset)^2*q.clfScale;
                    entry.affineNextState=a.states(:,2)+z2(q.stateIndices(:,2));
                    [t,x]=ode45(@(~,x)nonlinearBicycleModel.derivative(x,u,cfg),linspace(0,.05,31),m.initialState,odeset('RelTol',1e-11,'AbsTol',1e-12));
                    entry.actualNextState=x(end,:).';err=nonlinearBicycleModel.error(x(end,:).',m.lane,m.nominalReference);entry.actualV=norm(m.nominalReference.factor*err)^2;
                    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];gap=zeros(numel(t),1);
                    for k=1:numel(t),target=predictiveSafetyGeometry.targetFlow(m.targetEpoch,m.sampleIndex*.05+t(k));gap(k)=predictiveSafetyGeometry.rectangle(x(k,1:3).',shape,target(1:3),target(8:11));end
                    target=predictiveSafetyGeometry.targetFlow(m.targetEpoch,m.sampleIndex*.05+.05);
                    affineGap=predictiveSafetyGeometry.rectangle(entry.affineNextState(1:3),shape,target(1:3),target(8:11));
                    entry.clearance=struct('initial',gap(1),'midpoint',gap(16),'end',gap(end),'minimum',min(gap),'affineEnd',affineGap);
                end
            end
        end
        results=[results,entry];fprintf('EARLY %g %s t=%.2f %s valid=%d',c.speed,c.scenario,c.time,mode,valid);
        if ~isempty(entry.primary),fprintf(' flag=%d slack=%.7g',entry.primary.flag,entry.primary.safety);end
        if ~isempty(entry.input),fprintf(' u=[%.5g %.5g] predDV=%.5g realDV=%.5g gap=%.6g',entry.input(1),entry.input(2),entry.affineV-entry.v0,entry.actualV-entry.v0,entry.clearance.minimum);end
        fprintf('\n');
    end
end
file=fopen(fullfile(root,'early-probe.json'),'w');fprintf(file,'%s\n',jsonencode(results));fclose(file);
