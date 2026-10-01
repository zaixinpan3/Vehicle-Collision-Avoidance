% Geometry of the shifted-input rollout at the restart frames (set speed, name, captureTimes).
addpath('/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/methods');
h=0.05;record=replayCase(speed,name,round(max(captureTimes)/h)+1,'',captureTimes);cfg=record.configuration;
shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
for c=1:numel(record.capture)
    cap=record.capture(c);model=cap.model;model=rmfield(model,'linearization');model.terminal=cap.previousState.terminal;
    [anchor,source]=rtiInternals('initialization',model,cap.previousState);
    prev=cap.previousState;n=size(anchor.inputs,2);prefix=cfg.controller.horizonSteps;
    dAnchor=zeros(1,n+1);dPlan=NaN(1,n+1);
    for j=1:n+1
        q=predictiveSafetyGeometry.targetFlow(model.targetEpoch,(model.sampleIndex+j-1)*h);
        dAnchor(j)=predictiveSafetyGeometry.rectangle(anchor.states(1:3,j),shape,q(1:3),q(8:11));
        if j+1<=size(prev.stateTrajectory,2),dPlan(j)=predictiveSafetyGeometry.rectangle(prev.stateTrajectory(1:3,j+1),shape,q(1:3),q(8:11));end
    end
    seed=terminalContinuation.fit(model.terminal,[anchor.states(:,end);anchor.inputs(:,end)],model.sampleIndex+n);
    [value,~,deviation]=terminalContinuation.membership([anchor.states(:,end);anchor.inputs(:,end)],model.sampleIndex+n,seed);
    m=min(n,size(prev.stateTrajectory,2)-1);gap=vecnorm(prev.stateTrajectory(1:2,2:m+1)-anchor.states(1:2,1:m),2,1);
    [dmin,jmin]=min(dAnchor(prefix+2:end));
    fprintf('%s %d t=%.2f init=%s: shifted rollout tail (stages>%d) minimum distance %.4f m at stage %d (previous affine plan there %.4f m); whole rollout min %.4f m; terminal membership value %.4g (radius^2 %.3g); previous-plan vs rollout max position gap %.3f m\n', ...
        name,speed,cap.time,source,prefix,dmin,jmin+prefix,dPlan(jmin+prefix+1),min(dAnchor),value,model.terminal.radius^2,max(gap));
end
