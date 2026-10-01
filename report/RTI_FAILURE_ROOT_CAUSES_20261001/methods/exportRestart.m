% Geometry of the shifted-input rollout at the three restart calls, exported to CSV.
addpath('/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/methods');
cases={{15,"curvedHeadOn",0.10},{15,"curvedCrossing",[19.80 21.60]}};T=table();h=0.05;
for c=1:numel(cases)
    speed=cases{c}{1};name=cases{c}{2};captureTimes=cases{c}{3};
    record=replayCase(speed,name,round(max(captureTimes)/h)+1,'',captureTimes);cfg=record.configuration;
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];prefix=cfg.controller.horizonSteps;
    for k=1:numel(record.capture)
        cap=record.capture(k);model=rmfield(cap.model,'linearization');model.terminal=cap.previousState.terminal;
        [anchor,source]=rtiInternals('initialization',model,cap.previousState);prev=cap.previousState;n=size(anchor.inputs,2);
        dA=zeros(1,n+1);
        for j=1:n+1
            q=predictiveSafetyGeometry.targetFlow(model.targetEpoch,(model.sampleIndex+j-1)*h);
            dA(j)=predictiveSafetyGeometry.rectangle(anchor.states(1:3,j),shape,q(1:3),q(8:11));
        end
        seed=terminalContinuation.fit(model.terminal,[anchor.states(:,end);anchor.inputs(:,end)],model.sampleIndex+n);
        member=terminalContinuation.membership([anchor.states(:,end);anchor.inputs(:,end)],model.sampleIndex+n,seed);
        m=min(n,size(prev.stateTrajectory,2)-1);gap=vecnorm(prev.stateTrajectory(1:2,2:m+1)-anchor.states(1:2,1:m),2,1);
        [tailMin,tailStage]=min(dA(prefix+2:end));
        f=record.frame(abs([record.frame.time]-cap.time)<1e-9);
        row=table(name,speed,cap.time,string(source),string(mat2str(f.stageFlags)),tailMin,tailStage+prefix,min(dA),member,model.terminal.radius^2,max(gap), ...
            'VariableNames',["scenario","speed","time","initialization","exitFlagsIncludingRestart","tailMinimumDistance","tailMinimumStage", ...
            "rolloutMinimumDistance","endpointCoreMembership","coreRadiusSquared","previousPlanToRolloutMaxPositionGap"]);
        T=[T;row]; %#ok<AGROW>
    end
end
writetable(T,'/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/bundle/restart-geometry.csv');disp(T);
