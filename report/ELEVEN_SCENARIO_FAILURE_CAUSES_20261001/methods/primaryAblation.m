root='/home/zai/.cache/collisionAvoidance/eleven-failure-causes-20261001';addpath(root);
data=load(fullfile(root,'problems.mat'),'cases');cases=data.cases;results=struct([]);
for j=[1,3,8]
    c=cases{j};p=c.capture.problem;cfg=c.capture.model.cfg;
    for mode=["base","noTerminalCone","noTerminalGeometry","noCollisionRows","noTailCollisionRows", ...
            "noStateTrust","noInputTrust","noTrust","noClfCone"]
        trial=p;
        if mode=="noTerminalCone",trial.cones=trial.cones(2);end
        if mode=="noClfCone",trial.cones=trial.cones(1);end
        remove=false(size(p.b));
        if mode=="noTerminalGeometry",remove=p.rowGroups=="terminalGeometry";end
        if mode=="noCollisionRows",remove=startsWith(p.rowGroups,"collision");end
        if mode=="noTailCollisionRows",remove=startsWith(p.rowGroups,"collision") & p.rowStages>cfg.controller.horizonSteps;end
        trial.a(remove,:)=[];trial.b(remove)=[];
        if any(mode==["noStateTrust","noTrust"])
            ix=p.stateIndices;trial.lower(ix(:))=-Inf;trial.upper(ix(:))=Inf;
            low=[max(cfg.model.speedMinimum,cfg.model.scheduleSpeedFloor+1e-4);-cfg.model.lateralVelocityMaximum;-cfg.model.yawRateMaximum];
            high=[cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum];
            trial.lower(ix(4:6,2:end))=low-c.capture.anchor.states(4:6,2:end);
            trial.upper(ix(4:6,2:end))=high-c.capture.anchor.states(4:6,2:end);
        end
        if any(mode==["noInputTrust","noTrust"])
            iu=p.inputIndices;lo=[-cfg.model.frontWheelSteeringAngleMaximum;max(-1+1e-8,cfg.actuation.brakingRatioMinimum)];
            hi=[cfg.model.frontWheelSteeringAngleMaximum;min(1-1e-8,cfg.actuation.brakingRatioMaximum)];
            trial.lower(iu)=lo-c.capture.anchor.inputs;trial.upper(iu)=hi-c.capture.anchor.inputs;
        end
        [stats,z]=runProbe(trial,p.safetyObjective);
        entry=struct('speed',c.speed,'scenario',c.scenario,'mode',mode,'stats',stats);
        if isempty(results),results=entry;else,results(end+1)=entry;end
        fprintf('PRIMARY %g %s %s flag=%d objective=%.9g residual=%.9g\n',c.speed,c.scenario,mode,stats.flag,stats.safety,stats.rawResidual);
    end
end
file=fopen(fullfile(root,'primary-ablation.json'),'w');fprintf(file,'%s\n',jsonencode(results));fclose(file);
save(fullfile(root,'primary-ablation.mat'),'results');
