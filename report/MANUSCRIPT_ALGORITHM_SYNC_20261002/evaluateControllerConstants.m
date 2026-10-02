function evaluateControllerConstants(sourceRoot,outputFile)
%evaluateControllerConstants Export controller constants quoted in the manuscript.
% sourceRoot must contain controller/, config/ and scripts/ of the evaluated
% commit (for example an export of HEAD made with git archive), so that
% uncommitted working-tree edits do not enter. No closed loop is simulated.
    addpath(fullfile(sourceRoot,'controller'),fullfile(sourceRoot,'config'),fullfile(sourceRoot,'scripts'));
    out=struct('matlabVersion',version);
    for speed=[8,15]
        cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed, ...
            'controller',struct('horizonSteps',8+8*(speed==15))));
        seed=terminalContinuation.build(cfg,0);
        entry=struct('terminalRadius',seed.radius,'contractionBound',seed.contractionBound, ...
            'defectBound',seed.defectBound,'construction',seed.construction, ...
            'horizonSteps',min(cfg.controller.maximumHorizonSteps,cfg.controller.horizonSteps ...
                +ceil(cfg.nonlinear.recoveryHorizonSeconds/cfg.controller.sampleTime)), ...
            'integrationStepsPerHold',nonlinearBicycleModel.meshCount(cfg), ...
            'policyEvaluationSteps',round(cfg.nominalClf.evaluationSeconds/cfg.controller.sampleTime));
        for curvature=[0,.005]
            reference=nonlinearBicycleModel.cruise(cfg,curvature);
            tail=nonlinearBicycleModel.nominalTail(cfg,curvature);
            c=struct('trimState',reference.state,'trimInput',reference.input, ...
                'tailSpectralRadius',tail.spectralRadius);
            if curvature==0,entry.straight=c;else,entry.curved=c;end
        end
        tire=modifiedFialaTire.parameters(cfg);
        entry.staticNormalLoad=tire.staticNormalLoad;
        entry.egoCircumradius=norm([cfg.vehicle.length;cfg.vehicle.width]/2)+norm(cfg.vehicle.rectangleOffset);
        entry.trustRadius=cfg.nonlinear.trustRadius;
        out.(sprintf('speed%d',speed))=entry;
    end
    fid=fopen(outputFile,'w');fwrite(fid,jsonencode(out,'PrettyPrint',true));fclose(fid);
end
