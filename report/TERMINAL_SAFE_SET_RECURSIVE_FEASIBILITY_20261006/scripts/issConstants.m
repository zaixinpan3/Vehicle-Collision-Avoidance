function issConstants()
    cd('/home/zai/Downloads/ResearchProjects/collisionAvoidance');addpath('controller','config','scripts');
    out=struct([]);
    for speed=[8,15]
        for curvature=[0,.005]
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed,'controller',struct('horizonSteps',8+8*(speed==15))));
            reference=nonlinearBicycleModel.cruise(cfg,curvature);h=cfg.controller.sampleTime;
            transition=expm([reference.continuousA,reference.continuousB;zeros(2,7)]*h);
            b=transition(1:5,6:7);F=reference.factor;
            U=diag([cfg.clf.certificationSteeringRadians,cfg.clf.certificationBrakingRatio]);
            sensitivity=norm(F*b*U)*sqrt(2);
            level=min(cfg.terminal.levelMaximum,terminalSafeSet.stateLevel(reference,cfg));
            entry=struct('speed',speed,'curvature',curvature,'F',F,'sensitivity',sensitivity, ...
                'rho',reference.contraction,'cbar',level,'yawOffset',reference.state(3));
            out=[out,entry]; %#ok<AGROW>
            fprintf('v=%2d kappa=%.3f |F b U|sqrt2=%.4f  1/(1-sqrt(rho))=%.1f  => r_inf/eta=%.2f  sqrt(cbar)=%.3f  eta_max=%.4f\n', ...
                speed,curvature,sensitivity,1/(1-sqrt(reference.contraction)),sensitivity/(1-sqrt(reference.contraction)),sqrt(level), ...
                sqrt(level)*(1-sqrt(reference.contraction))/sensitivity);
        end
    end
    file=fopen('<scratchpad>/issConstants.json','w');
    fprintf(file,'%s',jsonencode(out));fclose(file);
end
