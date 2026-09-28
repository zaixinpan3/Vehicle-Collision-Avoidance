function preparation = prepareCollisionAvoidanceController(ego, road, cfg, target)
%prepareCollisionAvoidanceController Warm the MATLAB model and optimizer.
% Probe commands are discarded; this does not change a running controller.
    arguments
        ego (1,1) struct
        road
        cfg
        target = []
    end
    timer=tic;cfg=collisionAvoidanceControllerConfig(cfg);
    [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
    preparation=struct('performed',true,'elapsedSeconds',toc(timer), ...
        'optimizationReturned',problem.metadata.optimizationReturned, ...
        'predictiveBarrierValue',problem.metadata.predictiveBarrierValue, ...
        'discardedCommandCount',1,'nativeBuildRequired',false, ...
        'scope',"Nominal MATLAB MPC warm-up; no command applied");
end
