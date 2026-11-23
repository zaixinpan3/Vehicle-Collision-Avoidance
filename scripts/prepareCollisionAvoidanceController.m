function preparation = prepareCollisionAvoidanceController(block,road,cfg)
%prepareCollisionAvoidanceController Warm the raw-sensor controller on a discarded initial block.
    arguments
        block (1,1) struct
        road
        cfg
    end
    assert(isfield(block,'kind') && isequal(string(block.kind),"jointSensorBlock-v1"), ...
        'collisionAvoidanceController:invalidSensorBlock','Warm-up requires a jointSensorBlock-v1 initial block.');
    timer=tic;
    setupJointNlpSolver;
    addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))),'estimator'));
    onlineNrmmTrackingRuntime("prepareTarget",collisionAvoidanceControllerConfig(cfg));
    request=struct('operation',"prepareStep",'block',block,'cfg',cfg,'reference',road,'previousState',[]);
    result=solvePredictiveControl("outputFeedback",request);
    info=result.prediction.metadata;
    preparation=struct('performed',true,'elapsedSeconds',toc(timer), ...
        'optimizationReturned',info.optimizationReturned,'solver',info.solver, ...
        'discardedCommandCount',1,'nativeBuildRequired',false, ...
        'scope',"Raw-sensor controller warm-up with a discarded command");
    if isfield(info,'pcbfValue'),preparation.predictiveBarrierValue=info.pcbfValue;end
end
