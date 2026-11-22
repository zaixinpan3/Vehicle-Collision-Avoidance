function [command,predictedInput,prediction,controllerState] = ...
        collisionAvoidanceController(block,targetEstimate,reference,cfg,previousState)
%collisionAvoidanceController Compute a command from a complete raw sensor block.
% Pass [] as previousState to initialize. Omit it to use the retained state.
    persistent lastState
    callTimer=tic;
    if nargin<2,targetEstimate=[];end
    if nargin<3,reference=[];end
    if nargin<4,cfg=[];end
    if nargin<=3 && (ischar(block) || isstring(block))
        switch string(block)
            case "warmUp"
                solvePredictiveControl("outputFeedback",struct('operation',"warmUp", ...
                    'cfg',targetEstimate,'reference',reference));
            case "resetNominalTrajectory"
            otherwise
                error('collisionAvoidanceController:invalidAction','Use resetNominalTrajectory or warmUp.');
        end
        lastState=[];command=[];predictedInput=[];prediction=[];controllerState=[];return;
    end
    assert(isstruct(block) && isscalar(block) && isfield(block,'kind') && ...
        isequal(string(block.kind),"jointSensorBlock-v1"), ...
        'collisionAvoidanceController:invalidSensorBlock','Expected a jointSensorBlock-v1 raw sensor block.');
    assert(isempty(targetEstimate),'collisionAvoidanceController:independentEstimate', ...
        'The raw-sensor interface does not accept an independent target estimate.');
    if nargin<5,previousState=lastState;end
    request=struct('operation',"step",'block',block,'cfg',cfg,'reference',reference, ...
        'previousState',previousState,'callTimer',callTimer);
    result=solvePredictiveControl("outputFeedback",request);
    command=result.command;predictedInput=result.inputs;
    prediction=result.prediction;controllerState=result.state;lastState=controllerState;
end
