function sequence = certifyFialaFeedbackSequence(inletLower,inletUpper,nominalState,nominalInputs,gain,measurementRadius,previousInput,cfg,options)
%certifyFialaFeedbackSequence Certify a prescribed finite sampled policy.
% Each reference center is the previous certified nominal endpoint. New
% measurement errors enter once per actual sample; the correlated endpoint
% and prior command memory are retained. This does not optimize avoidance.
    arguments
        inletLower (6,1) double {mustBeReal,mustBeFinite}
        inletUpper (6,1) double {mustBeReal,mustBeFinite}
        nominalState (6,1) double {mustBeReal,mustBeFinite}
        nominalInputs (2,:) double {mustBeReal,mustBeFinite}
        gain (2,6) double {mustBeReal,mustBeFinite}
        measurementRadius (6,1) double {mustBeReal,mustBeFinite,mustBeNonnegative}
        previousInput (2,1) double {mustBeReal,mustBeFinite}
        cfg (1,1) struct
        options.maximumCellDuration (1,1) double {mustBePositive,mustBeFinite} = .005
        options.maximumGenerators (1,1) double {mustBeInteger,mustBeFinite} = 64
        options.maximumComputationTimePerSample (1,1) double {mustBePositive} = Inf
    end
    count=size(nominalInputs,2);
    assert(count>0,'collisionAvoidanceController:emptyFeedbackSequence','A finite nonempty policy is required.');
    certificates=cell(1,count);inlet=[];completed=0;
    for index=1:count
        timer=tic;
        certificate=certifyFialaFeedbackSample(inletLower,inletUpper, ...
            [nominalState;nominalInputs(:,index)],gain,measurementRadius,previousInput,cfg, ...
            inlet=inlet,maximumCellDuration=options.maximumCellDuration, ...
            maximumGenerators=options.maximumGenerators, ...
            maximumComputationTime=options.maximumComputationTimePerSample);
        certificate.wallSeconds=toc(timer);
        certificates{index}=certificate;
        if ~certificate.accepted
            break
        end
        completed=index;
        inlet=certificate;
        nominalState=certificate.center(1:6);
    end
    sequence=struct('accepted',completed==count,'requestedSamples',count, ...
        'completedSamples',completed,'samples',{certificates(1:index)}, ...
        'scope',"Prescribed finite ego feedback policy; no collision, road, target exit or solver guarantee", ...
        'trajectoryCertified',false);
end
