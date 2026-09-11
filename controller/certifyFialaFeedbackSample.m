function certificate = certifyFialaFeedbackSample(inletLower,inletUpper,nominal,gain,measurementRadius,previousInput,cfg,options)
%certifyFialaFeedbackSample Validate one complete sampled-and-held Fiala flow.
% The command is computed once at the sampling boundary. Internal time
% subdivision preserves the same command and shared uncertainty generators.
% This certifies nonlinear flow and command/slew bounds, not encounter safety.
% With inlet=anAcceptedCertificate, its correlated augmented endpoint replaces
% the box and previousInput arguments. maximumGenerators limits generators
% at the sample inlet; six local remainder generators are added per cell.
    arguments
        inletLower (6,1) double {mustBeReal,mustBeFinite}
        inletUpper (6,1) double {mustBeReal,mustBeFinite}
        nominal (8,1) double {mustBeReal,mustBeFinite}
        gain (2,6) double {mustBeReal,mustBeFinite}
        measurementRadius (6,1) double {mustBeReal,mustBeFinite,mustBeNonnegative}
        previousInput (2,1) double {mustBeReal,mustBeFinite}
        cfg (1,1) struct
        options.inlet = []
        options.maximumCellDuration (1,1) double {mustBePositive,mustBeFinite} = .005
        options.minimumCellDuration (1,1) double {mustBePositive,mustBeFinite} = 1e-5
        options.maximumGenerators (1,1) double {mustBeInteger,mustBeFinite} = 64
        options.maximumComputationTime (1,1) double {mustBePositive} = Inf
        options.maximumCells (1,1) double {mustBePositive,mustBeInteger,mustBeFinite} = 512
    end
    if exist('fialaFeedbackSampleMex','file')~=3
        error('collisionAvoidanceController:missingFialaVerifier', ...
            'Build and add the MPFR verifiers with buildFialaIntervalVerifier.');
    end
    cfg=collisionAvoidanceControllerConfig(cfg);
    parameters=fialaIntervalParameters(cfg);
    if ~isempty(options.inlet)
        assert(isstruct(options.inlet) && isscalar(options.inlet) && ...
            isfield(options.inlet,'accepted') && isequal(options.inlet.accepted,true), ...
            'collisionAvoidanceController:invalidFeedbackInlet','The previous sample must be completely certified.');
        assert(isfield(options.inlet,'modelParameters') && isequal(options.inlet.modelParameters,parameters), ...
            'collisionAvoidanceController:changedFeedbackModel','A correlated continuation requires the same true model.');
    end
    timing=[cfg.controller.sampleTime;options.maximumCellDuration;options.minimumCellDuration; ...
        options.maximumCells;options.maximumGenerators;min(options.maximumComputationTime,realmax)];
    execution=[-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum; ...
        cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum; ...
        cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
    certificate=fialaFeedbackSampleMex(inletLower,inletUpper,nominal,gain,measurementRadius, ...
        parameters,timing,previousInput,execution,options.inlet);
    certificate.modelParameters=parameters;
    certificate.feedbackNominal=nominal;
    certificate.feedbackGain=gain;
    certificate.measurementRadius=measurementRadius;
    certificate.scope="complete nonlinear held-feedback sample with input/slew checks; no encounter certificate";
    certificate.trajectoryCertified=false;
    certificate.inputEvaluations=1;
    certificate.stateOrder=["px";"py";"psi";"vx";"vy";"r";"delta";"beta"];
    certificate.arithmetic="directed MPFR flow, residual, exponential and generator enclosures";
end
