classdef fialaCertificate
    %fialaCertificate Validated nonlinear Fiala residuals and sampled feedback flows.

    methods (Static)
        function certificate = residual(lower,upper,a,b,c,cfg,feedback)
        %fialaCertificate.residual Domain-wide Fiala-to-affine residual enclosure.
        % Inputs are explicit world-state/input bounds [px;py;psi;vx;vy;r;delta;beta].
        % The native checker uses directed MPFR arithmetic and interval Jacobians.
        % This establishes a model inclusion; it does not certify a trajectory unless
        % its entire held-feedback state/input tube also lies inside this domain.
            arguments
                lower (8,1) double {mustBeReal,mustBeFinite}
                upper (8,1) double {mustBeReal,mustBeFinite}
                a (6,6) double {mustBeReal,mustBeFinite}
                b (6,2) double {mustBeReal,mustBeFinite}
                c (6,1) double {mustBeReal,mustBeFinite}
                cfg (1,1) struct
                feedback = []
            end
            if exist('fialaIntervalMex','file')~=3
                error('collisionAvoidanceController:missingFialaVerifier', ...
                    'Build and add the MPFR verifier with buildFialaIntervalVerifier before certification.');
            end
            parameters=fialaCertificate.parameters(cfg);
            if isempty(feedback)
                [residual,flow,jacobian]=fialaIntervalMex(lower,upper,[a,b],c,parameters);
                heldCell=[];
            else
                [residual,flow,jacobian,heldCell]=fialaIntervalMex(lower,upper,[a,b],c,parameters, ...
                    feedback.inletLower,feedback.inletUpper,feedback.nominal, ...
                    feedback.gain,feedback.measurementRadius,feedback.duration);
            end
            certificate=struct('lower',lower,'upper',upper,'residual',residual, ...
                'residualRateBound',max(abs(residual),[],2),'flow',flow,'jacobian',jacobian, ...
                'linearState',a,'linearInput',b,'offset',c, ...
                'scope',"domain-wide real Fiala equation inclusion with exact binary parameters", ...
                'arithmetic',"128-bit MPFR with directed outward binary64 endpoints", ...
                'heldCell',heldCell,'trajectoryCertified',false);
        end

        function certificate = sample(inletLower,inletUpper,nominal,gain,measurementRadius,previousInput,cfg,options)
        %fialaCertificate.sample Validate one complete sampled-and-held Fiala flow.
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
            parameters=fialaCertificate.parameters(cfg);
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

        function sequence = sequence(inletLower,inletUpper,nominalState,nominalInputs,gain,measurementRadius,previousInput,cfg,options)
        %fialaCertificate.sequence Certify a prescribed finite sampled policy.
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
                certificate=fialaCertificate.sample(inletLower,inletUpper, ...
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

        function parameters = parameters(cfg)
        %fialaCertificate.parameters Authoritative binary constants shared by verifiers.
            tire=modifiedFialaTire.parameters(cfg);road=cfg.roadLoad;
            parameters=[cfg.vehicle.m;cfg.vehicle.Iz;cfg.vehicle.lf;cfg.vehicle.lr; ...
                tire.corneringStiffness;tire.longitudinalForceScale;road.airDensity; ...
                road.dragCoefficient;road.frontalArea;road.rollingCoefficient; ...
                road.rollingSpeedCoefficient;road.rollingQuarticCoefficient; ...
                road.rollingTransitionSpeed;cfg.vehicle.gravity;cfg.model.scheduleSpeedFloor];
        end
    end
end
