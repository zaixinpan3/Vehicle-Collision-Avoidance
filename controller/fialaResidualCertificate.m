function certificate = fialaResidualCertificate(lower,upper,a,b,c,cfg,feedback)
%fialaResidualCertificate Domain-wide Fiala-to-affine residual enclosure.
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
    tire=modifiedFialaTire.parameters(cfg);road=cfg.roadLoad;
    parameters=[cfg.vehicle.m;cfg.vehicle.Iz;cfg.vehicle.lf;cfg.vehicle.lr; ...
        tire.corneringStiffness;tire.longitudinalForceScale;road.airDensity; ...
        road.dragCoefficient;road.frontalArea;road.rollingCoefficient; ...
        road.rollingSpeedCoefficient;road.rollingQuarticCoefficient; ...
        road.rollingTransitionSpeed;cfg.vehicle.gravity;cfg.model.scheduleSpeedFloor];
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
