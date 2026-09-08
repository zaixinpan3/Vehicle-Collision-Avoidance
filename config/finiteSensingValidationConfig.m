function [cfg, evidence] = finiteSensingValidationConfig()
%finiteSensingValidationConfig Explicit experimental PassVeh14DOF allowance.
% The 2026-09-07 open-loop calibration used 10 m/s, steering 0.06*sin(pi*t),
% and road-load braking ratio plus 0.02*sin(t), for eight seconds. The rates
% initially exceeded twice its sampled residual maxima. A subsequent joint
% avoidance traces exposed residuals up to [1.798;3.506;2.662] in [vx;vy;r]
% derivative units. The dynamic rates were therefore identified as
% [2.5;5;4] before the next validation run. These traces are identification
% data and are not counted as independent validation of the new allowance.
% This is an empirical allowance, not a continuous-time bound proven over
% every avoidance state or an independently validated global envelope.
% The controller default retains zero residual for declared-model studies.
% The straight runtime profile uses 32 prediction stages (1.6 s at the
% default 0.05 s period) and 1e-4 objective optimality tolerance. Independent
% hard-constraint feasibility checks retain their original tolerances.
    cfg = struct("controller",struct("horizonSteps",32), ...
        "model",struct("plantModelResidualRateBound",[0.2;0.06;0.02;2.5;5;4], ...
            "frontWheelSteeringRateMaximum",0.5,"brakingRatioRateMaximum",2), ...
        "solver",struct("optimalityTolerance",1e-4));
    evidence = struct("scope","empiricalPlantResidualAllowance", ...
        "stateOrder",["station";"lateral";"headingError";"vx";"vy";"yawRate"], ...
        "continuousTimeBoundProven",false,"calibrationReferenceSpeed",10, ...
        "calibrationDuration",8,"calibrationDate","2026-09-07");
end
