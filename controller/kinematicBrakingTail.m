function out = kinematicBrakingTail(action, cfg, varargin)
% kinematicBrakingTail The braking tail that closes the terminal set.
%
%   steps   = kinematicBrakingTail("steps", cfg)
%   profile = kinematicBrakingTail("profile", cfg, steps, speed)
%
% The terminal set of the two-stage program (TWO_STAGE_SAFETY.md, "The
% terminal set") is the set of terminal states from which an admissible
% KINEMATIC BRAKING TAIL reaches rest with every separation and road
% row satisfied along the way. The tail's longitudinal model is exact
% for piecewise-constant acceleration,
%
%   v_{k+1} = v_k + Ts a_k,   s_{k+1} = s_k + Ts v_k + 1/2 Ts^2 a_k,
%
% and its admissibility rows are a_k in
% [-terminal.backupDeceleration, 0], v_k >= 0, and REST at the last
% node: v = 0 with the last acceleration 0, so that the zero input the
% shifted candidate appends is itself admissible. Successive tail
% accelerations are independent decision variables. The optimizer
% supplies the tail witness; nothing here is ever executed. This file
% provides the two closed forms the program needs from that model:
%
%   "steps"    the stages needed to stop from the model's speed maximum
%              at the backup deceleration, plus two stages of rest.
%   "profile"  the fastest admissible stop from a given terminal speed,
%              padded with rest to `steps` stages. It supplies the
%              nominal tail of the schedule reference and cruise probe;
%              the optimization still decides its own tail.

    switch string(action)
        case "steps"
            out = localSteps(cfg);
        case "profile"
            out = localProfile(cfg, varargin{:});
        otherwise
            error("collisionAvoidanceController:invalidAction", ...
                "kinematicBrakingTail actions are steps and profile.");
    end
end

function steps = localSteps(cfg)
    sampleTime = cfg.controller.sampleTime;
    deceleration = cfg.terminal.backupDeceleration;
    stoppingStages = ceil(cfg.model.speedMaximum/(deceleration*sampleTime));
    steps = stoppingStages+2;
end

function profile = localProfile(cfg, steps, speed)
    if steps < 2 || steps ~= round(steps)
        error("collisionAvoidanceController:invalidFormulation", ...
            "The braking-tail length must be an integer of at least two.");
    end
    sampleTime = cfg.controller.sampleTime;
    deceleration = cfg.terminal.backupDeceleration;
    tolerance = 1.0e-9;
    remainingSpeed = max(speed, 0.0);
    profile = zeros(1, steps);
    for stageIdx = 1:steps-1
        if remainingSpeed <= tolerance
            break;
        end
        acceleration = -min(deceleration, remainingSpeed/sampleTime);
        profile(stageIdx) = acceleration;
        remainingSpeed = max(remainingSpeed+sampleTime*acceleration, 0.0);
    end
    if remainingSpeed > tolerance
        error("collisionAvoidanceController:invalidFormulation", ...
            "The braking tail cannot reach rest within %d stages " ...
            + "from %.3f m/s.", steps, speed);
    end
end
