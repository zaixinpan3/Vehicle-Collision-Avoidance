function out = brakingSchedule(action, cfg, varargin)
%brakingSchedule Initial speed schedule and continuation length.
% This helper only constructs an admission anchor. Every optimized stage
% uses ltvBicycleStageMatrices with steering and acceleration decisions;
% there is no kinematic handoff or separate backup vehicle model.
%
% steps = brakingSchedule("steps", cfg)
% profile = brakingSchedule("profile", cfg, steps, speed)

    switch string(action)
        case "steps"
            out = localSteps(cfg);
        case "profile"
            out = localProfile(cfg, varargin{:});
        otherwise
            error("collisionAvoidanceController:invalidAction", ...
                "brakingSchedule actions are steps and profile.");
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
