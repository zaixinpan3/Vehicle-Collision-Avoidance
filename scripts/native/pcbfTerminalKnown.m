function [value,normal,gradient,exitSteps]=pcbfTerminalKnown(ego,target,radius,cfg)
%pcbfTerminalKnown Evaluate the terminal law for a single known target motion.
    cfg.collision.encounterRangeMeters=cfg.collision.encounterRangeMeters+radius;
    [value,normal,gradient,exitSteps]=terminalSafeSet.movingValue(ego,target,cfg);
end
