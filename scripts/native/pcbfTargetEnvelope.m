function [boxes,centers] = pcbfTargetEnvelope(target,limits,times,domain)
%pcbfTargetEnvelope Enclose constant-parameter target motion over the prediction window.
    nominal=predictiveSafetyGeometry.predictTarget(target,times);
    [boxes,centers]=predictiveSafetyGeometry.targetMotionEnvelope(target,limits,times,nominal,domain);
end
