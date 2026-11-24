function [clearance,gradient] = pcbfNodeClearance(ego,target,shape)
%pcbfNodeClearance Evaluate the production signed-distance dual at a prediction node.
    [clearance,gradient]=predictiveSafetyGeometry.rectangleClearanceNumeric(ego,target,shape);
end
