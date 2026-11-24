function [avoidance,connection]=pcbfMemberPreview(ego,target,cfg)
    [avoidance,connection]=solvePredictiveControl("outputFeedback",struct('operation',"memberPreview", ...
        'ego',ego,'target',target,'cfg',cfg));
end
