function program = condensedAvoidanceTestOracle(qp)
% Independent physical-decision reference used only by transcription tests.
    constraints = qp.clf.constraints;
    matrices = cell(numel(constraints), 1);
    bounds = cell(numel(constraints), 1);
    for index = 1:numel(constraints)
        constraint = constraints(index);
        tMap = -constraint.linear.'*constraint.map;
        slackIndex = qp.layout.relaxationIndex(constraint.stage);
        tMap(slackIndex) = tMap(slackIndex)+1;
        tOffset = -constraint.linear.'*constraint.offset-constraint.constant;
        matrices{index} = -[tMap; 2*constraint.root*constraint.map; tMap];
        bounds{index} = [tOffset+1; 2*constraint.root*constraint.offset; tOffset-1];
    end
    program = struct("P", sparse(triu((qp.Hessian+qp.Hessian.')/2)), "q", qp.linear, ...
        "A", sparse([qp.inequalityMatrix; vertcat(matrices{:})]), ...
        "b", [qp.inequalityBound; vertcat(bounds{:})], ...
        "cones", [0; numel(qp.inequalityBound); 10*ones(numel(constraints), 1)], ...
        "physicalDecisionCount", qp.layout.decisionCount);
end
