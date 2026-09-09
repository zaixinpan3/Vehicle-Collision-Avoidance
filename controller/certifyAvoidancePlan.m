function check = certifyAvoidancePlan(qp, ~, model, decision)
% Acceptance never converts a negative physical margin into a certificate.
    check = struct("accepted", false, "failedConditions", "decision", ...
        "hardRowViolation", inf, "clfViolation", inf, "margin", -inf, ...
        "sweptClearanceMargin", -inf, "exitMargin", qp.exitMargin);
    if ~isnumeric(decision) || ~isreal(decision) || ~isvector(decision) ...
            || numel(decision) ~= qp.layout.decisionCount || any(~isfinite(decision))
        return;
    end
    decision = decision(:);
    if isfield(qp, "barrier") && isfield(qp.stageProgram, "fixedDecisionIndex") ...
            && ~isequal(decision(qp.stageProgram.fixedDecisionIndex), qp.stageProgram.fixedDecisionValue)
        check.failedConditions = "executedPrefix";
        return;
    end
    if isfield(qp,"fixedInput") && ~isequal(decision(1:2),qp.fixedInput)
        check.failedConditions = "committedInput";
        return;
    end
    operations = numel(decision)+2;
    gamma = operations*eps/(1-operations*eps);
    evaluationAllowance = gamma*(abs(qp.physicalBound)+abs(qp.inequalityMatrix)*abs(decision));
    margins = qp.physicalBound-qp.inequalityMatrix*decision-evaluationAllowance;
    check.hardRowViolation = max([0; -margins]);
    allowance = model.cfg.encounter.numericalMargin;
    check.sweptClearanceMargin = min([inf; margins(qp.safetyRows)])-allowance;
    check.margin = min([model.cfg.encounter.maximumCarriedMargin; ...
        check.sweptClearanceMargin; qp.exitMargin-allowance]);
    if isfield(qp, "barrier")
        % The same immutable reserve is charged at admission and every reuse.
        % Evaluate with the original matrices: shifting adds no rounding debt.
        evaluationAllowance = gamma*(abs(qp.barrier.baseBound)+abs(qp.inequalityMatrix)*abs(decision));
        certifiedMargins = qp.barrier.baseBound-qp.inequalityMatrix*decision-evaluationAllowance;
        selected = qp.barrier.scale > 0;
        check.margin = min([model.cfg.encounter.maximumCarriedMargin; ...
            certifiedMargins(selected)./qp.barrier.scale(selected)]);
        check.exitMargin = min(margins(qp.barrier.completionRows));
        check.hardRowViolation = max([check.hardRowViolation; -certifiedMargins]);
    end
    check.clfViolation = -inf;
    for index = 1:numel(qp.clf.constraints)
        constraint = qp.clf.constraints(index);
        value = constraint.map*decision+constraint.offset;
        residual = norm(constraint.root*value)^2+constraint.linear.'*value+constraint.constant;
        residual = residual+16*(numel(decision)+64)*eps*( ...
            norm(abs(constraint.root)*abs(value))^2+abs(constraint.linear).'*abs(value)+abs(constraint.constant));
        check.clfViolation = max(check.clfViolation, ...
            residual-decision(qp.layout.relaxationIndex(constraint.stage)));
    end
    conditions = [all(isfinite(margins)), check.hardRowViolation == 0, ...
        isfinite(check.clfViolation) && check.clfViolation <= 0, ...
        check.margin >= qp.requiredMargin, ~qp.certifiedInfeasible];
    names = ["finitePrediction", "hardRows", "sampledDataClf", "continuationMargin", "finiteProblem"];
    check.failedConditions = names(~conditions);
    check.accepted = all(conditions);
end
