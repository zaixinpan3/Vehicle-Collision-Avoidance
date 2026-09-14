function assessment = assessExactStateSafety(report)
%assessExactStateSafety Combine independent physical audits and witness checks.
% A controller's certification flag cannot override a failed truth audit.
% New admissions start an episode; value descent is checked on continuations.
    continuation = ~report.admissionFrame;
    tolerance = report.configuration.solver.lexicographicTieTolerance;
    arithmeticAllowance = 1e-9;
    assessment = struct();
    assessment.allCandidatesVerified = report.completed && all(report.candidateVerified(continuation));
    assessment.descentHolds = all(report.descentResidual(continuation)<=tolerance+1e-12);
    assessment.maximumDescentResidual = max([-inf,report.descentResidual(continuation)]);
    assessment.modelDomainHeld = report.minimumSampledModelDomainMargin>=0 ...
        && report.minimumSampledSpeed>=report.configuration.model.speedMinimum;
    assessment.truthContained = all(report.egoContainmentMargin>=-arithmeticAllowance) ...
        && all(report.targetContainmentMargin>=-arithmeticAllowance);
    assessment.containmentArithmeticAllowance = arithmeticAllowance;
    assessment.passed = report.completed && all(report.planCertified) ...
        && assessment.allCandidatesVerified && assessment.descentHolds ...
        && report.minimumSampledSeparationMargin>=0 && report.minimumSampledRoadMargin>=0 ...
        && report.maximumSlewViolation<=0 && assessment.modelDomainHeld && assessment.truthContained;
end
