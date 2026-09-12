function result = verifyFreeCompletionTime(outputDirectory)
%verifyFreeCompletionTime Verify rolling cruise and explicit solver-failure termination.
    arguments
        outputDirectory (1,1) string
    end
    result = struct();
    result.rolling = runExactStateRecursiveFeasibilityScenario(Scenario="crossing", ...
        SampleCount=24,OutputDirectory=fullfile(outputDirectory,"rolling"));
    result.failure = runExactStateRecursiveFeasibilityScenario(Scenario="crossing", ...
        SampleCount=24,FailAfterAdmission=true,OutputDirectory=fullfile(outputDirectory,"failure"));
    if ~isfolder(outputDirectory), mkdir(outputDirectory); end
    save(fullfile(outputDirectory,"completion-checks.mat"),"result");
    assert(result.rolling.passed && ~any(result.rolling.terminalActive));
    assert(result.rolling.executedHolds>result.rolling.admissionSteps);
    assert(~result.failure.completed && result.failure.executedHolds==1);
    assert(result.failure.failureIdentifier=="collisionAvoidanceController:noCertifiedContinuation");
    assert(~any(result.failure.retainedWitnessUsed));
end
