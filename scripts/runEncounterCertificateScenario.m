function report = runEncounterCertificateScenario(options)
%runEncounterCertificateScenario Exact-state crossing with rolling prediction.
% Uses the common independently integrated scheduled-model experiment. The
% target remains observed after crossing; terminal feedback is prediction-only.
    arguments
        options.ForceSolverFailure (1,1) logical = false
        options.DeadlineSeconds (1,1) double {mustBePositive} = 0.1
    end
    report = runExactStateRecursiveFeasibilityScenario(Scenario="crossing", ...
        SampleCount=24,FailAfterAdmission=options.ForceSolverFailure,DeadlineSeconds=options.DeadlineSeconds);
end
