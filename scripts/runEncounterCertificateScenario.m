function report = runEncounterCertificateScenario(options)
%runEncounterCertificateScenario Exact-state crossing with indefinite control.
% Uses the common independently integrated scheduled-model experiment. The
% target remains observed after crossing, and the invariant tail is executed.
    arguments
        options.ForceSolverFailure (1,1) logical = true
    end
    report = runExactStateRecursiveFeasibilityScenario(Scenario="crossing", ...
        SampleCount=80,FailAfterAdmission=options.ForceSolverFailure);
end
