function report = runEncounterCertificateScenario(options)
%runEncounterCertificateScenario Crossing with one trajectory solve per hold.
% The target stays observed. Any unsuccessful solve propagates an error.
    arguments
        options.ForceSolverFailure (1,1) logical = false
        options.DeadlineSeconds (1,1) double {mustBePositive} = 0.1
    end
    report = runExactStateRecursiveFeasibilityScenario(Scenario="crossing", ...
        SampleCount=24,FailAfterAdmission=options.ForceSolverFailure,DeadlineSeconds=options.DeadlineSeconds);
end
