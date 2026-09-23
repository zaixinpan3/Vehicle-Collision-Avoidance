function checkUncertaintyCases(outputFolder)
%checkUncertaintyCases Terminal-set fit and target uncertainty size for uncertainty cases.
    root = pwd;
    addpath(fullfile(root,"scripts"),fullfile(root,"controller"),fullfile(root,"config"));
    egoBase = [.01;.01;.001;.01;.01;.001];targetBase = [.1;.1;.05;.05;.01;.01;.01;.01];
    specs = {"39 stationary k0 unc10","stationary",0,10;"42 stationary k0.01 unc10","stationary",.01,10; ...
        "53 crossing k0.01 unc3","crossing",.01,3;"54 crossing k0.01 unc10","crossing",.01,10; ...
        "crossing k0.01 unc1 (passes)","crossing",.01,1;"stationary k0 unc3 (passes)","stationary",0,3};
    for index = 1:size(specs,1)
        k = specs{index,4};
        if isappdata(0,"diagnosticFrame"),rmappdata(0,"diagnosticFrame");end
        try
            runExactStateRecursiveFeasibilityScenario(Scenario=specs{index,2},RoadCurvature=specs{index,3}, ...
                SampleCount=1,DeadlineSeconds=Inf,SearchTimeLimitSeconds=30, ...
                OutputDirectory=fullfile(outputFolder,sprintf("unc%02d",index)), ...
                EgoErrorBound=k*egoBase,TargetErrorBound=k*targetBase,TargetJerkAmplitude=k*[.02;.02]);
        catch
        end
        frame = getappdata(0,"diagnosticFrame");model = frame.model;
        [program,~,~] = formulateAvoidanceProblem(model);
        rho = program.prediction.initialErrorBound(:,end);
        [~,margin] = hardEncounterBarrier.terminalMembership(program.terminal,program.terminal.reference,rho);
        records = program.jointCertificate.records([program.jointCertificate.records.isExit]==false);
        spread = arrayfun(@(r) r.positionBall+sum(vecnorm(r.generators)),records);
        fprintf("UNC %-30s horizon %.2f s | terminal box lateral %.3f m heading %.4f rad | best-case terminal margin %.4f | target position uncertainty radius first %.2f m max %.2f m\n", ...
            specs{index,1},program.prediction.stageCount*model.sampleTime,rho(2),rho(3),min(margin), ...
            spread(1),max(spread));
    end
end
