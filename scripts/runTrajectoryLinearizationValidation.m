function summary = runTrajectoryLinearizationValidation(outputDirectory)
%runTrajectoryLinearizationValidation Check the one-pass nonlinear scenarios.
    arguments
        outputDirectory (1,1) string
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'scripts'));
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    drivers = {@runOncomingVehicleAvoidanceScenario, ...
        @runCircularCenterlineStraightTargetAvoidanceScenario, ...
        @runVaryingCurvatureStraightTargetAvoidanceScenario};
    names = ["straight","circular","sCurve"];
    rows = cell(0,13);
    for index = 1:numel(drivers)
        result = drivers{index}(Duration=18,ReferenceSpeed=10,TargetSpeed=10, ...
            RecoveryWindow=2,Plot=false,Report=false,Progress=true);
        save(fullfile(outputDirectory,names(index)+'.mat'),'result');
        valid = result.attempts.metadata(1:numel(result.command));
        trajectoryHolds = sum(cellfun(@(m)m.linearizationPolicy=="trajectory",valid));
        firstVisible = find(result.attempts.targetVisible,1);
        firstVisibleTime = NaN;
        if ~isempty(firstVisible),firstVisibleTime=result.attempts.time(firstVisible);end
        rows(end+1,:) = {names(index),18,result.controlTime(end),numel(result.command), ...
            trajectoryHolds,firstVisibleTime,result.failure.identifier,result.failure.time, ...
            result.avoidance.minimumControlledSeparatingAxisMargin, ...
            result.avoidance.minimumRoadBoundaryFunctionMargin,result.avoidance.passedTarget, ...
            result.avoidance.recoveredCruise,result.passed}; %#ok<AGROW>
        summary = cell2table(rows,VariableNames={'Scenario','RequestedDurationS', ...
            'ExecutedDurationS','ExecutedHolds','TrajectoryLinearizedHolds','FirstVisibleTimeS', ...
            'FailureIdentifier','FailureTimeS','MinimumPrefixSATMarginM','MinimumPrefixRoadMargin', ...
            'PassedTarget','RecoveredCruise','Passed'});
        writetable(summary,fullfile(outputDirectory,'nonlinear-summary.csv'));
        disp(summary(end,:));
    end
end
