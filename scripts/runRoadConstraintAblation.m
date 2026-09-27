function result = runRoadConstraintAblation(outputDirectory, scenario)
%runRoadConstraintAblation Run the development controller without road rows.
% Retain the centerline reference, vehicle avoidance and dynamics constraints.
% Reconstruct road observations after execution for an independent offline
% boundary audit; these observations are never provided to the controller.
    arguments
        outputDirectory (1,1) string
        scenario (1,1) string {mustBeMember(scenario,["straight","circular","sCurve"])}
    end
    if ~isfolder(outputDirectory), mkdir(outputDirectory); end
    settings = struct('solver',struct('method',"nonlinearShooting"), ...
        'nonlinear',struct('horizonSeconds',5,'blockSteps',4, ...
        'maxIterations',12,'timeLimitSeconds',20,'feasibleRefinementIterations',4));
    common = {'Duration',18,'ReferenceSpeed',10,'TargetSpeed',10, ...
        'RecoveryWindow',2,'Plot',false,'Report',false,'Progress',true, ...
        'ControllerConfiguration',settings,'RoadBoundaries',false};
    switch scenario
        case "straight"
            result = runOncomingVehicleAvoidanceScenario(common{:});
        case "circular"
            result = runCircularCenterlineStraightTargetAvoidanceScenario(common{:});
        case "sCurve"
            result = runVaryingCurvatureStraightTargetAvoidanceScenario(common{:});
    end
    result.ablation = struct('roadConstraintsEnabled',false, ...
        'scope',"Offline road-free control with post-execution curb observations", ...
        'rightRoadBoundaryOffset',6,'leftRoadBoundaryOffset',8,'shoulderWidth',2.6, ...
        'boundaryFrames',{{}});
    for index = 1:numel(result.command)
        assert(result.attempts.metadata{index}.roadRowCount==0, ...
            'Road rows remain in the purported road-free controller.');
        geometry = result.attempts.roadPerception{index}.roadGeometry;
        assert(isempty(geometry.boundaries) && isempty(geometry.lateralClearance), ...
            'The controller still received road containment geometry.');
    end
    % Preserve the simulation result before optional postprocessing.
    save(fullfile(outputDirectory,scenario+".mat"),'result');
    frames = cell(numel(result.command),1);
    for index = 1:numel(frames)
        pose = result.controlState(index,1:3).';
        perceived = fitPerceivedRoadBoundaries(result.scenario.centerline,pose, ...
            PerceptionRange=30,RightOffset=6,LeftOffset=8,ShoulderWidth=2.6);
        frames{index} = perceived.roadGeometry.boundaries;
    end
    result.ablation.boundaryFrames = frames;
    save(fullfile(outputDirectory,scenario+".mat"),'result');
    fprintf('SAVED %s: failure=%d, executed %.2f s\n', ...
        scenario,result.failure.occurred,result.controlTime(end));
end
