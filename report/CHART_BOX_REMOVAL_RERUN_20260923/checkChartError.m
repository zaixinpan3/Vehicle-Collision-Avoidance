function checkChartError(outputFolder)
%checkChartError Chart error of the accepted plan versus the charged allowance (case 96).
    root = pwd;
    addpath(fullfile(root,"scripts"),fullfile(root,"controller"),fullfile(root,"config"));
    if isappdata(0,"frames"),rmappdata(0,"frames");end
    frames = {};
    for count = [1,2,5,10,20]
        if isappdata(0,"diagnosticFrame"),rmappdata(0,"diagnosticFrame");end
        try
            runExactStateRecursiveFeasibilityScenario(Scenario="stationary",RoadCurvature=.01,SampleCount=count, ...
                DeadlineSeconds=Inf,SearchTimeLimitSeconds=30,OutputDirectory=fullfile(outputFolder,"chart96"), ...
                InitialTrackingError=[0;.1;0;0;0]);
        catch
        end
        frames{end+1} = getappdata(0,"diagnosticFrame"); %#ok<AGROW>
    end
    for f = 1:numel(frames)
        model = frames{f}.model;cfg = frames{f}.cfg;cfg.solver.frameDeadlineSeconds = Inf;
        [program,~,~] = formulateAvoidanceProblem(model);
        [program,result] = solveHardCbfClf.fixedDirections(program,model,cfg);
        if ~result.feasible,fprintf("CHART t=%.2f infeasible\n",model.stateTime);continue;end
        plan = result.decision(program.layout.planIndex);
        prediction = program.prediction;worstExcess = -Inf;worstRatio = 0;
        for k = 1:numel(prediction.cells)
            tube = prediction.cells(k);state = tube.map*plan+tube.offset;
            frame = program.geometry.frames(k);
            excess = max(abs(state(1:3)-frame.domainCenter)-frame.domainRadius);
            exact = laneGeometry.fromFrenet(state,model.lane);
            affine = frame.positionOffset+frame.positionMap*state;
            ratio = norm(exact-affine)/max(frame.positionRemainder,eps);
            worstExcess = max(worstExcess,excess);worstRatio = max(worstRatio,ratio);
        end
        fprintf("CHART t=%.2f horizon %d | max box excess %.3f | max chart error / allowance %.3f\n", ...
            model.stateTime,prediction.stageCount,worstExcess,worstRatio);
    end
end
