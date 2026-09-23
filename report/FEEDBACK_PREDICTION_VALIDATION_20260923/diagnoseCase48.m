function diagnoseCase48(outputFolder)
%diagnoseCase48 Why case 48 (oncoming, curvature 0.01, uncertainty x10) fails with feedback.
    root = pwd;
    addpath(fullfile(root,"scripts"),fullfile(root,"controller"),fullfile(root,"config"),fullfile(root,"solver","clarabel","matlab"));
    egoBase = [.01;.01;.001;.01;.01;.001];targetBase = [.1;.1;.05;.05;.01;.01;.01;.01];
    if isappdata(0,"diagnosticFrame"),rmappdata(0,"diagnosticFrame");end
    try
        runExactStateRecursiveFeasibilityScenario(Scenario="oncoming",RoadCurvature=.01,SampleCount=240, ...
            DeadlineSeconds=Inf,SearchTimeLimitSeconds=30,OutputDirectory=fullfile(outputFolder,"case48"), ...
            EgoErrorBound=10*egoBase,TargetErrorBound=10*targetBase,TargetJerkAmplitude=10*[.02;.02]);
    catch exception
        fprintf("CASE48 harness %s\n",exception.identifier);
    end
    frame = getappdata(0,"diagnosticFrame");model = frame.model;cfg = frame.cfg;
    fprintf("CASE48 failing frame t=%.2f horizon %d\n",model.stateTime,model.horizonSteps);
    for enabled = [true,false]
        model.cfg.feedbackPrediction.enabled = enabled;cfg.feedbackPrediction.enabled = enabled;
        [program,~,~] = formulateAvoidanceProblem(model);
        [point,angles] = solveHardCbfClf.fluidInitialize(program,cfg);
        labels = program.physicalLabels;
        variants = {"full",false(size(labels));"no actuator amplitude",labels=="actuator"; ...
            "no slew",labels=="slew";"no actuator and slew",labels=="actuator" | labels=="slew"};
        support = program.prediction.feedbackInputSupport;
        fprintf("CASE48 feedback=%d max reserved steering %.4f rad braking %.4f; max slew reserve steering %.4f braking %.4f\n", ...
            enabled,max(support(1,:)),max(support(2,:)),max(program.prediction.feedbackSlewSupport(1,:)), ...
            max(program.prediction.feedbackSlewSupport(2,:)));
        for v = 1:size(variants,1)
            p = program;rows = find(variants{v,2});
            p.b(rows) = p.b(rows)+1e3;
            status = localSolve(p,point,angles,cfg);
            fprintf("CASE48   feedback=%d %-22s status %d\n",enabled,variants{v,1},status);
        end
    end
end

function status = localSolve(program,point,angles,cfg)
    conic = avoidanceStageQp.fixedDirections(program,point,angles,cfg);
    [reduced,~] = solveHardCbfClf.reduce(conic);
    center = zeros(numel(reduced.q),1);center(1:numel(reduced.anchorPlan)) = reduced.anchorPlan;
    linear = reduced.q+reduced.P*center;bound = reduced.b-reduced.A*center;
    scale = 1/max([1;abs(linear);abs(nonzeros(reduced.P))]);
    options = [cfg.solver.constraintTolerance,cfg.solver.optimalityTolerance,cfg.solver.maxIterations];
    [~,info] = solveAvoidanceSocpMex(scale*reduced.P,scale*linear,sparse(reduced.A),bound,reduced.cones,options);
    status = double(info.status);
end
