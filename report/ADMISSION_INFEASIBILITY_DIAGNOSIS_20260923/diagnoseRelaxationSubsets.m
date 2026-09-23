function results = diagnoseRelaxationSubsets(outputFolder)
%diagnoseInfeasibleAdmission Constraint-relaxation study of infeasible admission frames.
% Offline diagnostic only. Each failing sweep case is replayed with the
% declared-plant harness; the failing frame's model is rebuilt with the
% controller's own formulation and re-solved with constraint groups relaxed.
    arguments
        outputFolder (1,1) string
    end
    root = pwd;
    addpath(fullfile(root,"scripts"),fullfile(root,"controller"),fullfile(root,"config"), ...
        fullfile(root,"solver","clarabel","matlab"));
    egoBase = [.01;.01;.001;.01;.01;.001];
    targetBase = [.1;.1;.05;.05;.01;.01;.01;.01];
    unc = @(k) {"EgoErrorBound",k*egoBase,"TargetErrorBound",k*targetBase,"TargetJerkAmplitude",k*[.02;.02]};
    cases = {
        "ref-stationary-k0.01-pass", "stationary", .01, {}
        "27 crossing k-0.02", "crossing", -.02, {}
        "39 stationary k0 unc10", "stationary", 0, unc(10)
        "42 stationary k0.01 unc10", "stationary", .01, unc(10)
        "48 oncoming k0.01 unc10", "oncoming", .01, unc(10)
        "53 crossing k0.01 unc3", "crossing", .01, unc(3)
        "54 crossing k0.01 unc10", "crossing", .01, unc(10)
        "94 stationary k0.01 lat2", "stationary", .01, {"InitialTrackingError",[2;0;0;0;0]}
        "96 stationary k0.01 hdg0.1", "stationary", .01, {"InitialTrackingError",[0;.1;0;0;0]}
        "97 stationary k0.01 hdg0.2", "stationary", .01, {"InitialTrackingError",[0;.2;0;0;0]}
        "98 stationary k0.01 hdg0.3", "stationary", .01, {"InitialTrackingError",[0;.3;0;0;0]}
        "99 stationary k0.01 spd-2", "stationary", .01, {"InitialTrackingError",[0;0;-2;0;0]}};
    results = struct([]);
    for index = 1:size(cases,1)
        name = cases{index,1};
        rmappdata0();
        folder = fullfile(outputFolder,sprintf("case%02d",index));
        samples = 240;
        if startsWith(name,"ref"),samples = 1;end
        args = [{"Scenario",cases{index,2},"RoadCurvature",cases{index,3},"SampleCount",samples, ...
            "DeadlineSeconds",Inf,"SearchTimeLimitSeconds",30,"OutputDirectory",folder},cases{index,4}];
        failed = false;
        try
            runExactStateRecursiveFeasibilityScenario(args{:});
        catch exception
            failed = true;
            fprintf("%s: harness error %s\n",name,exception.identifier);
        end
        frame = getappdata(0,"diagnosticFrame");
        out = localDiagnose(frame.model,frame.cfg);
        out.name = name;out.harnessFailed = failed;
        results = [results,out]; %#ok<AGROW>
        localPrint(out);
        save(fullfile(outputFolder,"relaxationSubsets.mat"),"results");
    end
end

function rmappdata0()
    if isappdata(0,"diagnosticFrame"),rmappdata(0,"diagnosticFrame");end
end

function out = localDiagnose(model,cfg)
    big = 1e3;
    [program,~,~] = formulateAvoidanceProblem(model);
    [point,angles,information] = solveHardCbfClf.fluidInitialize(program,cfg);
    labels = program.physicalLabels;
    isExit = [program.jointCertificate.records.isExit];
    groups = ["exit","terminalCone","terminalEntry","terminalPoseDomain","poseDomain","referencePhaseDomain","collision"];
    present = [any(isExit),~isempty(program.terminalCone.bound),any(labels=="terminalEntry"), ...
        any(labels=="terminalPoseDomain"),any(labels=="poseDomain"),any(labels=="referencePhaseDomain"),any(~isExit)];
    active = find(present);
    subsets = false(0,numel(groups));feasible = false(0,1);
    for code = 0:2^numel(active)-1
        relax = false(1,numel(groups));relax(active) = bitget(code,1:numel(active))==1;
        p = program;a = angles;drop = false(size(isExit));
        if relax(1),drop = drop | isExit;end
        if relax(7),drop = drop | ~isExit;end
        if relax(2),p = localRelaxCones(p,big);end
        rows = false(size(labels));
        for g = 3:6
            if relax(g),rows = rows | labels==groups(g);end
        end
        p = localRelaxRows(p,rows,big);
        [p,a] = localDrop(p,a,drop);
        if isempty(p.jointCertificate.records)
            % The conic builder needs at least one record; keep a vacuous one.
            [p,a] = localKeepVacuous(program,angles,p,big);
        end
        status = localSolve(p,point,a,cfg);
        subsets(end+1,:) = relax; %#ok<AGROW>
        feasible(end+1,1) = any(status==[1,4]); %#ok<AGROW>
    end
    minimal = false(size(feasible));
    for k = find(feasible).'
        smaller = all(subsets<=subsets(k,:),2) & any(subsets<subsets(k,:),2) & feasible;
        minimal(k) = ~any(smaller);
    end
    prediction = program.prediction;
    out = struct("stateTime",model.stateTime,"horizonSteps",prediction.stageCount, ...
        "horizonSeconds",prediction.stageCount*model.sampleTime,"groups",groups,"present",present, ...
        "subsets",subsets,"feasible",feasible,"minimalRelaxations",{localNames(groups,subsets(minimal,:))}, ...
        "initialErrorBound",model.initialFrenetErrorBound,"terminalErrorBound",prediction.initialErrorBound(:,end), ...
        "initialization",information);
end

function [p,a] = localKeepVacuous(program,angles,p,big)
    keep = 1;
    p.jointCertificate.records = program.jointCertificate.records(keep);
    p.jointCertificate.records.clearance = p.jointCertificate.records.clearance-big;
    p.jointCertificate.angles = program.jointCertificate.angles(keep);
    p.jointCertificate.upperBound = program.jointCertificate.upperBound(keep);
    a = angles(keep);
end

function names = localNames(groups,rows)
    names = strings(size(rows,1),1);
    for k = 1:size(rows,1)
        if ~any(rows(k,:)),names(k) = "(none)";else,names(k) = strjoin(groups(rows(k,:)),"+");end
    end
end

function counts = localCounts(labels)
    names = unique(labels);counts = struct();
    for k = 1:numel(names),counts.(matlab.lang.makeValidName(names(k))) = nnz(labels==names(k));end
end

function [p,a] = localDrop(p,a,mask)
    p.jointCertificate.records(mask) = [];p.jointCertificate.angles(mask) = [];
    p.jointCertificate.upperBound(mask) = [];a(mask) = [];
end

function p = localRelaxRows(p,mask,big)
    geometric = numel(p.geometry.label);rows = find(mask);
    inGeometry = rows(rows<=geometric);other = rows(rows>geometric);
    p.safetyBound(inGeometry) = p.safetyBound(inGeometry)+big;
    p.b(other) = p.b(other)+big;
end

function p = localRelaxCones(p,big)
    count = numel(p.terminalCone.bound);
    tops = numel(p.b)-count+(1:3:count);
    p.b(tops) = p.b(tops)+big;
end

function [p,a] = localNoTerminal(p,a,isExit,labels,big)
    p = localRelaxCones(p,big);
    p = localRelaxRows(p,ismember(labels,["terminalEntry","terminalPoseDomain"]),big);
    [p,a] = localDrop(p,a,isExit);
end

function [status,decision] = localSolve(program,point,angles,cfg)
    conic = avoidanceStageQp.fixedDirections(program,point,angles,cfg);
    [reduced,retained] = solveHardCbfClf.reduce(conic);
    center = zeros(numel(reduced.q),1);
    center(1:numel(reduced.anchorPlan)) = reduced.anchorPlan;
    linear = reduced.q+reduced.P*center;bound = reduced.b-reduced.A*center;
    scale = 1/max([1;abs(linear);abs(nonzeros(reduced.P))]);
    options = [cfg.solver.constraintTolerance,cfg.solver.optimalityTolerance,cfg.solver.maxIterations];
    [native,info] = solveAvoidanceSocpMex(scale*reduced.P,scale*linear,sparse(reduced.A),bound,reduced.cones,options);
    status = double(info.status);
    expanded = zeros(numel(conic.q),1);expanded(retained) = native+center;
    decision = expanded(1:conic.primaryCount);
end

function states = localStates(prediction,plan)
    count = prediction.stageCount;states = zeros(6,count+1);
    for k = 1:count+1
        states(:,k) = prediction.egoStateOffset(:,k)+prediction.egoStateMatrix(:,:,k)*plan;
    end
end

function metrics = localMetrics(program,x,angles,isExit)
% Violations of the relaxed terminal requirements by the collision-only plan.
    records = program.jointCertificate.records;
    residual = avoidanceSafetyGeometry.jointResidual(program,x,angles);
    count = numel(program.terminalCone.bound);
    cone = program.b(end-count+1:end)-program.A(end-count+1:end,:)*x;
    modal = zeros(count/3,1);
    for m = 1:count/3,modal(m) = cone(3*m-2)-norm(cone(3*m-1:3*m));end
    labels = program.physicalLabels;
    rows = program.b(1:numel(labels))-program.A(1:numel(labels),:)*x;
    terminal = ismember(labels,["terminalEntry","terminalPoseDomain"]);
    states = localStates(program.prediction,x(program.layout.planIndex));
    metrics = struct("maxCollisionResidual",max(residual(~isExit)), ...
        "exitShortfallMeters",max(residual(isExit)), ...
        "minTerminalModalMargin",min(modal),"minTerminalRowMargin",min(rows(terminal)), ...
        "maxLateralMeters",max(abs(states(2,:))),"finalState",states(:,end), ...
        "records",numel(records));
end

function localPrint(out)
    fprintf("\n=== %s (t=%.2f s, horizon %.2f s, groups present: %s)\n",out.name,out.stateTime, ...
        out.horizonSeconds,strjoin(out.groups(out.present),", "));
    fprintf("  feasible subsets %d of %d\n",nnz(out.feasible),numel(out.feasible));
    fprintf("  minimal relaxations:\n");
    for k = 1:numel(out.minimalRelaxations),fprintf("    - %s\n",out.minimalRelaxations(k));end
    fprintf("  error bound initial [%s] terminal [%s]\n",num2str(out.initialErrorBound.',3),num2str(out.terminalErrorBound.',3));
end
