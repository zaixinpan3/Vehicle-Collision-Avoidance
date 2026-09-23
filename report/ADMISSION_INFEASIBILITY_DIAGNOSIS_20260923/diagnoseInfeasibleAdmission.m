function results = diagnoseInfeasibleAdmission(outputFolder)
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
        save(fullfile(outputFolder,"infeasibleAdmissionDiagnosis.mat"),"results");
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
    out = struct("stateTime",model.stateTime,"horizonSteps",program.prediction.stageCount, ...
        "horizonSeconds",program.prediction.stageCount*model.sampleTime, ...
        "initialEgoState",model.initialEgoState,"initialErrorBound",model.initialFrenetErrorBound, ...
        "targetCenter",model.encounter.center,"targetRadius",model.encounter.radius, ...
        "initialization",information,"labelCounts",localCounts(labels));
    variants = {
        "baseline",           @(p,a) deal(p,a)
        "noExit",             @(p,a) localDrop(p,a,isExit)
        "noTerminalCone",     @(p,a) deal(localRelaxCones(p,big),a)
        "noTerminalRows",     @(p,a) deal(localRelaxRows(p,ismember(labels,["terminalEntry","terminalPoseDomain"]),big),a)
        "noTerminalAtAll",    @(p,a) localNoTerminal(p,a,isExit,labels,big)
        "noChartDomain",      @(p,a) deal(localRelaxRows(p,ismember(labels,["poseDomain","referencePhaseDomain"]),big),a)
        "noSlew",             @(p,a) deal(localRelaxRows(p,labels=="slew",big),a)
        "noActuatorAmplitude",@(p,a) deal(localRelaxRows(p,labels=="actuator",big),a)
        "noCollision",        @(p,a) localDrop(p,a,~isExit)};
    out.variants = struct("name",{},"status",{},"feasible",{},"decision",{});
    for v = 1:size(variants,1)
        [p,a] = variants{v,2}(program,angles);
        [status,decision] = localSolve(p,point,a,cfg);
        out.variants(end+1) = struct("name",variants{v,1},"status",status,"feasible",any(status==[1,4]),"decision",decision);
    end
    % The other passing side of the two-candidate fluid reference.
    out.otherSide = struct("status",NaN,"feasible",false,"amplitude",NaN);
    if program.fluidReference.active
        chosen = find(abs(program.fluidReference.amplitudes-information.amplitudeMeters)<1e-12,1);
        other = program;other.fluidReference.valid(chosen) = false;
        [otherPoint,otherAngles,otherInfo] = solveHardCbfClf.fluidInitialize(other,cfg);
        if ~isempty(otherPoint)
            status = localSolve(program,otherPoint,otherAngles,cfg);
            out.otherSide = struct("status",status,"feasible",any(status==[1,4]),"amplitude",otherInfo.amplitudeMeters);
        end
    end
    % A longer prediction horizon (+1.6 s).
    longer = model;longer.horizonSteps = model.horizonSteps+32;
    [longProgram,~,~] = formulateAvoidanceProblem(longer);
    [longPoint,longAngles] = solveHardCbfClf.fluidInitialize(longProgram,cfg);
    status = NaN;
    if ~isempty(longPoint),status = localSolve(longProgram,longPoint,longAngles,cfg);end
    out.longerHorizon = struct("steps",longProgram.prediction.stageCount,"status",status,"feasible",any(status==[1,4]));
    % Metrics of the collision-only relaxation and directions re-derived from it.
    relaxed = out.variants(strcmp([out.variants.name],"noTerminalAtAll"));
    out.relaxedMetrics = struct();out.redirected = struct("status",NaN,"feasible",false);
    if relaxed.feasible
        x = relaxed.decision;plan = x(program.layout.planIndex);
        out.relaxedMetrics = localMetrics(program,x,angles,isExit);
        states = localStates(program.prediction,plan);
        redirected = angles;records = program.jointCertificate.records;
        for r = find(~isExit)
            item = records(r);state = states(:,item.stage+1);
            relative = item.positionOffset+item.positionMap*state;
            yaw = item.yawOffset+item.yawRow*state;
            normal = avoidanceSafetyGeometry.supportDirection(relative,yaw,[0;0],item.targetYaw, ...
                [item.egoHalfSize;item.targetHalfSize]);
            redirected(r) = atan2(normal(2),normal(1));
        end
        status = localSolve(program,x,redirected,cfg);
        out.redirected = struct("status",status,"feasible",any(status==[1,4]));
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
    fprintf("\n=== %s (t=%.2f s, horizon %d steps = %.2f s, init %s, amplitude %.3f m)\n", ...
        out.name,out.stateTime,out.horizonSteps,out.horizonSeconds,out.initialization.status, ...
        out.initialization.amplitudeMeters);
    for v = out.variants
        fprintf("  %-20s status %d %s\n",v.name,v.status,localWord(v.feasible));
    end
    fprintf("  %-20s status %g %s (amplitude %.3f m)\n","otherSide",out.otherSide.status, ...
        localWord(out.otherSide.feasible),out.otherSide.amplitude);
    fprintf("  %-20s status %g %s (%d steps)\n","longerHorizon",out.longerHorizon.status, ...
        localWord(out.longerHorizon.feasible),out.longerHorizon.steps);
    fprintf("  %-20s status %g %s\n","redirected",out.redirected.status,localWord(out.redirected.feasible));
    if isfield(out.relaxedMetrics,"exitShortfallMeters")
        m = out.relaxedMetrics;
        fprintf("  collision-only plan: max collision residual %.3g, exit shortfall %.3f m, terminal modal margin %.4f, terminal row margin %.4f, max |lateral| %.2f m\n", ...
            m.maxCollisionResidual,m.exitShortfallMeters,m.minTerminalModalMargin,m.minTerminalRowMargin,m.maxLateralMeters);
    end
end

function word = localWord(feasible)
    word = "infeasible";if feasible,word = "FEASIBLE";end
end
