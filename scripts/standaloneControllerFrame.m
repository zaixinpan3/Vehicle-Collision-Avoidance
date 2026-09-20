function [decision,angles,status,metrics] = standaloneControllerFrame(program,cfg)
%standaloneControllerFrame Native replay of a prepared joint-certificate frame.
%#codegen
% Preparation, sensing, reference synthesis and witness shifting remain with
% the MATLAB orchestrator. This entry is NOT a standalone closed-loop driver.
% Status: 0 rejected, 1 scalar admission, 2 optimized, 3 retained incumbent.
% Metrics: assembly/admission, native solve, verification seconds, solver status.
    coder.cinclude('nativeControllerBenchmarkBridge.h');
    assert(~program.terminalOptimization);
    program.terminalOptimization=false;
    decision=program.feasibleWitness;angles=program.jointCertificate.angles;
    metrics=zeros(4,1);status=0;
    start=localClock();
    [accepted,program]=localVerify(program,decision,angles);
    metrics(3)=localClock()-start;
    if ~program.inheritedPredictionFamily && ~accepted
        start=localClock();
        [candidate,directions]=solveHardCbfClf.admitSection(program,cfg);
        decision=candidate(:);angles=directions(:);
        metrics(1)=localClock()-start;
        if isempty(decision),return;end
        start=localClock();[accepted,~]=localVerify(program,decision,angles);
        metrics(3)=metrics(3)+localClock()-start;
        if accepted,status=1;end
        return;
    end
    if ~accepted,return;end
    incumbent=decision;oldAngles=angles;status=3;
    start=localClock();
    program.b(1:numel(program.safetyBound))=program.safetyBound;
    program.b(end-numel(program.terminalCone.bound)+1:end)=program.terminalCone.bound;
    program.terminalOptimization=false;
    conic=avoidanceStageQp.joint(program,decision,angles,cfg);
    [reduced,retained]=solveHardCbfClf.reduce(conic);
    center=zeros(numel(reduced.q),1);
    center(1:numel(reduced.anchorPlan))=reduced.anchorPlan;
    linear=reduced.q+reduced.P*center;bound=reduced.b-reduced.A*center;
    scale=1/max([1;abs(linear);abs(nonzeros(reduced.P))]);
    [pr,pc,pv]=find(scale*reduced.P);[ar,ac,av]=find(reduced.A);
    metrics(1)=localClock()-start;
    start=localClock();
    native=zeros(numel(linear),1);solverStatus=0;
    options=[cfg.solver.constraintTolerance;cfg.solver.optimalityTolerance;cfg.solver.maxIterations];
    if coder.target('MATLAB')
        [native,info]=solveAvoidanceSocpMex(scale*reduced.P,scale*linear,reduced.A,bound,reduced.cones,options);
        solverStatus=double(info.status);
    else
        scaledLinear=scale*linear;coneSizes=reduced.cones;
        solverStatus=coder.ceval('benchmark_solve',int32(numel(linear)),int32(numel(bound)), ...
            int32(numel(pv)),coder.rref(pr),coder.rref(pc),coder.rref(pv), ...
            int32(numel(av)),coder.rref(ar),coder.rref(ac),coder.rref(av), ...
            coder.rref(scaledLinear),coder.rref(bound),int32(numel(reduced.cones)), ...
            coder.rref(coneSizes),coder.rref(options),coder.wref(native));
    end
    metrics(2)=localClock()-start;metrics(4)=solverStatus;
    if solverStatus==1 || solverStatus==4
        start=localClock();
        expanded=zeros(numel(conic.q),1);expanded(retained)=native+center;
        proposed=expanded(1:conic.primaryCount);
        newAngles=angles+atan(expanded(conic.angleIndex));
        [accepted,~]=localVerify(program,proposed,newAngles);
        metrics(3)=metrics(3)+localClock()-start;
        if accepted,decision=proposed;angles=newAngles;status=2;return;end
    end
    decision=incumbent;angles=oldAngles;
end

function [accepted,program]=localVerify(program,decision,angles)
    accepted=false;
    if numel(decision)~=numel(program.q) || any(~isfinite(decision)),return;end
    [status,bound,terminal]=solveHardCbfClf.inspect(program,decision);
    if status~=0,return;end
    values=avoidanceSafetyGeometry.jointResidual(program,decision,angles);
    allowance=avoidanceSafetyGeometry.jointAllowance(program,decision);
    if any(~isfinite(values)) || any(values+allowance>0),return;end
    accepted=true;program.safetyBound=bound;program.terminalCone.bound=terminal;
    program.jointCertificate.angles=angles;
    program.jointCertificate.upperBound=max(program.jointCertificate.upperBound,values+allowance);
end

function seconds=localClock()
    coder.cinclude('nativeControllerBenchmarkBridge.h');
    if coder.target('MATLAB')
        seconds=toc(localTimer());
    else
        seconds=0;seconds=coder.ceval('benchmark_clock');
    end
end

function timer=localTimer()
    persistent reference
    if isempty(reference),reference=tic;end
    timer=reference;
end
