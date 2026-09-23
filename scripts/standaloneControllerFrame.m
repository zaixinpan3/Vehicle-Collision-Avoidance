function [decision,angles,status,metrics] = standaloneControllerFrame(program,cfg)
%standaloneControllerFrame Native replay of a prepared joint-certificate frame.
%#codegen
% Preparation, sensing, reference synthesis and witness shifting remain with
% the MATLAB orchestrator. This entry is NOT a standalone closed-loop driver.
% Status: 0 rejected, 2 optimized continuation, 3 retained shifted previous plan,
% 4 optimized admission. A direction-search seed is never issued directly.
% A solver-reported success is issued as returned.
% Metrics: assembly/admission, native solve, unused (0), solver status.
    coder.cinclude('nativeControllerBenchmarkBridge.h');
    assert(~program.terminalOptimization);
    program.terminalOptimization=false;
    decision=program.feasibleWitness;angles=program.jointCertificate.angles;
    metrics=zeros(4,1);status=0;
    admission=~program.inheritedPredictionFamily;
    if admission && program.fluidReference.active
        start=localClock();
        [decision,angles]=solveHardCbfClf.fluidInitialize(program,cfg);
        metrics(1)=localClock()-start;
        if isempty(decision),return;end
    end
    incumbent=decision;oldAngles=angles;
    if ~admission,status=3;end
    start=localClock();
    conic=avoidanceStageQp.fixedDirections(program,decision,angles,cfg);
    [reduced,retained]=solveHardCbfClf.reduce(conic);
    center=zeros(numel(reduced.q),1);
    center(1:numel(reduced.anchorPlan))=reduced.anchorPlan;
    linear=reduced.q+reduced.P*center;bound=reduced.b-reduced.A*center;
    scale=1/max([1;abs(linear);abs(nonzeros(reduced.P))]);
    [pr,pc,pv]=find(scale*reduced.P);[ar,ac,av]=find(reduced.A);
    metrics(1)=metrics(1)+localClock()-start;
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
    if (solverStatus==1 || solverStatus==4) && all(isfinite(native))
        expanded=zeros(numel(conic.q),1);expanded(retained)=native+center;
        decision=expanded(1:conic.primaryCount);status=2;
        if admission,status=4;end
        return;
    end
    decision=incumbent;angles=oldAngles;
    if admission,decision=zeros(0,1);end
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
