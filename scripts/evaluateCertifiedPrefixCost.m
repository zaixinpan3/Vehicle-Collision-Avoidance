function result = evaluateCertifiedPrefixCost(inputDirectory,outputDirectory)
%evaluateCertifiedPrefixCost Compare performance formulations on one certificate.
% Research ablations only. Every output is checked against all original hard
% rows; removed CLF cones change performance, and their slacks are reconstructed.
    arguments
        inputDirectory (1,1) string
        outputDirectory (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'solver','clarabel','matlab'));
    loaded=load(fullfile(inputDirectory,'original-functional.mat'),'unboundedRuntime');t=loaded.unboundedRuntime;
    [~,~,problem]=collisionAvoidanceController(t.attempts.controllerEgoEstimate{1},t.attempts.targetEstimate{1}, ...
        t.attempts.roadPerception{1}.roadGeometry,t.controllerConfiguration,[]);
    cfg=problem.model.cfg;qp=problem.qp;program=qp.stageProgram;
    incumbent=localLift(problem.decision,program,problem.prediction,problem.model);
    methods=["inheritedMarginSocp","trackingQp","fourStepTrackingPrefix"];
    result=struct('scope',"affine certificate ablations, not feedback-tube or runtime deployment", ...
        'originalMargin',problem.metadata.acceptance.margin,'methods',[]);
    candidates=cell(3,1);
    for method=1:3
        p=program;fixed=zeros(0,1);fixedValue=zeros(0,1);
        if method>=2
            selected=[p.rowMap.equality;p.rowMap.inequality];
            p.A=p.A(selected,:);p.b=p.b(selected);p.cones=p.cones(1:2);
            fixed=qp.layout.relaxationIndex(:);fixedValue=zeros(numel(fixed),1);
        end
        if method==3
            prefix=4;tailCells=find([problem.prediction.cells.stage]>=prefix+1);
            tailStates=program.stateIndex(:,tailCells);
            % The inlet at t=L*h and all later nominal states retain their
            % certified values. All tail inputs are retained as well.
            tailStates=[tailStates(:);program.stateIndex(:,end)];
            additional=[(2*prefix+1:qp.layout.planCount).';tailStates];
            fixed=[fixed;additional];fixedValue=[fixedValue;incumbent(additional)];
        end
        keep=true(numel(p.q),1);keep(fixed)=false;
        hessian=p.P(keep,keep);
        linear=full(p.q(keep)+(p.P(keep,fixed)+p.P(fixed,keep).')*fixedValue);
        bound=full(p.b-p.A(:,fixed)*fixedValue);matrix=p.A(:,keep);
        samples=zeros(5,1);statuses=zeros(5,1);accepted=false(5,1);
        for replay=1:5
            timer=tic;scale=1/max([1;abs(linear);abs(nonzeros(hessian))]);
            [x,info]=solveAvoidanceSocpMex(scale*hessian,scale*linear,matrix,bound,p.cones, ...
                [cfg.solver.constraintTolerance,cfg.solver.optimalityTolerance*scale,cfg.solver.maxIterations]);
            samples(replay)=toc(timer);statuses(replay)=info.status;
            expanded=zeros(numel(p.q),1);expanded(keep)=x;expanded(fixed)=fixedValue;
            decision=expanded(1:qp.layout.decisionCount);decision=localRepair(qp,decision,cfg);
            check=certifyAvoidancePlan(qp,problem.prediction,problem.model,decision);
            accepted(replay)=any(info.status==[1,4]) && check.accepted;
        end
        entry=struct('method',methods(method),'nativeRows',size(matrix,1),'nativeVariables',nnz(keep), ...
            'nativeCones',numel(p.cones)-2,'seconds',samples,'medianSeconds',median(samples(2:end)), ...
            'status',statuses,'accepted',accepted,'margin',check.margin, ...
            'inputChange',norm(decision(qp.layout.planIndex)-problem.decision(qp.layout.planIndex)), ...
            'objective',.5*decision.'*qp.Hessian*decision+qp.linear.'*decision+qp.constant);
        result.methods=[result.methods;entry];candidates{method}=decision;
    end
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    save(fullfile(outputDirectory,'prefix-cost.mat'),'result','candidates','problem');
    file=fopen(fullfile(outputDirectory,'prefix-cost.json'),'w');assert(file>=0);
    cleanup=onCleanup(@()fclose(file));fprintf(file,'%s\n',jsonencode(result,PrettyPrint=true));
    for entry=result.methods.'
        fprintf('%s: %.3f ms, %d vars, %d cones, accepted %d/5\n',entry.method, ...
            1e3*entry.medianSeconds,entry.nativeVariables,entry.nativeCones,nnz(entry.accepted));
    end
end

function decision=localLift(physical,program,prediction,model)
    decision=zeros(numel(program.q),1);decision(1:numel(physical))=physical;
    states=zeros(size(program.stateCenter));states(:,1)=model.initialEgoState;
    for k=1:numel(prediction.cells)
        tube=prediction.cells(k);states(:,k+1)=tube.endMap*physical(1:prediction.planCount)+tube.endOffset;
    end
    decision(program.stateIndex(:))=states(:)-program.stateCenter(:);
end

function decision=localRepair(qp,decision,cfg)
    slacks=zeros(qp.layout.relaxationCount,1);
    for k=1:numel(qp.clf.constraints)
        c=qp.clf.constraints(k);value=c.map*decision+c.offset;
        v=norm(c.root*value)^2+c.linear.'*value+c.constant;
        v=v+16*(numel(decision)+64)*eps*(norm(abs(c.root)*abs(value))^2+abs(c.linear).'*abs(value)+abs(c.constant));
        slacks(c.stage)=max(slacks(c.stage),v);
    end
    decision(qp.layout.relaxationIndex)=slacks+cfg.encounter.numericalMargin*(1+abs(slacks));
end
