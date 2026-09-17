classdef solveHardCbfClf
    %solveHardCbfClf Certified convex trajectory solves and independent hard-safety checks.
    methods (Static)
        function program = certify(program,decision)
        % Validate physical safety independently of a numerical success flag.
        % Transfer a feasible affine family WITHOUT repeated inward tightening.
            gamma=64*numel(decision)*eps;
            value=program.physicalMatrix*decision;
            allowance=gamma*(1+abs(program.physicalBound)+abs(program.physicalMatrix)*abs(decision));
            certified=max(program.safetyBound,value+allowance);
            map=program.terminalCone.matrix;
            input=decision(program.layout.planIndex);
            cone=program.terminalCone.bound-map*input;
            coneAllowance=gamma*(1+abs(program.terminalCone.bound)+abs(map)*abs(input));
            tops=1:3:numel(cone);
            adjusted=program.terminalCone.bound;
            for first=tops
                adjusted(first)=adjusted(first)+max(0,norm(cone(first+(1:2))) ...
                    +norm(coneAllowance(first+(0:2)))-cone(first));
            end
            first=program.cones(2)+1;
            clf=program.b(first:first+5)-program.A(first:first+5,:)*decision;
            if any(~isfinite(certified)) || any(certified>program.physicalBound) ...
                    || any(~isfinite(adjusted)) || any(adjusted(tops)>program.terminalConePhysicalBound(tops))
                [excess,row]=max(certified-program.physicalBound);
                error('collisionAvoidanceController:optimizationFailed', ...
                    ['The returned solution failed the hard-safety certificate ' ...
                    '(linear excess %.9g at %s, terminal excess %.9g). No command was issued.'], ...
                    excess,program.physicalLabels(row), ...
                    max(adjusted(tops)-program.terminalConePhysicalBound(tops)));
            end
            if norm(clf(2:end))-clf(1)>program.clfNumericalReserve ...
                    || decision(end)<-program.clfNumericalReserve
                error('collisionAvoidanceController:optimizationFailed', ...
                    'The returned solution failed the reserved soft-CLF inequality. No command was issued.');
            end
            program.safetyBound=certified;
            program.terminalCone.bound=adjusted;
        end

        function solve = constrained(program,cfg)
            program = localCompactPlanarRows(program);
            decisionCount=numel(program.q);
            program=avoidanceStageQp.build(program);
            problem = struct('layout',struct('decisionCount',decisionCount),'stageProgram',program);
            solve = localGeneratedSolve(problem,cfg);
        end

        function [solve,restoration] = restore(program,cfg)
        % Search-only deficits. This result is never an executable certificate.
            n=program.layout.planCount;labels=program.physicalLabels;
            soft=startsWith(labels,"collision:") | startsWith(labels,"road:") ...
                | startsWith(labels,"exit:") | labels=="terminalEntry";
            keys=labels;
            geometric=numel(program.geometry.label);
            keys(1:geometric)=keys(1:geometric)+":"+string(program.geometry.cellIndex);
            [names,~,group]=unique(keys(soft),'stable');
            count=numel(names);cones=numel(program.terminalCone.sizes);total=count+cones;
            column=zeros(numel(labels),1);column(soft)=group;
            charge=sparse(find(soft),column(soft),1,numel(labels),total);
            matrix=[program.physicalMatrix(:,1:n),-charge;zeros(total,n),-eye(total)];
            bound=[program.safetyBound;zeros(total,1)];
            coneCharge=sparse(1:3:3*cones,count+(1:cones),-1,3*cones,total);
            matrix=[matrix;program.terminalCone.matrix,coneCharge];
            bound=[bound;program.terminalCone.bound];
            scales=[repmat(cfg.vehicle.width+cfg.collision.clearanceMargin,count,1); ...
                max(program.terminal.radius(:),sqrt(eps))];
            restoration=struct('P',sparse(n+total,n+total),'q',[zeros(n,1);1./scales], ...
                'A',sparse(matrix),'b',bound,'cones',[0;numel(labels)+total;program.terminalCone.sizes], ...
                'anchorPlan',[program.anchorPlan;zeros(total,1)], ...
                'deficitNames',[names;"terminalCone:"+string((1:cones).')], ...
                'deficitScale',scales,'planCount',n, ...
                'prediction',program.prediction,'geometry',program.geometry, ...
                'cruiseCertificate',program.cruiseCertificate,'terminal',program.terminal, ...
                'terminalOptimization',program.terminalOptimization, ...
                'safetyBound',program.safetyBound,'restoration',true);
            solve=solveHardCbfClf.constrained(restoration,cfg);
            solve.deficits=inf(total,1);solve.normalizedDeficit=Inf;
            if solve.feasible
                solve.deficits=max(0,solve.decision(n+1:end));
                solve.normalizedDeficit=sum(solve.deficits./scales);
            end
        end

    end
end

function solve=localGeneratedSolve(problem,cfg)
% Constraint generation solves the same convex program. Every omitted row
% is checked against the complete returned decision before convergence.
    full=problem.stageProgram;count=full.cones(2);equalities=full.cones(1);
    if ~isfield(full,'retainedRows') || numel(full.geometry.label)<400 ...
            || ~isempty(cfg.solver.jointFunction)
        solve=localRunJointProgram(problem,cfg);return;
    end
    source=full.retainedRows(1:count);geometric=source<=numel(full.geometry.label);
    active=~geometric;groups=full.geometry.cellIndex(source(geometric));
    rows=find(geometric);anchor=full.anchorPlan;
    residual=full.A(equalities+rows,:)*anchor-full.b(equalities+rows);
    sorted=sortrows([groups,-residual,rows],[1,2]);
    first=[true;diff(sorted(:,1))~=0];starts=find(first);
    selected=unique([starts;min(starts+1,[starts(2:end)-1;size(sorted,1)])]);
    active(sorted(selected,3))=true;
    nativeCalls=0;
    for iteration=1:12
        selected=[(1:equalities).';equalities+find(active); ...
            (equalities+count+1:numel(full.b)).'];
        reduced=full;reduced.A=full.A(selected,:);reduced.b=full.b(selected);
        reduced.cones(2)=nnz(active);trial=problem;trial.stageProgram=reduced;
        solve=localRunJointProgram(trial,cfg);nativeCalls=nativeCalls+1;
        if ~solve.feasible,break;end
        value=full.A(equalities+(1:count),:)*solve.fullDecision-full.b(equalities+(1:count));
        allowance=cfg.solver.constraintTolerance*(1+abs(full.b(equalities+(1:count))));
        violated=~active & value>allowance;
        if ~any(violated),break;end
        % Add the worst row of each violated held-cell family. The original
        % full verifier remains authoritative for physical safety.
        rows=find(violated);groups=full.geometry.cellIndex(source(rows));
        sorted=sortrows([groups,-value(rows),rows],[1,2]);
        first=[true;diff(sorted(:,1))~=0];active(sorted(first,3))=true;
        if iteration==12,solve.feasible=false;solve.message="Constraint generation did not close the complete program.";end
    end
    solve.output.constraintGenerationSolves=nativeCalls;
    solve.output.generatedLinearRows=nnz(active);
end

function program = localCompactPlanarRows(program)
% Intersect all one-coordinate inequalities before the numerical solve.
% This is an equivalent hard-bound reduction (with inward roundoff), not
% a check of a returned plan. Coupled rows and every cone remain unchanged.
    if numel(program.q)~=2 || program.cones(1)~=0,return;end
    count = program.cones(2);
    linear = program.A(1:count,:);bound = program.b(1:count);
    if isfield(program,'decisionRadius')
        radius = program.decisionRadius;
        coefficients = full(linear);
        [small,coordinate] = min(abs(coefficients),[],2);
        large = max(abs(coefficients),[],2);
        selected = small>0 & small<=1e-10*large;
        rows = find(selected);
        % Retain small cross-effects as an uncertainty support over explicit
        % hard decision bounds. No coefficient is silently rounded to zero.
        support = small(selected).*radius(coordinate(selected));
        bound(selected) = bound(selected)-support ...
            -8*eps(max(1,abs(bound(selected))+support));
        coefficients(sub2ind(size(coefficients),rows,coordinate(selected))) = 0;
        linear = [sparse(coefficients);sparse([1,0;-1,0;0,1;0,-1])];
        bound = [bound;radius(1);radius(1);radius(2);radius(2)];
    end
    first = full(linear(:,1));second = full(linear(:,2));
    axis = (first~=0 & second==0) | (first==0 & second~=0);
    if ~any(axis),return;end
    indices = find(axis);
    coefficient = first(axis)+second(axis);
    limit = bound(axis)./abs(coefficient);
    limit = limit-8*eps(max(1,abs(limit)));
    finite = isfinite(limit);
    axis(indices(~finite)) = false;
    coefficient = coefficient(finite);limit = limit(finite);
    coordinate = 1+double(second(axis)~=0);
    group = 2*coordinate-1+double(coefficient<0);
    compact = accumarray(group,limit,[4,1],@min,Inf);
    present = isfinite(compact);
    directions = [1,0;-1,0;0,1;0,-1];
    retained = ~axis;
    program.A = [linear(retained,:);sparse(directions(present,:));program.A(count+1:end,:)];
    program.b = [bound(retained);compact(present);program.b(count+1:end)];
    program.cones(2) = nnz(retained)+nnz(present);
end

function solve = localRunJointProgram(problem, cfg)
    hook = cfg.solver.jointFunction;
    try
        if isempty(hook)
            solve = localDefaultSolve(problem,cfg);
        else
            % The hook solves the same reduced conic program as the native
            % path: inactive columns are removed together with the rows they
            % leave identically zero, so no degenerate dependent constraints
            % remain. The hook must satisfy the same feasibility/status contract.
            program = problem.stageProgram;
            [reduced,retained] = localReducedProgram(program);
            reduced.defaultSolver = @() localReduceSolve(localDefaultSolve(problem, cfg), retained);
            solve = hook("joint", reduced);
            if isstruct(solve) && isscalar(solve) && isfield(solve,"decision") ...
                    && numel(solve.decision)==nnz(retained)
                expanded = zeros(numel(program.q),1);
                expanded(retained) = solve.decision;
                solve.decision = expanded;
            end
        end
    catch exception
        solve = localEmptySolve();
        solve.exitFlag = -999;
        solve.output = struct("message", string(exception.message));
    end
    if isstruct(solve) && isscalar(solve) && isfield(solve,"decision") ...
            && numel(solve.decision)==numel(problem.stageProgram.q) ...
            && all(isfinite(solve.decision),"all")
        solve.fullDecision = solve.decision(:);
        solve.decision = solve.decision(1:problem.layout.decisionCount);
    end
    solve = localNormalizeSolve(solve, problem.layout.decisionCount, numel(problem.stageProgram.q));
end

function [reduced,retained] = localReducedProgram(program)
% Remove inactive columns and the equality/nonnegative rows they leave
% identically zero with a satisfiable right-hand side.
    retained = true(numel(program.q),1);
    if isfield(program,"inactiveSlackIndex"),retained(program.inactiveSlackIndex) = false;end
    matrix = program.A(:,retained);
    equalities = program.cones(1);
    linearRows = equalities+program.cones(2);
    rowNorm = full(sum(abs(matrix(1:linearRows,:)),2));
    trivialEquality = (1:equalities).' <= equalities & rowNorm(1:equalities)==0 & program.b(1:equalities)==0;
    trivialInequality = rowNorm(equalities+1:linearRows)==0 & program.b(equalities+1:linearRows)>=0;
    keep = true(size(matrix,1),1);
    keep(1:equalities) = ~trivialEquality;
    keep(equalities+1:linearRows) = ~trivialInequality;
    reduced = program;
    reduced.P = program.P(retained,retained);
    reduced.q = program.q(retained);
    reduced.A = matrix(keep,:);
    reduced.b = program.b(keep);
    reduced.cones = [equalities-nnz(trivialEquality);program.cones(2)-nnz(trivialInequality);program.cones(3:end)];
    reduced.inactiveSlackIndex = zeros(1,0);
end

function solve = localReduceSolve(solve, retained)
    if isstruct(solve) && isfield(solve,"decision") && numel(solve.decision)==numel(retained)
        solve.decision = solve.decision(retained);
    end
end

function solve = localDefaultSolve(problem, cfg)
    persistent nativeSolver
    if isempty(nativeSolver)
        if exist("solveAvoidanceSocpMex", "file") ~= 3
            solverPath = fullfile(fileparts(fileparts(mfilename("fullpath"))), ...
                "solver", "clarabel", "matlab");
            if isfolder(solverPath)
                addpath(solverPath);
            end
            if exist("solveAvoidanceSocpMex", "file") ~= 3
                error("collisionAvoidanceController:missingSocpSolver", ...
                    "Build the native solver once using scripts/buildAvoidanceSocpSolver.m.");
            end
        end
        nativeSolver = @solveAvoidanceSocpMex;
    end
    original = problem.stageProgram;
    [program,retained] = localReducedProgram(original);
    center = zeros(numel(program.q),1);
    if isfield(program,'anchorPlan')
        center(1:numel(program.anchorPlan)) = program.anchorPlan;
    end
    % Translate the full plan about its carried/trim seed. This is an exact
    % coordinate change; it does not constrain the optimizer toward that seed.
    linear = program.q+program.P*center;
    bound = program.b-program.A*center;
    % Positive objective scaling preserves minimizers. The lifted CLF
    % epigraph can otherwise trigger a false native infeasibility report.
    hessian = sparse(program.P);
    objectiveScale = 1/max([1;abs(linear);abs(nonzeros(hessian))]);
    % Apply the requested optimization tolerance to the normalized objective.
    % Rescaling it a second time can demand sub-machine objective accuracy;
    % the independent primal feasibility tolerance is unchanged.
    options = [cfg.solver.constraintTolerance, ...
        cfg.solver.optimalityTolerance,cfg.solver.maxIterations];
    if isfield(cfg.solver,"workTimer") && isfinite(cfg.solver.workTimeLimit)
        remaining = cfg.solver.workTimeLimit-toc(cfg.solver.workTimer);
        if remaining<=0
            solve = localEmptySolve();
            solve.exitFlag = 0;
            solve.output = struct("message","The program work deadline expired before the native solve.");
            return;
        end
        options(4) = remaining;
    end
    [nativeDecision, output] = nativeSolver( ...
        objectiveScale*hessian, objectiveScale*linear, sparse(program.A), bound, program.cones, ...
        options);
    output.objectiveValue = output.objectiveValue/objectiveScale ...
        +0.5*center.'*program.P*center+program.q.'*center;
    output.objectiveScale = objectiveScale;
    stageDecision = zeros(numel(original.q),1);stageDecision(retained) = nativeDecision+center;
    flag = -7;
    switch output.status
        case 1
            flag = 1;
        case 4
            flag = 2;
        case {2, 5}
            flag = -2;
        case {3, 6}
            flag = -3;
        case {7, 8}
            flag = 0;
    end
    output.algorithm = "Clarabel hard-safety soft-CLF SOCP";
    output.message = "Clarabel status "+string(output.status);
    solve = struct("decision", stageDecision, ...
        "exitFlag", flag, "output", output);
end

function solve = localNormalizeSolve(solve, decisionCount, variableCount)
    if ~isstruct(solve) || ~isscalar(solve)
        solve = localEmptySolve();
        solve.message = "solver hook returned no scalar result structure";
        return;
    end
    if ~isfield(solve, "decision"), solve.decision = zeros(0, 1); end
    if ~isfield(solve, "fullDecision"), solve.fullDecision = zeros(variableCount, 1); end
    if ~isfield(solve, "exitFlag"), solve.exitFlag = -999; end
    if ~isfield(solve, "output"), solve.output = struct(); end
    if ~isnumeric(solve.exitFlag) || ~isreal(solve.exitFlag) ...
            || ~isscalar(solve.exitFlag) || ~isfinite(solve.exitFlag)
        solve.exitFlag = -999;
    end
    solve.exitFlag = double(solve.exitFlag);
    % Exact optimality is unnecessary for recursive feasibility. A positive
    % approximate-solve status may proceed ONLY to independent certification.
    % Infeasibility, iteration limits and timeouts still issue no command.
    solve.feasible = any(solve.exitFlag==[1,2]) ...
        && isnumeric(solve.decision) && isreal(solve.decision) ...
        && numel(solve.decision) == decisionCount ...
        && all(isfinite(solve.decision), "all");
    solve.decision = solve.decision(:);
    solve.message = "solver exit flag "+string(solve.exitFlag);
    if isstruct(solve.output) && isfield(solve.output, "message")
        solve.message = solve.message+": "+string(solve.output.message);
    end
end

function solve = localEmptySolve()
    solve = struct("decision", zeros(0, 1), "fullDecision", zeros(0, 1), ...
        "exitFlag", -999, "output", struct(), "feasible", false, ...
        "message", "");
end
