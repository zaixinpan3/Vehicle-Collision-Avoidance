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
            if isfield(program,'jointCertificate')
                program=avoidanceSafetyGeometry.certifyJoint(program,decision);
            end
        end

        function [program,result,search] = joint(program,model,cfg)
        % Restoration is unexecuted. Only a separately verified incumbent can
        % leave this method, including when an improvement solve is stopped.
            [program,result,search]=localJointSearch(program,model,cfg);
        end

        function solve = constrained(program,cfg)
            program = localCompactPlanarRows(program);
            decisionCount=numel(program.q);
            program=avoidanceStageQp.build(program);
            problem = struct('layout',struct('decisionCount',decisionCount),'stageProgram',program);
            solve = localRunJointProgram(problem,cfg);
        end

    end
end

function [program,result,search]=localJointSearch(program,~,cfg)
% One geometric initialization, bounded feasibility search, then hard checking.
% A verified admission plan can be issued directly. Continuation still solves
% one performance problem containing its complete certified incumbent.
    search=struct('hardSolves',0,'restorationSolves',0,'baseSolves',0,'nativeSolves',0, ...
        'familyAttempts',0,'horizonAttempts',1,'formulationSeconds',0,'solveSeconds',0, ...
        'initialOverlappingNodes',program.supportGeometry.overlappingNodes, ...
        'policy',"boundedJointSupport",'violationHistory',{{}},'usedCertifiedIncumbent',false, ...
        'issuedAdmissionWitness',false,'initialCertificateAngles',program.jointCertificate.angles);
    point=program.feasibleWitness;angles=program.jointCertificate.angles;
    [admitted,accepted]=localVerifyJointPoint(program,point,angles);
    if admitted,program=localRefreshBounds(accepted);end
    if ~admitted
        % Dynamics, actuator, chart, CLF and terminal constraints stay hard.
        value=program.b-program.A*point;
        if ~localConesContain(value,program.cones)
            phase=tic;result=solveHardCbfClf.constrained(program,cfg);
            search.solveSeconds=toc(phase);search.baseSolves=1;search.nativeSolves=1;
            if ~result.feasible
                result.message="Joint convex base failed: "+result.message;return;
            end
            point=result.decision;
            [baseSafe,base]=localVerifyBasePoint(program,point);
            if ~baseSafe
                result.feasible=false;result.message="Joint convex base failed independent verification.";return;
            end
            program.safetyBound=base.safetyBound;program.terminalCone=base.terminalCone;
            program=localRefreshBounds(program);
        end
        phase=tic;angles=avoidanceSafetyGeometry.admissionAngles(program,point);
        search.formulationSeconds=search.formulationSeconds+toc(phase);
        search.initialCertificateAngles=angles;search.familyAttempts=1;
        values=avoidanceSafetyGeometry.jointResidual(program,point,angles);
        violation=max([0;values-program.jointCertificate.upperBound]);history=violation;
        for iteration=1:(cfg.jointCertificate.maximumAdmissionSolves-search.nativeSolves)
            [admitted,accepted]=localVerifyJointPoint(program,point,angles);
            if admitted,program=localRefreshBounds(accepted);break;end
            if ~localJointTimeAvailable(cfg),break;end
            phase=tic;conic=avoidanceStageQp.joint(program,point,angles,true,violation,cfg);
            search.formulationSeconds=search.formulationSeconds+toc(phase);
            phase=tic;trial=localSolveConic(conic,cfg);
            search.solveSeconds=search.solveSeconds+toc(phase);
            search.restorationSolves=search.restorationSolves+1;search.nativeSolves=search.nativeSolves+1;
            if ~trial.feasible,break;end
            candidate=trial.decision(1:conic.primaryCount);
            [baseSafe,base]=localVerifyBasePoint(program,candidate);
            if ~baseSafe,break;end
            nextAngles=angles+atan(trial.decision(conic.angleIndex));
            values=avoidanceSafetyGeometry.jointResidual(program,candidate,nextAngles);
            nextViolation=max([0;values-program.jointCertificate.upperBound]);
            if ~isfinite(nextViolation) || nextViolation>violation+128*eps*(1+violation),break;end
            program.safetyBound=base.safetyBound;program.terminalCone=base.terminalCone;
            program=localRefreshBounds(program);
            point=candidate;angles=nextAngles;history(end+1)=nextViolation; %#ok<AGROW>
            [admitted,accepted]=localVerifyJointPoint(program,point,angles);
            if admitted,program=localRefreshBounds(accepted);break;end
            if violation-nextViolation<=cfg.jointCertificate.stallTolerance*(1+violation),break;end
            violation=nextViolation;
        end
        search.violationHistory={history};
    end
    if ~admitted
        result=localEmptySolve();
        result.message="Bounded admission found no hard-certified plan in its selected local family (stall, solve or time limit).";
        return;
    end
    if search.nativeSolves>0
        % Original nonlinear supports and every hard constraint were checked.
        % A second solve is unnecessary for safety; performance is improved
        % on the next frame within this accepted certificate family.
        program=accepted;
        if search.restorationSolves>0,result=trial;end
        result.decision=point;
        result.message="Issued an independently verified admission witness.";
        search.issuedAdmissionWitness=true;
    else
        program.jointCertificate.angles=angles;incumbent=point;
        phase=tic;conic=avoidanceStageQp.joint(program,point,angles,false,0,cfg);
        search.formulationSeconds=search.formulationSeconds+toc(phase);
        phase=tic;trial=localSolveConic(conic,cfg);
        search.solveSeconds=search.solveSeconds+toc(phase);
        search.hardSolves=1;search.nativeSolves=search.nativeSolves+1;
        improved=false;
        if trial.feasible
            point=trial.decision(1:conic.primaryCount);
            nextAngles=angles+atan(trial.decision(conic.angleIndex));
            [improved,accepted]=localVerifyJointPoint(program,point,nextAngles);
        end
        if improved
            program=accepted;result=trial;result.decision=point;
        else
            result=localEmptySolve();result.decision=incumbent;result.feasible=true;
            result.exitFlag=trial.exitFlag;result.message="Retained verified certificate; improvement: "+trial.message;
            search.usedCertifiedIncumbent=true;
        end
    end
    program.supportGeometry.witnessPreserved=program.inheritedPredictionFamily;
    program.inheritedFeasibleFamily=program.inheritedPredictionFamily;
end

function result=localSolveConic(program,cfg)
    problem=struct('layout',struct('decisionCount',numel(program.q)),'stageProgram',program);
    result=localRunJointProgram(problem,cfg);
end

function [accepted,candidate]=localVerifyJointPoint(program,point,angles)
    candidate=program;candidate.jointCertificate.angles=angles;accepted=false;
    try
        candidate=solveHardCbfClf.certify(candidate,point);accepted=true;
    catch exception
        if ~strcmp(exception.identifier,'collisionAvoidanceController:optimizationFailed'),rethrow(exception);end
    end
end

function [accepted,candidate]=localVerifyBasePoint(program,point)
    candidate=rmfield(program,'jointCertificate');accepted=false;
    try
        candidate=solveHardCbfClf.certify(candidate,point);accepted=true;
    catch exception
        if ~strcmp(exception.identifier,'collisionAvoidanceController:optimizationFailed'),rethrow(exception);end
    end
end

function program=localRefreshBounds(program)
    program.b(1:numel(program.safetyBound))=program.safetyBound;
    program.b(end-numel(program.terminalCone.bound)+1:end)=program.terminalCone.bound;
end

function available=localJointTimeAvailable(cfg)
    available=~isfield(cfg.solver,'workTimer') || ~isfinite(cfg.solver.workTimeLimit) ...
        || toc(cfg.solver.workTimer)<cfg.solver.workTimeLimit;
end

function contained=localConesContain(value,cones)
    equality=cones(1);cursor=equality+cones(2);
    contained=all(isfinite(value)) && all(value(1:equality)==0) && all(value(equality+1:cursor)>=0);
    for size=cones(3:end).'
        block=value(cursor+(1:size));contained=contained && block(1)>=norm(block(2:end));cursor=cursor+size;
    end
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
