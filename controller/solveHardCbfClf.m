classdef solveHardCbfClf
    %solveHardCbfClf Finite convex admission branches and certified SOCP solves.
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
            problem = struct('layout',struct('decisionCount',numel(program.q)),'stageProgram',program);
            solve = localRunJointProgram(problem,cfg);
        end
        function family = buildBranches(model,prediction,program)
            count=model.cfg.encounter.separationDirectionCount;
            angle=2*pi*(0:count-1)/count;
            directions=[cos(angle);sin(angle)];
            directions(abs(directions)<32*eps)=0;
            directions=directions./vecnorm(directions);
            targets=numel(model.encounters);cells=numel(prediction.cells);
            groups=cell((cells+1)*targets,1);
            template=struct('matrix',[],'physicalBound',[],'bound',[], ...
                'label',"",'stage',0,'cellIndex',0,'targetIndex',0, ...
                'direction',zeros(2,1),'stateRow',[],'stateBound',[]);
            for group=1:numel(groups),groups{group}=repmat(template,1,count);end
            prediction.geometryAnchor=model.anchorPlan;
            [prediction.geometryFrames,prediction.geometryNominal]= ...
                laneGeometry.sweptCellFrames(model,prediction.cells,model.anchorPlan);
            for direction=1:count
                normal=directions(:,direction);
                prediction.separationNormals=repmat({repmat(normal,1,targets)},cells,1);
                geometry=avoidanceSafetyGeometry.build(model,prediction);
                for cellIndex=1:cells
                    for target=1:targets
                        group=(cellIndex-1)*targets+target;
                        label="collision:"+model.encounters(target).key;
                        keep=geometry.cellIndex==cellIndex & geometry.label==label;
                        option=template;
                        option.matrix=geometry.matrix(keep,:);
                        option.physicalBound=geometry.physicalBound(keep);
                        option.label=label;option.stage=prediction.cells(cellIndex).stage;
                        option.cellIndex=cellIndex;option.targetIndex=target;option.direction=normal;
                        option.bound=localTighten(option,program.decisionRadius,model.cfg);
                        groups{group}(direction)=option;
                    end
                end
                trial=model;trial.completionDirections=repmat(normal,1,targets);
                [matrix,bound,~,completion]=hardEncounterBarrier.completionRows(trial,prediction,geometry);
                for target=1:targets
                    option=template;row=size(matrix,1)-targets+target;
                    option.matrix=matrix(row,:);option.physicalBound=bound(row);
                    option.label="exit:"+model.encounters(target).key;
                    option.stage=prediction.stageCount;option.targetIndex=target;option.direction=normal;
                    option.stateRow=completion.stateRow(target,:);option.stateBound=completion.stateBound(target);
                    option.bound=localTighten(option,program.decisionRadius,model.cfg);
                    groups{cells*targets+target}(direction)=option;
                end
            end
            family=struct('groups',{groups},'directions',directions, ...
                'model',model,'prediction',prediction, ...
                'log10AssignmentCount',numel(groups)*log10(count), ...
                'scope',"complete finite direction assignments for the declared swept polyhedral inner approximation");
        end

        function [program,result,information] = branches(program,cfg)
            information=struct('conicCalls',0,'integerCalls',0,'integerNodes',0, ...
                'testedAssignments',0,'exhausted',false,'status',"singleConvexProgram", ...
                'log10AssignmentCount',0,'selectedDirections',zeros(0,1),'activeGroups',0, ...
                'integerSeconds',0);
            if isfield(program,'admissionGeometry')
                context=program.admissionGeometry;program=rmfield(program,'admissionGeometry');
                if program.dualConvexification.used
                    result=solveHardCbfClf.constrained(program,cfg);
                    information.conicCalls=information.conicCalls+1;
                    if result.feasible
                        information.status="distanceDualFeasible";return;
                    elseif result.exitFlag~=-2
                        information.status="searchIncomplete";return;
                    end
                end
                program.dualConvexification.used=false;
                if isfield(cfg.solver,'workTimer') && toc(cfg.solver.workTimer)>=cfg.solver.workTimeLimit
                    result=localFailure(0,"The distance-dual initialization reached the frame deadline.");
                    information.status="searchIncomplete";return;
                end
                program.branchFamily=solveHardCbfClf.buildBranches(context.model,context.prediction,program);
            end
            if ~isfield(program,'branchFamily')
                result=solveHardCbfClf.constrained(program,cfg);information.conicCalls=information.conicCalls+1;
                return;
            end
            family=program.branchFamily;program=rmfield(program,'branchFamily');
            base=localCommonProgram(program);
            information.log10AssignmentCount=family.log10AssignmentCount;
            result=solveHardCbfClf.constrained(base,cfg);information.conicCalls=information.conicCalls+1;
            if ~result.feasible
                information.exhausted=result.exitFlag==-2;
                information.status="searchIncomplete";
                if information.exhausted,information.status="commonConstraintsInfeasible";end
                return;
            end
            selection=localSatisfied(family,result.decision(1:end-1));
            if all(selection>0)
                program=localSelect(base,family,selection);
                information.status="feasible";information.selectedDirections=selection;return;
            end
            active=localViolatedGroup(family,result.decision(1:end-1));
            rejected=zeros(0,numel(family.groups));
            outerCuts=zeros(0,base.layout.planCount);outerBounds=zeros(0,1);
            while true
                remaining=Inf;
                if isfield(cfg.solver,'workTimer')
                    remaining=cfg.solver.workTimeLimit-toc(cfg.solver.workTimer);
                end
                if remaining<=0
                    result=localFailure(0,"Finite branch search reached the frame deadline.");
                    information.status="searchIncomplete";return;
                end
                [linear,bound,equality,equalityBound,lower,upper,indices,owner]= ...
                    localIntegerProgram(base,family,active);
                cuts=[outerCuts,sparse(size(outerCuts,1),numel(indices))];cutBounds=outerBounds;
                for excluded=1:size(rejected,1)
                    chosen=false(size(owner,1),1);
                    for index=1:size(owner,1)
                        chosen(index)=rejected(excluded,owner(index,1))==owner(index,2);
                    end
                    cut=sparse(ones(nnz(chosen),1),indices(chosen),ones(nnz(chosen),1),1,numel(lower));
                    cuts=[cuts;cut];cutBounds=[cutBounds;numel(family.groups)-1]; %#ok<AGROW>
                end
                if isfield(cfg.solver,'workTimer')
                    remaining=cfg.solver.workTimeLimit-toc(cfg.solver.workTimer);
                    if remaining<=0
                        result=localFailure(0,"Finite branch construction reached the frame deadline.");
                        information.status="searchIncomplete";return;
                    end
                end
                options=optimoptions('intlinprog','Display','off','MaxTime',remaining, ...
                    'ConstraintTolerance',cfg.solver.constraintTolerance,'MaxFeasiblePoints',1);
                % Rank feasible initialization toward forward path progress.
                % This changes no branch constraint or trajectory SOCP cost.
                objective=zeros(numel(lower),1);
                if isfield(family,'prediction')
                    station=family.prediction.egoStateMatrix(1,:,end).';
                    objective(1:numel(station))=-station/max(1,norm(station,inf));
                end
                integerTimer=tic;
                [candidate,~,flag,output]=intlinprog(objective,indices, ...
                    [linear;cuts],[bound;cutBounds],equality,equalityBound,lower,upper,options);
                information.integerSeconds=information.integerSeconds+toc(integerTimer);
                information.activeGroups=numel(active);
                information.integerCalls=information.integerCalls+1;
                information.integerNodes=information.integerNodes+output.numnodes;
                if flag==-2
                    result=localFailure(-2,"All finite branches are infeasible within the declared approximation.");
                    information.exhausted=true;information.status="finiteFamilyInfeasible";return;
                elseif isempty(candidate) || ~any(flag==[1,2])
                    result=localFailure(0,"Finite branch search stopped before a complete feasible assignment was established.");
                    information.status="searchIncomplete";return;
                end
                input=candidate(1:base.layout.planCount);
                selection=localSatisfied(family,input);
                for index=1:size(owner,1)
                    if candidate(indices(index))>.5
                        selection(owner(index,1))=owner(index,2);
                    end
                end
                if any(selection==0)
                    next=localViolatedGroup(family,input,active);
                    if isempty(next)
                        result=localFailure(0,"The integer search returned an incomplete assignment.");
                        information.status="searchIncomplete";return;
                    end
                    active=sort([active;next]);
                    continue;
                end
                trial=localSelect(base,family,selection);
                % This is a computed feasible-branch candidate used only as
                % numerical coordinates; it adds no trajectory constraints.
                trial.anchorPlan=input;
                result=solveHardCbfClf.constrained(trial,cfg);
                information.conicCalls=information.conicCalls+1;
                information.testedAssignments=information.testedAssignments+1;
                if result.feasible
                    program=trial;information.status="feasible";
                    information.selectedDirections=selection;return;
                elseif result.exitFlag~=-2
                    information.status="searchIncomplete";return;
                end
                % A no-good cut concerns a COMPLETE assignment. Activate all
                % groups before excluding it, so other unexamined directions
                % of previously lazy groups can never be pruned accidentally.
                active=(1:numel(family.groups)).';
                rejected(end+1,:)=selection.'; %#ok<AGROW>
                [outer,outerBound]=localTerminalCuts(base,input);
                outerCuts=[outerCuts;outer];outerBounds=[outerBounds;outerBound]; %#ok<AGROW>
            end
        end
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

function bound=localTighten(option,radius,cfg)
    scale=1+abs(option.physicalBound)+abs(option.matrix)*radius;
    bound=option.physicalBound-4*max(cfg.encounter.numericalMargin,cfg.solver.constraintTolerance) ...
        *scale.*any(option.matrix~=0,2);
end

function program=localCommonProgram(program)
    remove=startsWith(program.physicalLabels,"collision:") | startsWith(program.physicalLabels,"exit:");
    keep=~remove;
    linear=program.cones(2);
    program.A=[program.A([keep;true],:);program.A(linear+1:end,:)];
    program.b=[program.b([keep;true]);program.b(linear+1:end)];
    program.cones(2)=nnz(keep)+1;
    program.physicalMatrix=program.physicalMatrix(keep,:);
    program.physicalBound=program.physicalBound(keep);
    program.safetyBound=program.safetyBound(keep);program.physicalLabels=program.physicalLabels(keep);
    geometry=program.geometry;keep=~startsWith(geometry.label,"collision:");
    geometry.matrix=geometry.matrix(keep,:);geometry.physicalBound=geometry.physicalBound(keep);
    geometry.label=geometry.label(keep);geometry.stage=geometry.stage(keep);
    geometry.safety=geometry.safety(keep);geometry.cellIndex=geometry.cellIndex(keep);
    program.geometry=geometry;program.obstacleCbfRowCount=0;
end

function selection=localSatisfied(family,input)
    selection=zeros(numel(family.groups),1);
    for group=1:numel(family.groups)
        options=family.groups{group};
        for index=1:numel(options)
            option=options(index);
            allowance=64*numel(input)*eps*(1+abs(option.bound)+abs(option.matrix)*abs(input));
            if all(option.matrix*input+allowance<=option.bound)
                selection(group)=index;break;
            end
        end
    end
end

function program=localSelect(program,family,selection)
    selected=cell(numel(selection),1);
    for group=1:numel(selection),selected{group}=family.groups{group}(selection(group));end
    selected=vertcat(selected{:});
    matrix=vertcat(selected.matrix);physical=vertcat(selected.physicalBound);bound=vertcat(selected.bound);
    labels=strings(size(bound));cursor=0;geometry=program.geometry;
    for group=1:numel(selected)
        option=selected(group);count=numel(option.bound);
        labels(cursor+(1:count))=option.label;cursor=cursor+count;
        target=option.targetIndex;
        if option.cellIndex>0
            geometry.matrix=[geometry.matrix;option.matrix];
            geometry.physicalBound=[geometry.physicalBound;option.physicalBound];
            geometry.label=[geometry.label;repmat(option.label,count,1)];
            geometry.stage=[geometry.stage;repmat(option.stage,count,1)];
            geometry.safety=[geometry.safety;true(count,1)];
            geometry.cellIndex=[geometry.cellIndex;repmat(option.cellIndex,count,1)];
            geometry.normals{option.cellIndex}(:,target)=option.direction;
        else
            program.completion.direction(:,target)=option.direction;
            program.completion.stateRow(target,:)=option.stateRow;
            program.completion.stateBound(target)=option.stateBound;
        end
    end
    linear=program.cones(2);
    program.A=[program.A(1:linear,:);matrix,zeros(size(matrix,1),1);program.A(linear+1:end,:)];
    program.b=[program.b(1:linear);bound;program.b(linear+1:end)];
    program.cones(2)=linear+numel(bound);
    program.physicalMatrix=[program.physicalMatrix;matrix,zeros(size(matrix,1),1)];
    program.physicalBound=[program.physicalBound;physical];
    program.safetyBound=[program.safetyBound;bound];program.physicalLabels=[program.physicalLabels;labels];
    if isfield(family,'model')
        % Keep local and condensed geometry consistent for diagnostic users.
        prediction=family.prediction;prediction.separationNormals=geometry.normals;
        geometry=avoidanceSafetyGeometry.build(family.model,prediction);
    end
    program.geometry=geometry;program.obstacleCbfRowCount=nnz(startsWith(geometry.label,"collision:"));
end

function [matrix,bound,equality,equalityBound,lower,upper,indices,owner]=localIntegerProgram(program,family,active)
    count=program.layout.planCount;radius=program.decisionRadius;
    binaries=sum(cellfun(@numel,family.groups(active)));indices=count+(1:binaries);
    lower=[-radius;zeros(binaries,1)];upper=[radius;ones(binaries,1)];
    matrices=cell(binaries+2,1);bounds=cell(binaries+2,1);
    matrices{1}=[program.A(1:program.cones(2),1:count),sparse(program.cones(2),binaries)];
    bounds{1}=program.b(1:program.cones(2));
    [outer,outerBound]=localTerminalCuts(program,[]);
    matrices{2}=[outer,sparse(size(outer,1),binaries)];bounds{2}=outerBound;
    equalityBound=ones(numel(active),1);equalityRows=zeros(binaries,1);
    owner=zeros(binaries,2);cursor=0;
    for activeIndex=1:numel(active)
        group=active(activeIndex);options=family.groups{group};
        for branch=1:numel(options)
            option=options(branch);cursor=cursor+1;owner(cursor,:)=[group,branch];
            maximum=abs(option.matrix)*radius;
            reserve=128*eps*(1+abs(maximum)+abs(option.bound));
            bigM=max(0,maximum-option.bound+reserve);
            integers=sparse(1:numel(bigM),repmat(cursor,numel(bigM),1),bigM,numel(bigM),binaries);
            matrices{cursor+2}=[sparse(option.matrix),integers];bounds{cursor+2}=option.bound+bigM;
            equalityRows(cursor)=activeIndex;
        end
    end
    equality=sparse(equalityRows,indices,ones(binaries,1),numel(active),count+binaries);
    matrix=vertcat(matrices{:});bound=vertcat(bounds{:});
end

function [matrix,bound]=localTerminalCuts(program,input)
    matrix=zeros(0,program.layout.planCount);bound=zeros(0,1);
    for first=1:3:numel(program.terminalCone.bound)
        rows=first+(0:2);a=program.terminalCone.matrix(rows,:);b=program.terminalCone.bound(rows);
        if isempty(input)
            angles=2*pi*(0:31)/32;directions=[cos(angles);sin(angles)];
        else
            value=b-a*input;directions=value(2:3)/max(norm(value(2:3)),realmin);
        end
        cut=a(1,:)-directions.'*a(2:3,:);limit=b(1)-directions.'*b(2:3);
        allowance=128*eps*(1+abs(limit)+abs(cut)*program.decisionRadius);
        matrix=[matrix;cut];bound=[bound;limit+allowance]; %#ok<AGROW>
    end
end

function result=localFailure(flag,message)
    result=struct('feasible',false,'exitFlag',flag,'decision',[], ...
        'message',message,'output',struct('message',message));
end

function group=localViolatedGroup(family,input,active)
    if nargin<3,active=[];end
    residual=-inf(numel(family.groups),1);
    for index=1:numel(family.groups)
        if ismember(index,active),continue;end
        options=family.groups{index};best=Inf;
        for branch=1:numel(options)
            option=options(branch);
            violation=max(option.matrix*input-option.bound);
            best=min(best,violation);
        end
        residual(index)=best;
    end
    [value,group]=max(residual);
    if ~isfinite(value),group=[];end
end
