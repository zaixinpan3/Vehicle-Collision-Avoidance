classdef solveHardCbfClf
    %solveHardCbfClf Convex trajectory solves and hard-safety checks of carried witnesses.
    methods (Static)
        function reference = vffmReference(time,progress,target,lane,road,width,passingOffsets)
        % Time-consistent Gaussian preference in a regular normal road chart.
        % progress is [station; station rate; station acceleration] in SI units.
        % Each column of passingOffsets specifies fixed lateral passing ordinates
        % in metres for one obstacle. Its Gaussian centre and chart width vary with time.
        % This geometric reference has no execution or feasibility authority.
            validateattributes(time,{'double'},{'row','finite','nonnegative'});
            validateattributes(progress,{'double'},{'size',[3,numel(time)],'finite','real'});
            validateattributes(width,{'double'},{'scalar','positive','finite','real'});
            validateattributes(passingOffsets,{'double'},{'row','finite','real'});
            reference=localTimeDependentReference(time,progress,target,lane,road,width,passingOffsets);
        end

        function reference = prepareFluidReference(program,model)
        % Prepare numeric geometry once; native frame replay uses the same data.
            reference=localPrepareFluidReference(program,model);
        end

        function program = certify(program,decision)
        % Validate physical safety independently of a numerical success flag.
        % Transfer a feasible affine family WITHOUT repeated inward tightening.
        % The controller applies it to carried witnesses before a solve, never
        % to a solver-accepted plan.
            [status,certified,adjusted]=solveHardCbfClf.inspect(program,decision);
            tops=1:3:numel(adjusted);
            if status==1
                [excess,row]=max(certified-program.physicalBound);
                error('collisionAvoidanceController:optimizationFailed', ...
                    ['The returned solution failed the hard-safety certificate ' ...
                    '(linear excess %.9g at %s, terminal excess %.9g). No command was issued.'], ...
                    excess,program.physicalLabels(row), ...
                    max(adjusted(tops)-program.terminalConePhysicalBound(tops)));
            end
            if status==2
                error('collisionAvoidanceController:optimizationFailed', ...
                    'The returned solution failed the reserved soft-CLF inequality. No command was issued.');
            end
            program.safetyBound=certified;
            program.terminalCone.bound=adjusted;
            if isfield(program,'jointCertificate')
                program=avoidanceSafetyGeometry.certifyJoint(program,decision);
            end
        end

        function [status,certified,adjusted,clf] = inspect(program,decision)
        % Numeric hard-row, terminal and soft-CLF check shared by native builds.
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
            for first=1:3:numel(cone)
                adjusted(first)=adjusted(first)+max(0,norm(cone(first+(1:2))) ...
                    +norm(coneAllowance(first+(0:2)))-cone(first));
            end
            first=program.cones(2)+1;
            clf=program.b(first:first+5)-program.A(first:first+5,:)*decision;
            status=0;
            if any(~isfinite(certified)) || any(certified>program.physicalBound) ...
                    || any(~isfinite(adjusted)) || any(adjusted(tops)>program.terminalConePhysicalBound(tops))
                status=1;
            elseif norm(clf(2:end))-clf(1)>program.clfNumericalReserve ...
                    || decision(end)<-program.clfNumericalReserve || any(~isfinite(clf))
                status=2;
            end
        end

        function [reduced,retained] = reduce(program)
        % Preserve the exact row/column reduction in generated solver adapters.
            [reduced,retained]=localReducedProgram(program);
        end

        function [program,result,search] = fixedDirections(program,model,cfg)
        % A solver-accepted plan, or on solver failure the admitted inherited
        % incumbent, leaves this method. An initializer is never issued.
            [program,result,search]=localFixedDirectionSearch(program,model,cfg);
        end

        function solve = constrained(program,cfg)
            program = localCompactPlanarRows(program);
            decisionCount=numel(program.q);
            program=avoidanceStageQp.build(program);
            problem = struct('layout',struct('decisionCount',decisionCount),'stageProgram',program);
            solve = localRunJointProgram(problem,cfg);
        end

        function [decision,angles,information] = fluidInitialize(program,cfg)
        % Cheng et al. (2021), DOI 10.1109/TITS.2020.2990211, Eqs. (34)-(36).
        % Timed NRMM/VFFM references initialize the geometry of one full SOCP.
        % Moving targets, quadratic charts and affine-model fitting are extensions;
        % this seed is neither a feasible certificate nor an executable plan.
            [decision,angles,information]=localFluidInitialize(program,cfg);
        end

    end
end

function [program,result,search]=localFixedDirectionSearch(program,~,cfg)
% Direction search supplies an optimizer center, never a new issued plan.
% Only an inherited, previously optimized certificate can survive solve failure.
    search=struct('hardSolves',0,'restorationSolves',0,'baseSolves',0,'nativeSolves',0, ...
        'familyAttempts',0,'horizonAttempts',1,'formulationSeconds',0,'solveSeconds',0, ...
        'initialOverlappingNodes',program.supportGeometry.overlappingNodes, ...
        'policy',"fixedDirectionTrajectoryOptimization",'violationHistory',{{}},'usedCertifiedIncumbent',false, ...
        'issuedAdmissionWitness',false,'usedFullPlanAdmission',false, ...
        'fullPlanStatus',"notAttempted", ...
        'initialCertificateAngles',program.jointCertificate.angles, ...
        'fixedCertificateAngles',program.jointCertificate.angles,'directionSeedSource',"nominalWitness");
    point=program.feasibleWitness;angles=program.jointCertificate.angles;
    inherited=program.inheritedPredictionFamily;
    [admitted,accepted]=localVerifyJointPoint(program,point,angles);
    if inherited && ~admitted
        result=localEmptySolve();result.message="The inherited witness failed independent verification.";return;
    end
    if ~inherited && ~admitted
        phase=tic;
        [point,angles,search.initialization]=solveHardCbfClf.fluidInitialize(program,cfg);
        search.formulationSeconds=toc(phase);search.familyAttempts=1;
        search.directionSeedSource="chengFluidReference";
        if isempty(point)
            result=localEmptySolve();
            result.message="Fluid initialization failed: "+search.initialization.status;return;
        end
    elseif admitted
        program=localRefreshBounds(accepted);
        if inherited,search.directionSeedSource="inheritedWitness";end
    end
    search.fixedCertificateAngles=angles;
    incumbent=point;
    phase=tic;conic=avoidanceStageQp.fixedDirections(program,point,angles,cfg);
    search.formulationSeconds=search.formulationSeconds+toc(phase);
    phase=tic;trial=localSolveConic(conic,cfg);search.solveSeconds=toc(phase);
    search.hardSolves=1;search.nativeSolves=1;improved=trial.feasible;
    search.fullPlanStatus="solverRejected";
    if improved
        % A solver-reported success is accepted as returned; the plan is not
        % re-verified after the solve. It carries exactly the normals selected above.
        program=avoidanceSafetyGeometry.setDirections(program,angles);
        result=trial;result.decision=trial.decision(1:conic.primaryCount);
        search.usedFullPlanAdmission=~inherited;search.fullPlanStatus="accepted";
    elseif inherited
        result=localEmptySolve();result.decision=incumbent;result.feasible=true;
        result.exitFlag=trial.exitFlag;result.message="Retained previously optimized certificate: "+trial.message;
        search.usedCertifiedIncumbent=true;
    else
        result=localEmptySolve();
        result.message="Fixed-direction trajectory optimization failed: "+search.fullPlanStatus+": "+trial.message;
        return;
    end
    program.supportGeometry.witnessPreserved=inherited;
    program.inheritedFeasibleFamily=inherited;
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

function program=localRefreshBounds(program)
    program.b(1:numel(program.safetyBound))=program.safetyBound;
    program.b(end-numel(program.terminalCone.bound)+1:end)=program.terminalCone.bound;
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
    columns=find(retained);
    matrix = program.A(:,columns);
    equalities = program.cones(1);
    linearRows = equalities+program.cones(2);
    rowNorm = full(sum(abs(matrix(1:linearRows,:)),2));
    trivialEquality = (1:equalities).' <= equalities & rowNorm(1:equalities)==0 & program.b(1:equalities)==0;
    trivialInequality = rowNorm(equalities+1:linearRows)==0 & program.b(equalities+1:linearRows)>=0;
    keep = true(size(matrix,1),1);
    keep(1:equalities) = ~trivialEquality;
    keep(equalities+1:linearRows) = ~trivialInequality;
    reduced = program;
    reduced.P = program.P(columns,columns);
    reduced.q = program.q(retained);
    reduced.A = matrix(find(keep),:);
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
    % Solved (1) and reduced-accuracy AlmostSolved (2) count as success and
    % the returned plan is issued without re-verification. Infeasibility,
    % iteration limits, timeouts and malformed decisions issue no command.
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

function states=localStates(prediction,inputs)
    count=prediction.stageCount;states=zeros(6,count+1);
    states(:,1)=prediction.egoStateOffset(:,1);
    for stage=1:count
        states(:,stage+1)=prediction.stageMatrixA(:,:,stage)*states(:,stage) ...
            +prediction.stageMatrixB(:,:,stage)*inputs(2*stage-1:2*stage)+prediction.stageAffine(:,stage);
    end
end

function reference=localTimeDependentReference(time,progress,target,lane,road,width,passingOffsets)
    chart=laneGeometry.normalRoadChart(progress(1,:),lane,road);
    candidates=size(passingOffsets,2);nodes=numel(time);
    z=zeros(candidates,nodes);zd=z;zdd=z;valid=all(chart.valid);
    targetStation=zeros(1,nodes);targetLateral=targetStation;
    coefficients=zeros(candidates,nodes);
    if ~isempty(target)
        motion=targetPrediction.nominalFlow(target,time);
        projection=laneGeometry.project(motion(1:2,:),lane,progress(1,:));
        s=projection.station;d=projection.lateralPosition;
        targetChart=laneGeometry.normalRoadChart(s,lane,road);
        k=targetChart.curvature;kp=targetChart.curvatureDerivative;regular=1-k.*d;
        valid=valid && all(targetChart.valid) && all(regular>sqrt(eps));
        if valid
            sd=sum(targetChart.tangent.*motion(3:4,:),1)./regular;
            dd=sum(targetChart.normal.*motion(3:4,:),1);
            sdd=(sum(targetChart.tangent.*motion(5:6,:),1)+2*k.*sd.*dd+kp.*d.*sd.^2)./regular;
            m=targetChart.midpoint;h=targetChart.halfWidth;
            hd=h(2,:).*sd;hdd=h(3,:).*sd.^2+h(2,:).*sdd;
            numerator=passingOffsets.'-m(1,:);
            numeratorD=-m(2,:).*sd;
            numeratorDD=-m(3,:).*sd.^2-m(2,:).*sdd;
            b=numerator./h(1,:);bd=(numeratorD-b.*hd)./h(1,:);
            bdd=(numeratorDD-b.*hdd-2*bd.*hd)./h(1,:);
            q=progress(1,:)-s;qd=progress(2,:)-sd;qdd=progress(3,:)-sdd;
            exponent=exp(-.5*(q/width).^2);
            lambda=-q.*qd/width^2;
            lambdaD=-(qd.^2+q.*qdd)/width^2;
            z=b.*exponent;zd=(bd+b.*lambda).*exponent;
            zdd=(bdd+2*bd.*lambda+b.*(lambda.^2+lambdaD)).*exponent;
            targetStation(1,:)=s;targetLateral(1,:)=d;
            coefficients=b;
        end
    end
    m=chart.midpoint;h=chart.halfWidth;sd=progress(2,:);sdd=progress(3,:);
    d=m(1,:)+h(1,:).*z;
    dd=(m(2,:)+h(2,:).*z).*sd+h(1,:).*zd;
    ddd=(m(3,:)+h(3,:).*z).*sd.^2+(m(2,:)+h(2,:).*z).*sdd ...
        +2*h(2,:).*sd.*zd+h(1,:).*zdd;
    regular=1-chart.curvature.*d;
    tangential=regular.*sd;
    at=regular.*sdd-chart.curvatureDerivative.*d.*sd.^2-2*chart.curvature.*sd.*dd;
    an=chart.curvature.*regular.*sd.^2+ddd;
    speed=hypot(tangential,dd);course=atan2(dd,tangential);
    curvature=(tangential.*an-dd.*at)./max(speed,sqrt(eps)).^3;
    positions=zeros(2,nodes,candidates);velocity=positions;acceleration=positions;
    for candidate=1:candidates
        positions(:,:,candidate)=chart.position+chart.normal.*d(candidate,:);
        velocity(:,:,candidate)=chart.tangent.*tangential(candidate,:)+chart.normal.*dd(candidate,:);
        acceleration(:,:,candidate)=chart.tangent.*at(candidate,:)+chart.normal.*an(candidate,:);
    end
    reference=struct('lateralPosition',d,'lateralRate',dd,'lateralAcceleration',ddd, ...
        'courseOffset',course,'speed',speed,'curvature',curvature,'normalAcceleration',speed.^2.*curvature, ...
        'position',positions,'velocity',velocity,'acceleration',acceleration, ...
        'targetStation',targetStation,'targetLateral',targetLateral,'coefficients',coefficients, ...
        'midpoint',m,'halfWidth',h,'bounded',chart.bounded, ...
        'valid',valid & all(regular>sqrt(eps) & speed>sqrt(eps),2).');
end

function reference=localPrepareFluidReference(program,model)
    count=program.prediction.stageCount;nodes=count+1;
    reference=struct('bump',zeros(2,nodes),'heading',zeros(2,nodes),'valid',true(1,2), ...
        'amplitudes',zeros(1,2),'width',0,'center',0,'stages',zeros(1,2), ...
        'active',false, ...
        'targetStation',zeros(0,nodes),'targetLateral',zeros(0,nodes), ...
        'lateralRate',zeros(2,nodes),'lateralAcceleration',zeros(2,nodes), ...
        'normalAcceleration',zeros(2,nodes),'bounded',false(1,nodes));
    if program.inheritedPredictionFamily || isempty(model.encounter),return;end
    records=program.jointCertificate.records;
    values=avoidanceSafetyGeometry.jointResidual(program,program.feasibleWitness,program.jointCertificate.angles) ...
        -program.jointCertificate.upperBound;
    values([records.isExit])=-Inf;
    nominal=localStates(program.prediction,program.anchorPlan);
    time=(0:count)*model.sampleTime;progress=zeros(3,nodes);progress(1,:)=nominal(1,:);
    for node=1:nodes
        stage=min(node,count);a=program.prediction.continuousA(:,:,stage);
        velocity=a*nominal(:,node)+program.prediction.continuousB(:,:,stage) ...
            *program.anchorPlan(2*stage-1:2*stage)+program.prediction.continuousC(:,stage);
        acceleration=a*velocity;progress(2:3,node)=[velocity(1);acceleration(1)];
    end
    target=model.encounter;
    conflicts=find(values>0);
    if isempty(conflicts),return;end
    stages=[records(conflicts).stage];first=min(stages);last=max(stages);
    middle=round((first+last)/2)+1;
    motion=targetPrediction.nominalFlow(target,time);
    projection=laneGeometry.project(motion(1:2,:),model.lane,progress(1,:));
    q=progress(1,:)-projection.station;
    width=model.cfg.admission.widthScale*max(model.cfg.admission.minimumWidthMeters, ...
        (max(q(first+1:last+1))-min(q(first+1:last+1)))/2);
    transverse=projection.lateralPosition(last+1)-projection.lateralPosition(first+1);
    side=-sign(transverse);
    if abs(transverse)<=1e-6,side=sign(nominal(2,middle)-projection.lateralPosition(middle));end
    if side==0,side=1;end
    normal=[-sin(projection.heading(middle));cos(projection.heading(middle))];
    allowance=model.cfg.vehicle.width/2+targetPrediction.rectangleSupport( ...
        target.halfLength,target.halfWidth,normal,motion(7,middle),0) ...
        +model.cfg.admission.clearanceAllowanceMeters;
    passingOffsets=projection.lateralPosition(middle)+[side,-side]*allowance;
    reference.amplitudes=passingOffsets;
    reference.width=width;reference.center=projection.station(middle);reference.stages=[first,last];
    field=solveHardCbfClf.vffmReference(time,progress,target,model.lane,model.road,width,passingOffsets);
    reference.bump=field.lateralPosition-nominal(2,:);
    % Course minus the anchor sideslip gives a body-yaw preference. The fit
    % and subsequent full trajectory solve determine the actual lateral state.
    heading=field.courseOffset-atan2(nominal(5,:),nominal(4,:))-nominal(3,:);
    reference.heading=atan2(sin(heading),cos(heading));reference.valid=field.valid;
    reference.active=true;
    reference.targetStation=field.targetStation;reference.targetLateral=field.targetLateral;
    reference.lateralRate=field.lateralRate;reference.lateralAcceleration=field.lateralAcceleration;
    reference.normalAcceleration=field.normalAcceleration;reference.bounded=field.bounded;
end

function [point,angles,information]=localFluidInitialize(program,cfg)
    point=zeros(0,1);angles=program.jointCertificate.angles;
    coder.varsize('point',[Inf,1],[true,false]);
    information=struct('status',"nominal",'amplitudeMeters',0,'widthMeters',0, ...
        'centerStationMeters',0,'conflictStages',zeros(1,2),'terminalFitError',0, ...
        'maximumPhysicalExcess',NaN,'maximumSupportResidual',NaN, ...
        'referenceCount',0,'referenceScores',Inf(1,2),'referencePhysicalExcess',Inf(1,2),'activeTarget',false);
    if ~localInitializationTimeAvailable(cfg),information.status="searchTimeLimit";return;end
    records=program.jointCertificate.records;
    reference=program.fluidReference;
    plans=program.anchorPlan;amplitudes=0;terminalErrors=0;
    if reference.active
        amplitudes=reference.amplitudes;
        [plans,terminalErrors]=localFitFluidReference(program,reference.bump,reference.heading,cfg);
        information.status="candidate";information.widthMeters=reference.width;
        information.centerStationMeters=reference.center;information.conflictStages=reference.stages;
        information.referenceCount=2;information.activeTarget=reference.active;
    end
    best=Inf;bestPhysical=false;
    for candidate=1:size(plans,2)
        if ~localInitializationTimeAvailable(cfg)
            point=zeros(0,1);information.status="searchTimeLimit";return;
        end
        plan=plans(:,candidate);
        if ~reference.valid(candidate) || any(~isfinite(plan)) || terminalErrors(candidate)>1e-8,continue;end
        trial=localFluidPoint(program,plan);
        physicalExcess=max(program.physicalMatrix*trial-program.physicalBound);
        physical=physicalExcess<=0;
        information.referencePhysicalExcess(candidate)=physicalExcess;
        % A physically inadmissible fit cannot outrank an already selected
        % physical fit, regardless of its support score. Preserve side order
        % and tie rules; unevaluated diagnostic scores remain Inf.
        if bestPhysical && ~physical,continue;end
        states=localStates(program.prediction,plan);
        directions=angles;
        for index=1:numel(records)
            item=records(index);state=states(:,item.stage+1);
            relative=item.positionOffset+item.positionMap*state;
            yaw=item.yawOffset+item.yawRow*state;
            normal=avoidanceSafetyGeometry.supportDirection(relative,yaw,[0;0],item.targetYaw, ...
                [item.egoHalfSize;item.targetHalfSize]);
            directions(index)=atan2(normal(2),normal(1));
        end
        residual=avoidanceSafetyGeometry.jointResidual(program,trial,directions);
        score=0;maximum=0;
        if ~isempty(residual)
            score=max(residual-program.jointCertificate.upperBound);maximum=max(residual);
        end
        information.referenceScores(candidate)=score;
        if ~isfinite(score),continue;end
        % Prefer an actuator/road/chart/terminal-admissible fit before comparing
        % support residuals. No fitted seed, including such a fit, is issued.
        if isempty(point) || (physical && ~bestPhysical) ...
                || (physical==bestPhysical && score<best-64*eps*(1+abs(best)))
            best=score;bestPhysical=physical;point=trial;angles=directions;
            information.amplitudeMeters=amplitudes(candidate);
            information.terminalFitError=terminalErrors(candidate);
            information.maximumSupportResidual=maximum;
        end
    end
    if isempty(point),information.status="invalidFit";return;end
    information.maximumPhysicalExcess=max(program.physicalMatrix*point-program.physicalBound);
end

function point=localFluidPoint(program,plan)
    first=program.cones(2)+1;
    clf=program.b(first:first+5)-program.A(first:first+5,:)*[plan;0];
    slack=max(0,norm(clf(2:end))-clf(1));
    point=[plan;slack+64*eps*(1+slack+norm(clf))];
end

function [plans,terminalErrors]=localFitFluidReference(program,bump,heading,cfg)
% Fit both sides in one positive-definite system with two right-hand sides.
% Terminal state/input stay fixed; other constraints belong to the full SOCP.
    maps=program.prediction.egoStateMatrix;count=program.layout.planCount;
    weights=[1;cfg.admission.headingWeight];
    weighted=maps([2,3],:,2:end).*reshape(weights,[],1,1);
    map=reshape(permute(weighted,[1,3,2]),[],count);
    error=zeros(count,2);error(1:2:end,:)=bump(:,2:end).';
    error(2:2:end,:)=cfg.admission.headingWeight*heading(:,2:end).';
    difference=eye(count)-diag(ones(count-2,1),-2);
    metric=diag(program.inputWeight)+cfg.encounter.inputRateWeight/cfg.controller.sampleTime^2*(difference.'*difference);
    hessian=map.'*map+cfg.admission.regularizationWeight*metric;
    constraint=[maps(:,:,end);zeros(2,count)];constraint(end-1:end,end-1:end)=eye(2);
    scale=max(vecnorm(constraint,2,2),eps);scaled=constraint./scale;
    inverse=hessian\[map.'*error,scaled.'];
    change=inverse(:,1:2)-inverse(:,3:end)*(pinv(scaled*inverse(:,3:end))*(scaled*inverse(:,1:2)));
    terminalErrors=max(abs(constraint*change),[],1);
    plans=program.anchorPlan+change;
end

function available=localInitializationTimeAvailable(cfg)
    available=~isfield(cfg.solver,'workTimer') || ~isfinite(cfg.solver.workTimeLimit) ...
        || toc(cfg.solver.workTimer)<cfg.solver.workTimeLimit;
end
