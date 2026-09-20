classdef solveHardCbfClf
    %solveHardCbfClf Certified convex trajectory solves and independent hard-safety checks.
    methods (Static)
        function program = certify(program,decision)
        % Validate physical safety independently of a numerical success flag.
        % Transfer a feasible affine family WITHOUT repeated inward tightening.
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

        function [program,result,search] = joint(program,model,cfg)
        % Only a separately verified scalar plan or inherited incumbent can
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

        function [decision,angles,information] = admitSection(program,cfg)
            origin=program.anchorPlan;count=program.layout.planCount;
            records=program.jointCertificate.records;
            angles=program.jointCertificate.angles;decision=[];
            information=struct('status',"noDirection",'baseInterval',[], ...
                'safeIntervals',zeros(0,2),'amplitude',NaN,'basisResidual',NaN, ...
                'normalCount',cfg.admission.normalCount,'amplitudeCells',cfg.admission.amplitudeCells,'maximumYaw',NaN);
            if ~localSectionTimeAvailable(cfg),information.status="searchTimeLimit";return;end
            states=localStates(program.prediction,origin);
            [direction,information.basisResidual]=localDirection(program,cfg,states);
            if isempty(direction),return;end
            slope=localDirections(program.prediction,direction);
            physical=program.physicalMatrix(:,1:count);
            interval=solveHardCbfClf.linearInterval(physical*direction, ...
                program.safetyBound-physical*origin,[-Inf,Inf]);
            information.status="emptyLinearBase";
            if isempty(interval),return;end
            center=program.terminalCone.bound-program.terminalCone.matrix*origin;
            tangent=-program.terminalCone.matrix*direction;
            for first=1:3:numel(center)
                assert(tangent(first)==0,'affinePlanAdmission:variableConeTop', ...
                    'Terminal modal cones must have a constant radius.');
                interval=solveHardCbfClf.ballInterval(center(first+(1:2)), ...
                    tangent(first+(1:2)),center(first),interval);
                if isempty(interval),information.status="emptyTerminalBase";return;end
            end
            if any(~isfinite(interval),'all'),information.status="unboundedParameter";return;end
            information.baseInterval=interval;
            dictionary=program.geometry.frames(1).heading+2*pi*(0:cfg.admission.normalCount-1)/cfg.admission.normalCount;
            normals=[cos(dictionary);sin(dictionary)];
            total=numel(records);relative=zeros(total,2);relativeSlope=relative;
            yaw=zeros(total,1);yawSlope=yaw;egoSizes=zeros(total,2);egoRadius=yaw;
            targetSizes=egoSizes;targetYaw=yaw;targetRadius=yaw;constant=yaw;
            generatorSupport=zeros(total,numel(dictionary));
            for index=1:total
                item=records(index);state=states(:,item.stage+1);delta=slope(:,item.stage+1);
                relative(index,:)=(item.positionOffset+item.positionMap*state).';
                relativeSlope(index,:)=(item.positionMap*delta).';
                yaw(index)=item.yawOffset+item.yawRow*state;yawSlope(index)=item.yawRow*delta;
                egoSizes(index,:)=item.egoHalfSize.';egoRadius(index)=item.egoYawRadius;
                targetSizes(index,:)=item.targetHalfSize.';targetYaw(index)=item.targetYaw;
                targetRadius(index)=item.targetYawRadius;
                constant(index)=item.clearance+item.positionBall-program.jointCertificate.upperBound(index);
                generatorSupport(index,:)=sum(abs(item.generators.'*normals),1);
            end
            fixed=constant+generatorSupport+solveHardCbfClf.rectangleSupports( ...
                targetSizes,targetRadius,dictionary-targetYaw)-relative*normals;
            coefficient=relativeSlope*normals;
            boundaries=linspace(interval(1),interval(2),cfg.admission.amplitudeCells+1);
            intervals=zeros(cfg.admission.amplitudeCells*(total+1),2);intervalCount=0;collisionComponents=0;
            for cellIndex=1:cfg.admission.amplitudeCells
                if ~localSectionTimeAvailable(cfg),information.status="searchTimeLimit";return;end
                domain=boundaries(cellIndex:cellIndex+1);middle=mean(domain,'all');radius=diff(domain,1,2)/2;
                limits=fixed+solveHardCbfClf.rectangleSupports(egoSizes, ...
                    egoRadius+abs(yawSlope)*radius,dictionary-yaw-yawSlope*middle);
                % Charge evaluation and endpoint arithmetic before subtraction.
                limits=limits+256*eps*(1+abs(limits)+abs(coefficient)*max(abs(domain),[],'all'));
                ratio=limits./coefficient;
                lower=ratio;lower(coefficient>=0)=-Inf;lower=max(lower,[],2);
                upper=ratio;upper(coefficient<=0)=Inf;upper=min(upper,[],2);
                alwaysSafe=any(coefficient==0 & limits<=0,2);
                keep=~alwaysSafe & lower<upper;
                collision=keep & ~[records.isExit].';
                collisionIntervals=solveHardCbfClf.subtractIntervals(domain,[lower(collision),upper(collision)]);
                collisionComponents=collisionComponents+size(collisionIntervals,1);
                component=solveHardCbfClf.subtractIntervals(domain,[lower(keep),upper(keep)]);
                intervals(intervalCount+(1:size(component,1)),:)=component;
                intervalCount=intervalCount+size(component,1);
            end
            intervals=intervals(1:intervalCount,:);information.safeIntervals=intervals;
            if isempty(intervals)
                information.status="collisionExcluded";
                if collisionComponents>0,information.status="exitExcluded";end
                return;
            end
            [quadratic,linear,clfCenter,clfSlope,clfRadius]=localObjective(program,origin,direction,states,slope);
            left=interval(1);right=interval(2);
            for iteration=1:cfg.admission.performanceIterations
                middle=(left+right)/2;value=clfCenter+clfSlope*middle;length=norm(value);
                derivative=2*quadratic*middle+linear;
                if length>clfRadius && length>0
                    derivative=derivative+2*program.slackWeight*(length-clfRadius)*(clfSlope.'*value)/length;
                end
                if derivative>0,right=middle;else,left=middle;end
            end
            inward=64*eps*(1+max(abs(intervals),[],2));
            lower=min(intervals(:,1)+inward,mean(intervals,2));
            upper=max(intervals(:,2)-inward,mean(intervals,2));
            candidates=min(max((left+right)/2,lower),upper);
            slack=max(0,vecnorm(clfCenter+clfSlope*candidates.',2,1).'-clfRadius);
            costs=quadratic*candidates.^2+linear*candidates+program.slackWeight*slack.^2;
            % Resolve arithmetic-scale objective ties by interval order so
            % generated BLAS/SVD arithmetic cannot flip symmetric solutions.
            best=min(costs);
            selected=find(costs<=best+64*eps*(1+abs(best)),1,'first');
            selected=selected(1);amplitude=candidates(selected);
            plan=origin+amplitude*direction;
            decision=[plan;slack(selected)+64*eps*(1+slack(selected)+norm(clfCenter+clfSlope*amplitude))];
            chosen=states+amplitude*slope;
            % Select a certifying dictionary direction at the chosen witness.
            actual=constant+generatorSupport+solveHardCbfClf.rectangleSupports( ...
                targetSizes,targetRadius,dictionary-targetYaw) ...
                +solveHardCbfClf.rectangleSupports(egoSizes,egoRadius,dictionary-yaw-yawSlope*amplitude) ...
                -(relative+amplitude*relativeSlope)*normals;
            [~,indices]=min(actual,[],2);angles=dictionary(indices).';
            if size(angles,2)>1,angles=angles.';end
            information.status="candidate";information.amplitude=amplitude;
            information.maximumYaw= max(abs(chosen(3,:)),[],'all');
        end

        function interval = linearInterval(coefficient,bound,domain)
            interval=domain;
            coder.varsize('interval',[1,2],[true,true]);
            if isempty(interval),return;end
            if any(~isfinite(coefficient)) || any(~isfinite(bound)) ...
                    || any(coefficient==0 & bound<0)
                interval=[];return;
            end
            positive=coefficient>0;negative=coefficient<0;
            interval=[max([interval(1);bound(negative)./coefficient(negative)]), ...
                min([interval(2);bound(positive)./coefficient(positive)])];
            if interval(1)>interval(2),interval=[];end
        end

        function interval = ballInterval(center,direction,radius,domain)
            interval=domain;
            coder.varsize('interval',[1,2],[true,true]);
            if isempty(interval),return;end
            if radius<0 || ~isfinite(radius) || any(~isfinite([center;direction]))
                interval=[];return;
            end
            length=norm(direction);
            if length==0
                if norm(center)>radius,interval=[];end
                return;
            end
            unit=direction/length;middle=-(unit.'*center)/length;
            distance=norm(center+middle*direction);
            if distance>radius,interval=[];return;end
            halfWidth=sqrt(max(0,(radius-distance)*(radius+distance)))/length;
            interval=[max(interval(1),middle-halfWidth),min(interval(2),middle+halfWidth)];
            if interval(1)>interval(2),interval=[];end
        end

        function intervals = subtractIntervals(domain,forbidden)
            % The forbidden intervals are open: touching endpoints survive.
            forbidden=sortrows(forbidden,1);intervals=zeros(size(forbidden,1)+1,2);
            cursor=domain(1);count=0;
            for index=1:size(forbidden,1)
                lower=forbidden(index,1);upper=forbidden(index,2);
                if lower>=upper || upper<=cursor || lower>domain(2),continue;end
                if lower>=cursor
                    count=count+1;intervals(count,:)=[cursor,min(lower,domain(2))];
                end
                cursor=max(cursor,upper);
                if cursor>domain(2),break;end
            end
            if cursor<=domain(2),count=count+1;intervals(count,:)=[cursor,domain(2)];end
            intervals=intervals(1:count,:);
        end

        function support = rectangleSupports(halfSize,radius,angle)
            % Analytic maximum over the complete yaw interval, row by row.
            long=halfSize(:,1);wide=halfSize(:,2);circumradius=hypot(long,wide);
            first=angle-radius;last=angle+radius;
            support=max(long.*abs(cos(first))+wide.*abs(sin(first)), ...
                long.*abs(cos(last))+wide.*abs(sin(last)));
            peak=atan2(wide,long);
            distance=min(abs(mod(angle-peak+pi/2,pi)-pi/2),abs(mod(angle+peak+pi/2,pi)-pi/2));
            support=max(support,circumradius.*(distance<=radius+32*eps*(1+abs(angle))));
            support=support+32*eps*(1+circumradius);
        end
    end
end

function [program,result,search]=localJointSearch(program,~,cfg)
% Fresh admission searches one scalar section. Continuation retains its witness.
    search=struct('hardSolves',0,'restorationSolves',0,'baseSolves',0,'nativeSolves',0, ...
        'familyAttempts',0,'horizonAttempts',1,'formulationSeconds',0,'solveSeconds',0, ...
        'initialOverlappingNodes',program.supportGeometry.overlappingNodes, ...
        'policy',"affineSectionAdmission",'violationHistory',{{}},'usedCertifiedIncumbent',false, ...
        'issuedAdmissionWitness',false,'initialCertificateAngles',program.jointCertificate.angles);
    point=program.feasibleWitness;angles=program.jointCertificate.angles;
    [admitted,accepted]=localVerifyJointPoint(program,point,angles);
    if ~program.inheritedPredictionFamily && ~admitted
        phase=tic;
        [point,angles,search.section]=solveHardCbfClf.admitSection(program,cfg);
        search.formulationSeconds=toc(phase);search.familyAttempts=1;
        if ~isempty(point)
            [admitted,accepted]=localVerifyJointPoint(program,point,angles);
            if ~admitted,search.section.status="independentVerificationFailed";end
        end
        if ~admitted
            result=localEmptySolve();
            result.message="Scalar admission found no hard-certified plan: "+search.section.status;
            return;
        end
        program=accepted;result=localEmptySolve();result.decision=point;result.feasible=true;
        result.exitFlag=1;result.message="Issued an independently verified scalar-admission witness.";
        search.issuedAdmissionWitness=true;
    else
        if ~admitted
            result=localEmptySolve();result.message="The inherited witness failed independent verification.";return;
        end
        program=localRefreshBounds(accepted);incumbent=point;
        phase=tic;conic=avoidanceStageQp.joint(program,point,angles,cfg);
        search.formulationSeconds=toc(phase);
        phase=tic;trial=localSolveConic(conic,cfg);search.solveSeconds=toc(phase);
        search.hardSolves=1;search.nativeSolves=1;improved=false;
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

function states=localStates(prediction,inputs)
    count=prediction.stageCount;states=zeros(6,count+1);
    states(:,1)=prediction.egoStateOffset(:,1);
    for stage=1:count
        states(:,stage+1)=prediction.stageMatrixA(:,:,stage)*states(:,stage) ...
            +prediction.stageMatrixB(:,:,stage)*inputs(2*stage-1:2*stage)+prediction.stageAffine(:,stage);
    end
end

function states=localDirections(prediction,inputs)
    count=prediction.stageCount;states=zeros(6,count+1);
    for stage=1:count
        states(:,stage+1)=prediction.stageMatrixA(:,:,stage)*states(:,stage) ...
            +prediction.stageMatrixB(:,:,stage)*inputs(2*stage-1:2*stage);
    end
end

function [direction,residual]=localDirection(program,cfg,states)
    records=program.jointCertificate.records;
    values=avoidanceSafetyGeometry.jointResidual(program,program.feasibleWitness,program.jointCertificate.angles);
    values([records.isExit])=-Inf;
    [~,critical]=max(values-program.jointCertificate.upperBound);
    matching=false(numel(records),1);
    for index=1:numel(records)
        matching(index)=isequal(records(index).key,records(critical).key);
    end
    selected=find(matching & ~[records.isExit].' & values>program.jointCertificate.upperBound);
    count=program.layout.planCount;maps=program.prediction.egoStateMatrix;
    if isempty(selected)
        index=program.terminal.stateIndex;
        constraint=[maps(index,:,end);zeros(2,count)];
        constraint(end-1:end,end-1:end)=eye(2);
        desired=[program.terminal.reference(index)-states(index,end); ...
            program.terminal.input-program.anchorPlan(end-1:end)];
    else
        stages=[records(selected).stage];first=min(stages,[],'all');last=max(stages,[],'all');
        samples=unique([first,round((first+last)/2),last]);
        initial=records(selected(1));final=records(selected(end));
        travel=final.positionOffset+final.positionMap*states(:,final.stage+1) ...
            -initial.positionOffset-initial.positionMap*states(:,initial.stage+1);
        transverse=initial.positionMap(:,1:2)\[-travel(2);travel(1)];
        if norm(transverse)<=64*eps*(1+norm(travel)),transverse=[0;1];end
        transverse=transverse/norm(transverse);
        constraint=[reshape(permute(maps(1:2,:,samples+1),[1,3,2]),[],count);maps(:,:,end);zeros(2,count)];
        constraint(end-1:end,end-1:end)=eye(2);
        terminalChange=zeros(6,1);index=program.terminal.stateIndex;
        terminalChange(index)=program.terminal.reference(index)-states(index,end);
        desired=[repmat(transverse,numel(samples),1);terminalChange; ...
            program.terminal.input-program.anchorPlan(end-1:end)];
    end
    scale=max(vecnorm(constraint,2,2),eps);constraint=constraint./scale;desired=desired./scale;
    difference=speye(count)-spdiags(ones(count,1),-2,count,count);
    metric=spdiags(program.inputWeight,0,count,count) ...
        +cfg.encounter.inputRateWeight/cfg.controller.sampleTime^2*(difference.'*difference);
    root=chol(metric);map=constraint/root;
    direction=root\(pinv(full(map))*desired);
    residual=norm(constraint*direction-desired,Inf);
    if any(~isfinite(direction)) || norm(direction)==0,direction=zeros(0,1);end
end

function [quadratic,linear,center,slope,radius]=localObjective(program,origin,direction,states,stateSlope)
    quadratic=sum(program.inputWeight.*direction.^2);
    linear=2*sum(program.inputWeight.*(origin-program.referenceInputs(:)).*direction);
    for stage=1:program.prediction.stageCount
        error=states(2:6,stage+1)-program.referenceStates(2:6,stage+1);
        change=stateSlope(2:6,stage+1);matrix=program.referenceMatrices(:,:,stage+1);
        quadratic=quadratic+change.'*matrix*change;
        linear=linear+2*error.'*matrix*change;
    end
    first=program.cones(2)+1;
    value=program.b(first:first+5)-program.A(first:first+5,:)*[origin;0];
    delta=-program.A(first:first+5,:)*[direction;0];
    assert(delta(1)==0 && value(1)>=0,'affinePlanAdmission:invalidClf','The CLF norm radius must be fixed and nonnegative.');
    center=value(2:end);slope=delta(2:end);radius=value(1);
end

function available=localSectionTimeAvailable(cfg)
    available=~isfield(cfg.solver,'workTimer') || ~isfinite(cfg.solver.workTimeLimit) ...
        || toc(cfg.solver.workTimer)<cfg.solver.workTimeLimit;
end
