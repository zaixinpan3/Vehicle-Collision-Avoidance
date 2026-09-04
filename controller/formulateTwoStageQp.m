function [qp, common] = formulateTwoStageQp( ...
        model, prediction, linearizationInput, options, common)
% formulateTwoStageQp Assemble the two-stage program at one linearization.
%
% The program of TWO_STAGE_QP.md, after Li, Zhang, Guo, Lenzo and Guo,
% "Real-Time Optimal Trajectory Planning for Autonomous Driving with
% Collision Avoidance Using Convex Optimization" (Automotive Innovation
% 2023), Sect. 3. The collision-free condition dist(ego_k, O_k) >= d_min
% is non-convex in the state; the paper replaces it in TWO STAGES:
%
%   STAGE 1 (Eq. 12). At every node solve the DUAL of the distance
%   problem between the linearization trajectory's ego pose pbar_k and
%   the obstacle polygon O = {z : A z <= b},
%
%       lambda*_k = argmax  (A pbar_k - b)' lambda
%                   s.t.    ||A' lambda||_2 <= 1,   lambda >= 0.
%
%   Its optimal value is the distance, its optimizer the separating
%   direction n_k = A' lambda*_k (a unit vector) with support
%   lambda*_k' b = h_O(n_k).
%
%   STAGE 2 (Eq. 13). With lambda*_k FROZEN the collision row is affine
%   in the state,
%
%       (A p_k(z) - b)' lambda*_k >= d_min,
%
%   and the trajectory is one convex QP.
%
% By weak duality (A p - b)' lambda* <= dist(p, O) for EVERY p, so the
% frozen row is SUFFICIENT for the true condition at every plan; by
% strong duality it is EXACT at pbar_k (TWO_STAGE_SAFETY.md, Lemma 2).
% Geometrically the row is the supporting half-plane of the obstacle
% facing the linearization trajectory - the tangent of the convex
% distance function there.
%
% THE LINEARIZATION TRAJECTORY DECIDES THE HOMOTOPY CLASS, AND THAT IS
% THE WHOLE DIFFICULTY. The frozen row is one of the four supporting
% half-planes of the box, and which one is a function of the
% trajectory alone. A trajectory behind the target yields the REAR
% half-plane "stay behind the rear face": its coefficient on the
% lateral coordinate is zero, so the program cannot see that steering
% would clear the target, and its feasible set contains no plan that
% passes. Re-solving at the solution reproduces a behind trajectory,
% so the receding-horizon iteration - and any number of sequential
% convex iterations within one sample - has a FIXED POINT in the
% braking class and stays there even when braking can no longer avoid
% the target and passing is feasible. No convex relaxation escapes on
% its own: the union of the four half-planes is the complement of the
% box, whose convex hull is the whole plane, so any single convex
% program is blind to every class but the one it was linearized in.
%
% THE ESCAPE, therefore, cannot come from the QP. It comes from where
% stage 1 is EVALUATED, and this function is written so that the
% caller can evaluate it anywhere: `linearizationInput` is any input
% plan, and stage 1, the CLF tangents and the trust region are all
% built about it. Certified mode evaluates hard problems directly at
% the shifted plan and, when enabled, the CRUISE PROBE of
% localCruiseProbeInput - the trajectory the vehicle would follow if
% the target were not there. The probe runs INTO the target
% box, where stage 1's interior extension returns the face of least
% penetration: for a configuration obstacle longer than it is wide,
% and a probe that enters from behind, that edge is a LATERAL one from
% the first deeply penetrating node on. Those are rows that credit
% steering. Nothing here names a
% side, ramps a lateral profile, or is told which way to go: the side
% is the sign of the least-penetration face of the probe, and the
% choice between the classes found is the objective's. If no hard candidate
% is feasible, certified mode declares controller failure. The elastic
% sequential-convex ladder described below belongs only to the optional
% noncertified legacy path.
%
% `options` (all optional):
%   label             the candidate's name, reported.
%   obstacleMode      "hard" (default) - the target rows are hard, a
%                     plan that cannot honour them is "no solution";
%                     "certified" - every collision and road CBF row,
%                     including stage 0 and the terminal domain, is hard;
%                     only the exact first-step CLF may be relaxed;
%                     "elastic" - one nonnegative slack per imposed
%                     target node, priced violationPenalty per metre in
%                     the objective, and no gate: the program is always
%                     feasible. ELASTIC PROGRAMS ARE NEVER COMMITTED.
%                     They are the ladder's PROBES - the virtual
%                     control of a successive-convexification scheme,
%                     which exists to stop the linearization from being
%                     artificially infeasible while it moves between
%                     classes. A probe therefore carries only what
%                     decides whether a CLASS is reachable - the road
%                     rows, the speed and heading domains, and the input
%                     box, all hard - and drops
%                     what only shapes a committed plan: the friction
%                     polygons (960 of the 1960 rows, and the facets the
%                     active set walks along), the CLF rows and their
%                     trust region, whose bound on a tangent's validity
%                     is meaningless for a trajectory nobody executes.
%                     Measured on the probe of a locked sample: 1960
%                     rows and 709 active-set pivots (0.43 s) against
%                     524 rows and 16 interior-point iterations
%                     (0.06 s), the same escape trajectory, and the
%                     same feasible hard program after it. The hard
%                     program re-imposes every row, so nothing a probe
%                     omits can reach the vehicle.
%   violationPenalty  the elastic price, metres^-1 (ignored when hard).
%   trustRegionScale  the CLF trust region of this program, in error
%                     scales (Inf removes it; a probe carries none).
%
% WHERE A HARD ROW IS A CONSTRAINT. Under the Euler stage map the pose
% one step ahead is a fact of the measured state and the input's
% authority over a node's margin grows like the fourth power of the
% node index. One sample of plant mismatch moves a margin by up to the
% declared cfg.collision.disturbanceBound d; a row no admissible plan
% satisfies is not a constraint but a fact of the state, and a fact
% is only ever mismatch-sized. Two tiers decide, node by node
% (localSeparationRows): a row some plan in the input reach
% box satisfies is imposed; a row every plan in the box misses, at a
% node k of the NEAR WINDOW (authority below
% cfg.collision.factWindowFactor times d, top node W) by at most the
% mismatch it can have accumulated since it was last a hard row,
% (W - k + 1) d, is a fact; a row missed by more, or outside the
% window, makes the candidate infeasible (certified without a solve).
% When a candidate is nevertheless infeasible - the friction polygon,
% which the box does not see, takes the capacity away at the near
% nodes - the window rows no admissible plan satisfies jointly are
% named by one least-relaxation LP and, if each is missed by at most
% its allowance, become facts
% (collisionAvoidanceController>localFactRelaxation). Facts are
% evaluated and reported; rows outside the window are never dropped,
% so a conflict the input has authority over, or a class the vehicle
% cannot reach in time, is "no solution" (TWO_STAGE_SAFETY.md,
% Proposition 2). The gate applies to HARD programs only: an elastic
% iterate carries every covered row, because its slack, not its
% feasibility, is what the ladder reads.
%
% THE GEOMETRY IS THE TRUE CONFIGURATION OBSTACLE. Everything is in
% PATH COORDINATES over the scheduled Frenet LTV bicycle
% (ltvBicyclePrediction), x = [s; d; ePsi; vx; vy; r], nodes
% x_k = E_k u + e_k, k = 0..N, x_0 the measured state. At node k the
% target is an ORIENTED RECTANGLE - its projected centre (sT_k, dT_k),
% its heading error eT_k to the path, its true half-extents - and the
% ego is an oriented rectangle at the plan's heading error. The two
% intersect exactly when the ego CENTRE lies in their Minkowski sum,
% which for two centrally symmetric rectangles is the zonotope of
% their four half-edge generators: a convex polygon of at most eight
% vertices whose edge normals are the two rectangles' own. No bounding
% box is taken of either rectangle anywhere. Stage 1 projects the
% linearization's centre onto that polygon
% (rectangleConfigurationDistance), which returns the signed distance
% and the supporting normal n_k directly - and, for a probe that has
% run inside it, the outward normal of the least-penetrated EDGE, the
% same minimum-penetration answer with no rectangle-specific face
% logic anywhere.
%
% Stage 2 freezes n_k and writes the row through the two rectangles'
% SUPPORT FUNCTIONS. For a unit n the separation of the ego centre p
% from the obstacle is
%
%   n'(p - c_T) >= h_T(n; eT) + h_E(n; ePsi),
%   h(n; psi) = l |cos(alpha - psi)| + w |sin(alpha - psi)|, alpha = angle(n),
%
% by weak duality, the support of a Minkowski sum being the sum of the
% supports. h_T is a constant of the node; h_E moves with the
% decision's heading and is not affine in it, so it is charged by the
% EXACT Lipschitz bound of h_E over the heading interval the program
% admits,
%
%   h_E(n; ePsi) <= h_E(n; ePsibar) + b_k |ePsi - ePsibar|,
%
% b_k = max |dh_E/dpsi| there (localSupportSlopeBound, closed form),
% anchored at the LINEARIZATION's heading and not at zero. The row is
% that bound split into its two signs sigma = +-1,
%
%   n_k'p_k(z) - sigma b_k (ePsi_k(z) - ePsibar_k)
%       - n_k'c_T - h_T - h_E(n_k; ePsibar_k) - tau_k >= 0,
%
% whose minimum over sigma is a sufficient condition for the TRUE
% rectangle separation at every plan and exact at the linearization
% (TWO_STAGE_SAFETY.md, Lemmas 1'-4). tau_k carries the clearance
% margin, the position radii and the curvature sagitta
% (localTargetTightening); the yaw radii of both rectangles are taken
% DIRECTIONALLY inside their supports (localRectangleSupport), where an
% inflated axis-aligned box could only over-charge them. Every road
% boundary is a half-plane obstacle with the same row and the same two
% helpers (n = -+e_d).
%
% THE TERMINAL SET closes the horizon (TWO_STAGE_SAFETY.md, "The
% terminal set"). After node N the prediction continues as the
% KINEMATIC BRAKING TAIL of ltvBicyclePrediction: N_b further stages
% whose accelerations a_N..a_{N+N_b-1} are decision variables, the
% lateral state carried frozen inside the band that
% terminalLateralCertificate certifies for the tail's lane-keeping
% law. The tail carries (localTailRows, localTailBox) the box
% [-a_b, 0], nonnegative speed and REST at its last node; the terminal node
% carries the handoff rows |ePsi_N|, |vy_N| and |r_N - kappa vx_N|
% under which the certificate applies (localTerminalRows); and every
% tail node carries the same separation and road rows as the head,
% with the ego's support maximized over the certified heading band and
% the band's lateral drift in the tightening. A plan is thus admitted
% only if the vehicle can come to rest from its terminal state without
% violating a row on the way. At the rest node a hard inertial row
% separates the ego from the support of the controller's complete
% predicted target continuation. The target predictor's support interface,
% rather than a trajectory-class predicate, closes the infinite tail. The
% tail is proof-side: nothing of it is executed, and its accelerations are
% priced only for positive definiteness.
%
% The rest of the program: the soft CLF rows of the path-frame cruise
% error about the cruise equilibrium, linearized at THIS
% linearization (localClfData, cruiseEquilibrium), their trust region
% (localClfTrustRegionRows), the hard speed-, heading-,
% friction-polygon and slip rows and the input box
% (all independent of the linearization, built once into `common`),
% and the minimum-intervention objective. The decision is
%
%   z = [u_0; ...; u_{N-1}; a_N; ...; a_{N+N_b-1};
%        delta_0; ...; delta_{N-1}; xi_1; ...; xi_M],
%
% u_i = [deltaF; a], a_{N+j} the tail accelerations, delta the CLF
% relaxations, xi the elastic target slacks (absent when the rows are
% hard); the head inputs and the tail together are the PLAN
% (layout.planIndex), and every state row is written over the plan
% columns. `linearizationInput` is a plan vector. The objective is
%
%   sum_i Ts (u_i - u_eq,i)' R (u_i - u_eq,i) + w2 sum delta_i^2
%   + w1 sum delta_i + violationPenalty sum xi_j.
%
% `common` is returned so the ladder's every program reuses the
% linearization-independent half of the build.

    if nargin < 5 || isempty(common)
        common = localCommonParts(model, prediction);
    end
    cfg = model.cfg;
    if nargin < 4
        options = struct();
    end
    options = localFillOptions(options, cfg);
    baseLayout = common.layout;

    % The linearization PLAN: the head's input plan stacked column-wise
    % and the tail's accelerations after it - one vector over the plan
    % columns of the decision (layout.planIndex).
    linearizationPlan = linearizationInput(:);
    nominalState = localNominalState(prediction, model, linearizationPlan);

    % ---- STAGE 1: the dual separation of every obstacle AT THIS
    % linearization trajectory, over the head and the tail alike.
    families = [localTargetFamily(model, prediction, nominalState, ...
            common.tightening, common.terminal); ...
        localRoadFamilies(model, prediction, nominalState, ...
            common.terminal)];

    % ---- STAGE 2: the affine separation rows with the frozen duals.
    [families, obstacleMatrix, obstacleBound, obstacleFamily, ...
        obstacleNode, obstacleWindow, obstacleAllowance, ...
        obstacleSlackNode, certifiedInfeasible] = localSeparationRows( ...
            families, prediction, common, cfg.collision, ...
            options.obstacleMode, linearizationPlan);

    % ---- THE DISJUNCTIVE MODEL. In this mode the target contributes
    % no row to the base program: the exact condition is the OR over
    % the obstacle's facets, and which facet holds at each node is a
    % decision the search makes (collisionAvoidanceController>
    % localDisjunctiveSearch), not a consequence of the linearization.
    disjunction = repmat(localEmptyDisjunctionNode(baseLayout), 0, 1);
    if options.disjunctive
        disjunction = localDisjunctionData(model, prediction, common, ...
            cfg.collision, families(1), nominalState);
        for nodeIdx = 1:numel(disjunction)
            families(1).nodes(nodeIdx).imposed = ...
                disjunction(nodeIdx).imposed;
            families(1).nodes(nodeIdx).factSource = ...
                disjunction(nodeIdx).factSource;
        end
        targetRow = obstacleFamily == "collision";
        obstacleMatrix = obstacleMatrix(~targetRow, :);
        obstacleBound = obstacleBound(~targetRow);
        obstacleNode = obstacleNode(~targetRow);
        obstacleWindow = obstacleWindow(~targetRow);
        obstacleAllowance = obstacleAllowance(~targetRow);
        obstacleSlackNode = obstacleSlackNode(~targetRow);
        obstacleFamily = obstacleFamily(~targetRow);
        certifiedInfeasible = certifiedInfeasible ...
            || any([disjunction.certifiedInfeasible]);
    end

    % ---- the optional legacy elastic-probe slack block. Certified mode
    % introduces no CBF slack: every collision and road row is hard.
    % Elastic refinement is available only to the noncertified legacy path
    % and its target-only slacks are never committed.
    slackNodes = unique(obstacleSlackNode(obstacleSlackNode > 0));
    slackCount = numel(slackNodes);
    slackSelector = zeros(numel(obstacleBound), 1);
    for slackIdx = 1:slackCount
        slackSelector(obstacleSlackNode == slackNodes(slackIdx)) = slackIdx;
    end
    layout = baseLayout;
    layout.slackCount = slackCount;
    layout.slackIndex = baseLayout.decisionCount+(1:slackCount);
    layout.decisionCount = baseLayout.decisionCount+slackCount;
    slackBlock = zeros(numel(obstacleBound), slackCount);
    for rowIdx = 1:numel(obstacleBound)
        if slackSelector(rowIdx) > 0
            slackBlock(rowIdx, slackSelector(rowIdx)) = -1.0;
        end
    end

    % ---- the CLF rows and the trust region AT THIS linearization.
    % A probe carries neither, and only the model rows that decide
    % whether a class is reachable (common.probeRow).
    clf = localClfData(prediction, model, linearizationPlan, ...
        baseLayout, common.equilibrium);
    [trustMatrix, trustBound, trustNode] = localClfTrustRegionRows( ...
        clf, model, baseLayout, options.trustRegionScale);
    modelRow = true(numel(common.modelBound), 1);
    if options.obstacleMode == "elastic"
        modelRow = common.probeRow;
        trustMatrix = trustMatrix(1:0, :);
        trustBound = trustBound(1:0);
        trustNode = trustNode(1:0);
        clfMatrix = clf.constraintMatrix(1:0, :);
        clfBound = clf.constraintBound(1:0);
        clfNode = zeros(0, 1);
    elseif options.obstacleMode == "certified"
        trustMatrix = trustMatrix(1:0, :);
        trustBound = trustBound(1:0);
        trustNode = trustNode(1:0);
        clfMatrix = clf.constraintMatrix(1:0, :);
        clfBound = clf.constraintBound(1:0);
        clfNode = zeros(0, 1);
    else
        clfMatrix = clf.constraintMatrix;
        clfBound = clf.constraintBound;
        clfNode = (2:baseLayout.horizonSteps+1).';
    end
    modelMatrix = common.modelMatrix(modelRow, :);
    modelBound = common.modelBound(modelRow);
    modelFamily = common.modelFamily(modelRow);
    modelNode = common.modelNode(modelRow);

    modelCount = numel(modelBound);
    trustCount = numel(trustBound);
    clfCount = numel(clfBound);
    pad = @(matrix) [matrix, zeros(size(matrix, 1), slackCount)];

    qp = struct();
    if options.obstacleMode == "certified"
        qp.problemClass = "recursiveFeasibleHardCbfClfSocpPathCoordinates";
    else
        qp.problemClass = "twoStageApproximateConvexQpPathCoordinates";
    end
    qp.label = options.label;
    qp.obstacleMode = options.obstacleMode;
    qp.violationPenalty = options.violationPenalty;
    qp.trustRegionScale = options.trustRegionScale;
    qp.trustRegion = isfinite(options.trustRegionScale);
    qp.certifiedInfeasible = certifiedInfeasible;
    qp.layout = layout;
    qp.Hessian = blkdiag(common.hessian, ...
        2.0e-8*speye(slackCount, slackCount));
    qp.Hessian = full(qp.Hessian);
    qp.linear = [common.linear; ...
        repmat(options.violationPenalty, slackCount, 1)];
    qp.constant = common.constant;
    qp.inequalityMatrix = [[obstacleMatrix, slackBlock]; ...
        pad(modelMatrix); pad(trustMatrix); pad(clfMatrix)];
    qp.inequalityBound = [obstacleBound; modelBound; trustBound; ...
        clfBound];
    qp.rowFamily = [obstacleFamily; modelFamily; ...
        repmat("clfTrust", trustCount, 1); repmat("clf", clfCount, 1)];
    qp.rowNode = [obstacleNode; modelNode; trustNode; clfNode];
    % Rows of the near window: the only ones the fact relaxation may
    % declare facts when the program is infeasible.
    qp.rowWindow = [obstacleWindow; ...
        false(modelCount+trustCount+clfCount, 1)];
    qp.rowAllowance = [obstacleAllowance; ...
        zeros(modelCount+trustCount+clfCount, 1)];
    qp.lowerBound = [common.lowerBound; zeros(slackCount, 1)];
    qp.upperBound = [common.upperBound; inf(slackCount, 1)];
    qp.variableScale = [common.variableScale; ones(slackCount, 1)];
    qp.collision = families(1);
    qp.disjunction = disjunction;
    qp.disjunctive = options.disjunctive;
    qp.road = families(2:end);
    qp.clf = clf;
    qp.terminal = common.terminal;
    qp.linearizationPlan = linearizationPlan;
    qp.nominalState = nominalState;
    qp.safetySlackNodes = slackNodes;

    % Initial point: the linearization plan itself, with each CLF
    % relaxation and each elastic slack at its own need there - a
    % feasible point of every soft row (and, when the linearization is
    % separated from the target, of the hard ones too: Lemma 4).
    initialDecision = zeros(layout.decisionCount, 1);
    initialDecision(layout.planIndex) = linearizationPlan;
    if clfCount > 0
        clfResidual = clfMatrix ...
            * initialDecision(1:baseLayout.decisionCount)-clfBound;
        initialDecision(layout.relaxationIndex) = max(clfResidual, 0.0);
    end
    if slackCount > 0
        baseDecision = initialDecision(1:baseLayout.decisionCount);
        obstacleResidual = obstacleMatrix*baseDecision-obstacleBound;
        for slackIdx = 1:slackCount
            rows = slackSelector == slackIdx;
            initialDecision(layout.slackIndex(slackIdx)) = ...
                max([0.0; obstacleResidual(rows)]);
        end
    end
    qp.initialDecision = initialDecision;
end

function options = localFillOptions(options, cfg)
    if ~isfield(options, "label") || strlength(string(options.label)) == 0
        options.label = "incumbent";
    end
    options.label = string(options.label);
    if ~isfield(options, "obstacleMode")
        options.obstacleMode = "hard";
    end
    options.obstacleMode = string(options.obstacleMode);
    if ~ismember(options.obstacleMode, ["hard", "elastic", "certified"])
        error("collisionAvoidanceController:invalidFormulation", ...
            "obstacleMode must be hard, elastic, or certified.");
    end
    if ~isfield(options, "violationPenalty") ...
            || options.obstacleMode == "hard"
        options.violationPenalty = 0.0;
    end
    if ~isfield(options, "trustRegionScale")
        options.trustRegionScale = cfg.clf.trustRegionScale;
    end
    if ~isfield(options, "disjunctive")
        options.disjunctive = false;
    end
    options.disjunctive = logical(options.disjunctive) ...
        && options.obstacleMode == "hard";
end

% ====================================================================
% The linearization-independent half of the build
% ====================================================================

function common = localCommonParts(model, prediction)
% Everything that does not depend on where stage 1 and the CLF are
% linearized: the model-domain, friction and slip rows and the input
% box, the minimum-intervention objective (the cruise
% equilibrium is a function of the schedule curvature and the
% published bias, not of any trajectory), the reach box of the gate,
% the target tightening, and the CLF certificate. Built once per
% sample and shared by every program of the ladder.
    cfg = model.cfg;
    horizonSteps = model.horizonSteps;
    inputCount = prediction.inputCount;
    tailSteps = prediction.tailSteps;
    planCount = prediction.planCount;
    % The decision: the head inputs, the tail accelerations (the
    % terminal set's witness), the CLF relaxations, then any elastic
    % slacks. The PLAN columns are the first two blocks - every state
    % row is written over them.
    layout = struct( ...
        "inputDimension", model.inputDimension, ...
        "horizonSteps", horizonSteps, ...
        "tailSteps", tailSteps, ...
        "inputCount", inputCount, ...
        "planCount", planCount, ...
        "relaxationCount", horizonSteps, ...
        "slackCount", 0, ...
        "decisionCount", planCount+horizonSteps, ...
        "inputIndex", 1:inputCount, ...
        "tailIndex", inputCount+(1:tailSteps), ...
        "planIndex", 1:planCount, ...
        "relaxationIndex", planCount+(1:horizonSteps), ...
        "slackIndex", zeros(1, 0));

    common = struct();
    common.layout = layout;
    speedInterval = localPlannedSpeedInterval(prediction, model);
    common.terminal = localTerminalBudget(model, prediction, speedInterval);
    [common.reachCentre, common.reachHalfWidth] = localReachBox( ...
        model, layout);
    common.tightening = localTargetTightening(model, prediction, ...
        common.terminal);
    common.certificate = localClfCertificate(model);
    common.equilibrium = localCruiseEquilibriumProfile(model, prediction);
    [common.hessian, common.linear, common.constant] = localObjective( ...
        model, common.equilibrium.input, layout);

    [speedMatrix, speedBound] = localSpeedDomainRows( ...
        prediction, model, layout, speedInterval);
    [headingMatrix, headingBound] = localHeadingDomainRows( ...
        prediction, model, layout);
    % frictionCirclePolygonRows writes its rows as matrix*u + offset <= 0
    % over the plan columns; it touches the head inputs only.
    [frictionMatrix, frictionOffset] = ...
        frictionCirclePolygonRows(prediction, model);
    frictionMatrix = [frictionMatrix, zeros(size(frictionMatrix, 1), ...
        layout.decisionCount-size(frictionMatrix, 2))];
    frictionBound = -frictionOffset;
    [tailMatrix, tailBound] = localTailRows(prediction, model, layout);
    [terminalMatrix, terminalBound] = localTerminalRows( ...
        prediction, model, layout, common.terminal);
    common.modelMatrix = [speedMatrix; headingMatrix; frictionMatrix; ...
        tailMatrix; terminalMatrix];
    common.modelBound = [speedBound; headingBound; frictionBound; ...
        tailBound; terminalBound];
    common.modelFamily = [ ...
        repmat("speedDomain", numel(speedBound), 1); ...
        repmat("headingDomain", numel(headingBound), 1); ...
        repmat("friction", numel(frictionBound), 1); ...
        repmat("tail", numel(tailBound), 1); ...
        repmat("terminal", numel(terminalBound), 1)];
    common.modelNode = zeros(numel(common.modelBound), 1);
    % The model rows a linearization PROBE carries: the ones that
    % decide whether a homotopy class is reachable at trajectory scale
    % - the tail and terminal rows among them, since a class whose
    % terminal state admits no braking tail is not reachable at all.
    % The friction polygons are dropped there - they shape the plan,
    % not the class, and their facets are what an active-set kernel
    % walks along (measured 709 pivots against 16 interior-point
    % iterations once they are gone). The hard program re-imposes them.
    common.probeRow = common.modelFamily ~= "friction";

    [inputLowerBound, inputUpperBound] = localInputBox(model);
    [tailLowerBound, tailUpperBound] = localTailBox(model, layout);
    common.lowerBound = [inputLowerBound; tailLowerBound; ...
        zeros(horizonSteps, 1)];
    common.upperBound = [inputUpperBound; tailUpperBound; ...
        inf(horizonSteps, 1)];
    [~, inputScale] = localInputWeightAndScale(cfg);
    common.variableScale = [repmat(inputScale, horizonSteps, 1); ...
        repmat(inputScale(2), tailSteps, 1); ones(horizonSteps, 1)];
    % The homotopy probe: built here so the controller's ladder can
    % start from it (a local function is private to its file).
    common.probeInput = localCruiseProbeInput(model, prediction, common);
end

function nominalState = localNominalState(prediction, model, plan)
% The linearization trajectory at every node: the head by the stage
% rollout, the tail by its condensed map from the terminal state.
    inputPlan = reshape(plan(1:prediction.inputCount), ...
        model.inputDimension, model.horizonSteps);
    nominalState = zeros(6, prediction.nodeCount);
    nominalState(:, 1:prediction.headNodeCount) = ltvBicycleRollout( ...
        prediction.stageMatrixA, prediction.stageMatrixB, ...
        prediction.stageAffine, model.initialEgoState, inputPlan);
    for nodeIdx = prediction.tailNodeIndex
        nominalState(:, nodeIdx) = prediction.egoStateMatrix(:, :, nodeIdx) ...
            * plan+prediction.egoStateOffset(:, nodeIdx);
    end
end

function terminal = localTerminalBudget(model, prediction, speedInterval)
% THE TERMINAL SET's data for this sample (TWO_STAGE_SAFETY.md, "The
% terminal set"): the closed-form lateral certificate at the route's
% curvature maximum, the constant yaw radius and lateral drift every
% tail row charges, and the terminal rows' net bounds - the certified
% heading bound less the course contributions of the terminal lateral
% velocity (|vy|/vx at the terminal speed floor) and of the terminal
% yaw-rate error over the yaw mode's time constant, each net of the
% estimate radius at the terminal node.
    cfg = model.cfg;
    curvatureMaximum = max(abs(model.lane.segmentCurvature));
    certificate = terminalLateralCertificate(cfg, curvatureMaximum);
    terminalNode = prediction.headNodeCount;
    radii = prediction.egoStateErrorBound(:, terminalNode);
    speedFloor = max(speedInterval.plannedLower(terminalNode) ...
        - radii(4), cfg.model.speedMinimum);
    lateralVelocityBound = cfg.terminal.lateralVelocityMaximum;
    yawRateErrorBound = cfg.terminal.yawRateErrorMaximum;
    courseRadius = lateralVelocityBound/speedFloor ...
        + yawRateErrorBound*certificate.yawTimeConstant;
    headingRow = certificate.headingBound-courseRadius-radii(3);
    lateralVelocityRow = lateralVelocityBound-radii(5);
    yawRateRow = yawRateErrorBound-radii(6)-curvatureMaximum*radii(4);
    if headingRow <= 0.0 || lateralVelocityRow <= 0.0 || yawRateRow <= 0.0
        error("collisionAvoidanceController:invalidFormulation", ...
            "The terminal rows have no budget left: heading %.4f rad, " ...
            + "lateral velocity %.4f m/s, yaw-rate error %.4f rad/s " ...
            + "after the course contributions and the estimate radii " ...
            + "at the terminal node.", headingRow, lateralVelocityRow, ...
            yawRateRow);
    end
    terminal = struct( ...
        "certificate", certificate, ...
        "headingRadius", certificate.headingBound, ...
        "lateralDrift", certificate.gammaLateral*certificate.headingBound, ...
        "courseRadius", courseRadius, ...
        "headingRow", headingRow, ...
        "lateralVelocityRow", lateralVelocityRow, ...
        "yawRateRow", yawRateRow, ...
        "terminalNode", terminalNode);
end

function [matrix, bound] = localTailRows(prediction, model, layout)
% HARD rows of the kinematic braking tail (kinematicBrakingTail): the
% nonnegative speed at every tail node and REST - zero speed - at the
% last one. The box [-a_b, 0] and the zero last acceleration are bounds
% (localTailBox). Speeds are nondimensionalized by the speed maximum.
    cfg = model.cfg;
    tailSteps = layout.tailSteps;
    speedMaximum = cfg.model.speedMaximum;
    rowCount = tailSteps+1;
    matrix = zeros(rowCount, layout.decisionCount);
    bound = zeros(rowCount, 1);
    rowIdx = 0;
    for stageIdx = 1:tailSteps
        nodeIdx = prediction.tailNodeIndex(stageIdx);
        speedRow = prediction.egoStateMatrix(4, :, nodeIdx);
        speedOffset = prediction.egoStateOffset(4, nodeIdx);
        rowIdx = rowIdx+1;
        matrix(rowIdx, layout.planIndex) = -speedRow/speedMaximum;
        bound(rowIdx) = speedOffset/speedMaximum;
    end
    restNode = prediction.tailNodeIndex(end);
    rowIdx = rowIdx+1;
    matrix(rowIdx, layout.planIndex) = ...
        prediction.egoStateMatrix(4, :, restNode)/speedMaximum;
    bound(rowIdx) = -prediction.egoStateOffset(4, restNode)/speedMaximum;
end

function [lowerBound, upperBound] = localTailBox(model, layout)
% The tail accelerations lie in [-a_b, 0]; the last one is zero, so
% that rest is rest under the appended zero input of the shifted
% candidate.
    lowerBound = repmat(-model.cfg.terminal.backupDeceleration, ...
        layout.tailSteps, 1);
    upperBound = zeros(layout.tailSteps, 1);
    lowerBound(end) = 0.0;
end

function [matrix, bound] = localTerminalRows( ...
        prediction, ~, layout, terminal)
% HARD handoff rows at the terminal node, the conditions under which
% the tail's kinematic lateral certificate applies to the dynamic
% bicycle's terminal state: |ePsi_N| within the certified heading
% budget, |vy_N| within the declared lateral-velocity bound and the
% yaw-rate error |r_N - kappa_N vx_N| within its bound, each net of
% the estimate radii (localTerminalBudget). Nondimensional by the
% bounds.
    node = terminal.terminalNode;
    stateMatrix = prediction.egoStateMatrix(:, :, node);
    stateOffset = prediction.egoStateOffset(:, node);
    curvature = prediction.scheduleCurvature(node);
    rows = { ...
        stateMatrix(3, :), stateOffset(3), terminal.headingRow; ...
        stateMatrix(5, :), stateOffset(5), terminal.lateralVelocityRow; ...
        stateMatrix(6, :)-curvature*stateMatrix(4, :), ...
            stateOffset(6)-curvature*stateOffset(4), terminal.yawRateRow};
    matrix = zeros(2*size(rows, 1), layout.decisionCount);
    bound = zeros(2*size(rows, 1), 1);
    for rowIdx = 1:size(rows, 1)
        valueRow = rows{rowIdx, 1};
        valueOffset = rows{rowIdx, 2};
        limit = rows{rowIdx, 3};
        matrix(2*rowIdx-1, layout.planIndex) = valueRow/limit;
        bound(2*rowIdx-1) = (limit-valueOffset)/limit;
        matrix(2*rowIdx, layout.planIndex) = -valueRow/limit;
        bound(2*rowIdx) = (limit+valueOffset)/limit;
    end
end

function equilibrium = localCruiseEquilibriumProfile(model, prediction)
% The cruise equilibrium of the declared model at every node's
% schedule curvature and the published longitudinal bias: the state
% the CLF error is measured against and the input the objective is
% measured against. A function of the route and the estimator only.
    nodeCount = model.horizonSteps+1;
    equilibrium = struct( ...
        "input", zeros(model.inputDimension, model.horizonSteps), ...
        "lateralVelocity", zeros(1, nodeCount), ...
        "yawRate", zeros(1, nodeCount));
    for nodeIdx = 1:nodeCount
        point = cruiseEquilibrium(model.referenceSpeed, ...
            prediction.scheduleCurvature(nodeIdx), ...
            model.longitudinalAccelerationBias, model.cfg);
        equilibrium.lateralVelocity(nodeIdx) = point.lateralVelocity;
        equilibrium.yawRate(nodeIdx) = point.yawRate;
        if nodeIdx <= model.horizonSteps
            equilibrium.input(:, nodeIdx) = [point.steeringAngle; ...
                point.longitudinalAcceleration];
        end
    end
end

function [centre, halfWidth] = localReachBox(model, layout)
% The box every admissible input plan lies in. Each head input may range
% independently over its physical magnitude bounds; the friction rows
% are not included. The largest margin over this relaxation bounds the
% largest margin any admissible plan reaches - the cheap side of the
% gate and a sound certificate.
    cfg = model.cfg;
    [accelerationMinimum, accelerationMaximum] = ...
        longitudinalAccelerationBounds(cfg);
    lower = [-cfg.model.frontWheelSteeringAngleMaximum; accelerationMinimum];
    upper = [cfg.model.frontWheelSteeringAngleMaximum; accelerationMaximum];
    centre = repmat((lower+upper)/2.0, layout.horizonSteps, 1);
    halfWidth = repmat((upper-lower)/2.0, layout.horizonSteps, 1);
    % The tail's box, [-a_b, 0] at every tail stage: a relaxation of the
    % tail's admissible set exactly as the head's box is of the head's.
    [tailLower, tailUpper] = localTailBox(model, layout);
    centre = [centre; (tailLower+tailUpper)/2.0];
    halfWidth = [halfWidth; (tailUpper-tailLower)/2.0];
end

function inputPlan = localCruiseProbeInput(model, prediction, common)
% THE PROBE that starts the homotopy escape: the trajectory the
% vehicle would follow if the target were not there.
%
% It is the controller's own cruise law - the CLF certificate's LQR
% gain about the cruise equilibrium, u_k = u_eq,k - K e_k - rolled out
% from the MEASURED state through the declared stage map and clipped into
% the input box. Nothing about the target enters it: it is a
% function of the route, the measured state, the published bias and
% the declared model. It is never executed and never committed; it is
% a point at which to evaluate stage 1.
%
% Its value is that it goes THROUGH the configuration obstacle whenever
% the target blocks the route, and the interior extension of stage 1
% answers a penetrating point with the outward normal of the
% least-penetrated EDGE of that polygon. The configuration obstacle of
% two cars is about 10 m long and 4 m wide, and the probe enters it
% from behind, so from the first node that is more than half its width
% past the rear edge the least-penetrated edge is a LATERAL one. Those are the rows that credit steering, and they
% are how the ladder leaves the braking class - with no side named
% anywhere, the sign being the sign of the probe's own lateral offset
% from the target's centre line.
    cfg = model.cfg;
    horizonSteps = model.horizonSteps;
    gain = common.certificate.feedbackGain;
    [accelerationMinimum, accelerationMaximum] = ...
        longitudinalAccelerationBounds(cfg);
    lower = [-cfg.model.frontWheelSteeringAngleMaximum; accelerationMinimum];
    upper = [cfg.model.frontWheelSteeringAngleMaximum; accelerationMaximum];
    inputPlan = zeros(model.inputDimension, horizonSteps);
    state = model.initialEgoState;
    for stageIdx = 1:horizonSteps
        error = state(2:6)-[0.0; 0.0; model.referenceSpeed; ...
            common.equilibrium.lateralVelocity(stageIdx); ...
            common.equilibrium.yawRate(stageIdx)];
        input = common.equilibrium.input(:, stageIdx)-gain*error;
        input = min(max(input, lower), upper);
        inputPlan(:, stageIdx) = input;
        state = prediction.stageMatrixA(:, :, stageIdx)*state ...
            + prediction.stageMatrixB(:, :, stageIdx)*input ...
            + prediction.stageAffine(:, stageIdx);
    end
    % The probe's tail: the fastest braking profile from its terminal
    % speed. A nominal only - the
    % elastic program decides its own tail.
    tail = kinematicBrakingTail("profile", cfg, prediction.tailSteps, ...
        min(max(state(4), 0.0), cfg.model.speedMaximum));
    inputPlan = [inputPlan(:); tail(:)];
end

function node = localEmptyObstacleNode(inputCount)
    node = struct( ...
        "covered", false, ...
        "imposed", false, ...
        "normal", zeros(2, 1), ...
        "facetNormal", zeros(2, 8), ...
        "facetCount", 0, ...
        "regionCode", 0, ...
        "supportValue", 0.0, ...
        "targetSupport", 0.0, ...
        "egoSupport", 0.0, ...
        "headingCoefficient", 0.0, ...
        "nominalHeading", 0.0, ...
        "tightening", 0.0, ...
        "dualDistance", inf, ...
        "nominalMargin", inf, ...
        "incumbentMargin", inf, ...
        "boxMaximum", inf, ...
        "capacity", 0.0, ...
        "authority", 0.0, ...
        "window", false, ...
        "allowance", 0.0, ...
        "elastic", false, ...
        "terminalInvariant", false, ...
        "terminalContinuationAxis", "", ...
        "terminalFutureSupport", inf, ...
        "terminalSupportDirection", zeros(2, 1), ...
        "terminalSegmentEnforced", false, ...
        "terminalSegmentIndex", 0, ...
        "terminalStationLower", -inf, ...
        "terminalStationUpper", inf, ...
        "outside", true, ...
        "factSource", "", ...
        "marginMatrix", zeros(2, inputCount), ...
        "marginOffset", zeros(2, 1));
end

function family = localEmptyFamily(name, id, nodeCount, inputCount)
    family = struct("name", string(name), "id", string(id), ...
        "nodes", repmat(localEmptyObstacleNode(inputCount), ...
            nodeCount, 1));
end

function family = localTargetFamily(model, prediction, nominalState, ...
        tightening, terminal)
% STAGE 1 on the TRUE configuration obstacle. At every node the exact
% Minkowski sum of the target rectangle at its predicted pose and the
% ego rectangle at the linearization's pose - both in path coordinates,
% both as oriented rectangles, neither replaced by a bounding box -
% and the exact projection of the linearization's centre onto it
% (rectangleConfigurationDistance): the signed distance, the
% supporting normal n_k, and the polygon's support in that direction.
% Outside, n_k points from the closest boundary point to the centre;
% inside - the probe that has run into the obstacle - it is the outward
% normal of the least-penetrated edge, the same minimum-penetration
% answer for a polygon of any shape. Node 1 (k = 0, the measured
% state) is evaluated for the report and carries no row.
%
% The support is split back into the two rectangles' own supports,
% because stage 2 needs them separately: the target's is a constant of
% the node, the ego's moves with the decision's heading. Each is taken
% as the MAXIMUM over the declared yaw uncertainty of that rectangle
% (localRectangleSupport), which is the exact directional treatment of
% the yaw radii - the retired box formulation charged them as extra
% half-extents of an axis-aligned box instead.
%
% TAIL NODES carry the same row with a CONSTANT ego support: the tail's
% lane-keeping law keeps the ego's heading within the certified band
% about zero (terminalLateralCertificate), so the ego's support is
% maximized over that band once and the row has no heading term, and
% the band's lateral drift is in the node's tightening. The facet set
% is still taken at the linearization's terminal heading.
    nodeCount = prediction.nodeCount;
    family = localEmptyFamily("collision", model.targetKey, nodeCount, ...
        prediction.planCount);
    if ~model.hasTarget
        return;
    end
    target = model.targetPath;
    horizon = model.targetHorizon;
    halfDimensions = [model.egoHalfLength; model.egoHalfWidth; ...
        model.targetHalfLength; model.targetHalfWidth];
    for nodeIdx = 1:nodeCount
        nominal = nominalState(:, nodeIdx);
        centre = [target.station(nodeIdx); target.lateral(nodeIdx)];
        targetHeading = target.headingError(nodeIdx);
        % The configuration obstacle about the TARGET CENTRE: path
        % stations run to hundreds of metres, and the projection is
        % done in metres of relative position.
        [distance, normal, ~, outside, facetNormal] = ...
            rectangleConfigurationDistance( ...
                nominal(1:2)-centre, nominal(3), zeros(2, 1), ...
                targetHeading, halfDimensions);
        [egoYaw, egoYawRadius, slopeRange] = localEgoYawData( ...
            model, prediction, terminal, nodeIdx, nominal(3));
        targetYawRadius = horizon.targetYawErrorBound(nodeIdx);
        node = localEmptyObstacleNode(prediction.planCount);
        node.covered = true;
        node.normal = normal;
        % The obstacle's own facets: the disjuncts of the exact
        % collision-free condition at this node.
        node.facetCount = size(facetNormal, 2);
        node.facetNormal(:, 1:node.facetCount) = facetNormal;
        node.regionCode = localRegionCode(normal);
        node.targetSupport = localRectangleSupport( ...
            model.targetHalfLength, model.targetHalfWidth, normal, ...
            targetHeading, targetYawRadius);
        node.egoSupport = localRectangleSupport( ...
            model.egoHalfLength, model.egoHalfWidth, normal, ...
            egoYaw, egoYawRadius);
        % n' z <= supportValue for every z of the obstacle, the ego's
        % part evaluated at the linearization's heading (or over the
        % tail's band).
        node.supportValue = normal.'*centre+node.targetSupport ...
            + node.egoSupport;
        node.headingCoefficient = localSupportSlopeBoundOnRange( ...
            model, normal, slopeRange);
        node.nominalHeading = nominal(3);
        node.tightening = tightening.base(nodeIdx);
        node.dualDistance = distance;
        node.nominalMargin = normal.'*nominal(1:2)-node.supportValue ...
            - node.tightening;
        node.outside = outside;
        family.nodes(nodeIdx) = node;
    end
    if model.cfg.certification.enabled
        family.nodes(nodeCount) = localTerminalInvariantTargetNode( ...
            model, prediction, nominalState(:, nodeCount), ...
            family.nodes(nodeCount), terminal);
    end
end

function node = localTerminalInvariantTargetNode( ...
        model, prediction, nominalState, node, terminal)
% Hard target-conditioned terminal condition. For each fixed inertial
% direction, the target predictor supplies the support of its COMPLETE
% future centre trajectory from the rest time. The best terminal
% half-space on the fixed direction grid separates the resting ego from
% that entire prediction. Rectangle circumradii cover every future target
% and ego yaw. No stationary, straight, monotone, curvature, or target
% motion-class predicate is used.
%
% The Frenet-to-Cartesian map is exactly affine on one lane polyline
% segment. Two additional hard rows keep the terminal station on the
% segment selected at the incumbent, so the inertial separating row is
% valid for every admitted decision and exact at the shifted incumbent.
    restNode = prediction.nodeCount;
    tolerance = model.cfg.certification.shiftConsistencyTolerance;
    [segmentIdx, affineOrigin, tangent, lateralAxis, stationLower, ...
        stationUpper] = localTerminalLaneFrame( ...
            model.lane, nominalState(1), tolerance);
    egoCentre = affineOrigin+tangent*nominalState(1) ...
        + lateralAxis*nominalState(2);
    direction = model.targetHorizon.terminalSupportDirection;
    futureSupport = ...
        model.targetHorizon.terminalFuturePositionSupport;
    targetRadius = hypot(model.targetHalfLength, model.targetHalfWidth);
    egoRadius = hypot(model.egoHalfLength, model.egoHalfWidth);
    coefficient = [direction.'*tangent, direction.'*lateralAxis];
    baseTightening = model.cfg.collision.clearanceMargin ...
        + sum(model.targetHorizon.targetPositionErrorBound(:, restNode)) ...
        + terminal.lateralDrift;
    egoError = prediction.egoStateErrorBound(1:2, restNode);
    directionTightening = baseTightening ...
        + abs(coefficient)*egoError;
    margin = direction.'*egoCentre-futureSupport.' ...
        - targetRadius-egoRadius-directionTightening;
    margin(~isfinite(futureSupport.')) = -inf;
    [~, directionIdx] = max(margin);
    invariantModel = any(isfinite(futureSupport));
    if invariantModel
        inertialNormal = direction(:, directionIdx);
        selectedSupport = futureSupport(directionIdx);
        axisName = "predictedTrajectorySupport";
    else
        inertialNormal = direction(:, 1);
        selectedSupport = 0.0;
        axisName = "noFinitePredictedTrajectorySupport";
    end
    normal = [dot(inertialNormal, tangent); ...
        dot(inertialNormal, lateralAxis)];
    node.normal = normal;
    node.regionCode = localRegionCode(normal);
    node.targetSupport = targetRadius;
    node.egoSupport = egoRadius;
    node.supportValue = selectedSupport+targetRadius+egoRadius ...
        - inertialNormal.'*affineOrigin;
    node.headingCoefficient = 0.0;
    node.nominalHeading = nominalState(3);
    node.tightening = baseTightening+abs(normal).'*egoError;
    node.dualDistance = normal.'*nominalState(1:2)-node.supportValue;
    node.nominalMargin = node.dualDistance-node.tightening;
    node.outside = node.nominalMargin >= 0.0;
    node.terminalInvariant = invariantModel;
    node.terminalContinuationAxis = axisName;
    node.terminalFutureSupport = selectedSupport;
    node.terminalSupportDirection = inertialNormal;
    node.terminalSegmentEnforced = true;
    node.terminalSegmentIndex = segmentIdx;
    node.terminalStationLower = stationLower;
    node.terminalStationUpper = stationUpper;
end

function [segmentIdx, affineOrigin, tangent, lateralAxis, ...
        stationLower, stationUpper] = localTerminalLaneFrame( ...
        lane, station, tolerance)
    segmentIdx = find(lane.segmentStation <= station, 1, "last");
    if isempty(segmentIdx)
        segmentIdx = 1;
    end
    segmentIdx = min(segmentIdx, numel(lane.segmentLength));
    stationLower = lane.segmentStation(segmentIdx);
    stationUpper = stationLower+lane.segmentLength(segmentIdx);
    if segmentIdx < numel(lane.segmentLength)
        guard = max(10.0*tolerance*(1.0+abs(stationUpper)), ...
            100.0*eps(1.0+abs(stationUpper)));
        guard = min(guard, 0.25*lane.segmentLength(segmentIdx));
        stationUpper = stationUpper-guard;
    end
    tangent = lane.tangent(segmentIdx, :).';
    lateralAxis = [-tangent(2); tangent(1)];
    affineOrigin = lane.segmentStart(segmentIdx, :).' ...
        - tangent*stationLower;
end

function [egoYaw, egoYawRadius, slopeRange] = localEgoYawData( ...
        model, prediction, terminal, nodeIdx, nominalHeading)
% What the ego's yaw is at a node for the support rows. Head nodes: the
% linearization's heading with its estimate radius, and the heading
% interval the program admits for the Lipschitz charge. Tail nodes: the
% certified band about zero, and no Lipschitz charge (an empty range),
% since the tail's heading is not a decision.
    if nodeIdx <= prediction.headNodeCount
        egoYaw = nominalHeading;
        egoYawRadius = prediction.egoStateErrorBound(3, nodeIdx);
        slopeRange = localHeadingRange(model, prediction, nodeIdx, ...
            nominalHeading);
    else
        egoYaw = 0.0;
        egoYawRadius = terminal.headingRadius;
        slopeRange = zeros(1, 0);
    end
end

function slope = localSupportSlopeBoundOnRange(model, normal, range)
    if isempty(range)
        slope = 0.0;
        return;
    end
    slope = localSupportSlopeBound(model.egoHalfLength, ...
        model.egoHalfWidth, normal, range(1), range(2));
end

function node = localEmptyDisjunctionNode(layout)
    node = struct( ...
        "covered", false, ...
        "imposed", false, ...
        "certifiedInfeasible", false, ...
        "facetCount", 0, ...
        "facetNormal", zeros(2, 0), ...
        "facetMatrix", zeros(0, layout.planCount), ...
        "facetOffset", zeros(0, 1), ...
        "facetMaximum", zeros(1, 0), ...
        "facetRegion", zeros(1, 0), ...
        "nodeMaximum", inf, ...
        "allowance", 0.0, ...
        "window", false, ...
        "factSource", "");
end

function disjunction = localDisjunctionData(model, prediction, common, ...
        collisionCfg, family, nominalState)
% THE DISJUNCTIVE MODEL of the collision constraint, node by node.
%
% The ego rectangle misses the target rectangle exactly when the ego
% centre lies outside the configuration obstacle, and a convex polygon
% is the intersection of its facet half-planes, so
%
%   outside  <=>  OR over facets j of  ( n_j'p >= h(n_j) ),
%
% with no facet left out: by the separating-axis theorem for two convex
% polygons, if they are disjoint then one of the two rectangles' own
% edge normals separates them, and those are exactly the polygon's
% facet normals (rectangleConfigurationDistance). This function builds
% every disjunct - for each facet the same pair of affine rows stage 2
% builds for one direction, through the two rectangles' support
% functions with the ego's yaw charged by its exact Lipschitz bound -
% and leaves the choice among them to the search.
%
% That is the difference from a single frozen normal. One normal is a
% sufficient condition whose choice is made by the linearization, so
% the program can only return plans of that normal's homotopy class;
% the OR is the exact condition, and which disjunct holds at each node
% is decided by the optimization. At the linearization's own heading
% the disjunction is NECESSARY AND SUFFICIENT for collision freedom -
% it is the polygon - so the only inexactness left in the model is the
% yaw charge, which is zero at the linearization.
%
% The gate is the same as elsewhere, taken over the disjunction: a node
% is imposed when SOME facet is satisfiable by some plan in the
% input reach box, a fact when none is and the shortfall is
% within the node.s allowance, and otherwise the candidate is certified
% infeasible - no admissible plan can be outside the obstacle there at
% all.
    layout = common.layout;
    nodeCount = numel(family.nodes);
    disjunction = repmat(localEmptyDisjunctionNode(layout), nodeCount, 1);
    if ~model.hasTarget
        return;
    end
    target = model.targetPath;
    horizon = model.targetHorizon;
    disturbanceBound = collisionCfg.disturbanceBound;
    windowBound = collisionCfg.factWindowFactor*disturbanceBound;
    windowTop = localWindowTop(family, prediction, common, windowBound);
    for nodeIdx = 1:nodeCount
        source = family.nodes(nodeIdx);
        if ~source.covered
            continue;
        end
        entry = localEmptyDisjunctionNode(layout);
        entry.covered = true;
        entry.facetCount = source.facetCount;
        entry.facetNormal = source.facetNormal(:, 1:source.facetCount);
        entry.facetMatrix = zeros(2*entry.facetCount, layout.planCount);
        entry.facetOffset = zeros(2*entry.facetCount, 1);
        entry.facetMaximum = zeros(1, entry.facetCount);
        entry.facetRegion = zeros(1, entry.facetCount);
        nominal = nominalState(:, nodeIdx);
        centre = [target.station(nodeIdx); target.lateral(nodeIdx)];
        stateMatrix = prediction.egoStateMatrix(:, :, nodeIdx);
        stateOffset = prediction.egoStateOffset(:, nodeIdx);
        [egoYaw, egoYawRadius, slopeRange] = localEgoYawData( ...
            model, prediction, common.terminal, nodeIdx, nominal(3));
        targetYawRadius = horizon.targetYawErrorBound(nodeIdx);
        for facetIdx = 1:entry.facetCount
            normal = entry.facetNormal(:, facetIdx);
            targetSupport = localRectangleSupport( ...
                model.targetHalfLength, model.targetHalfWidth, normal, ...
                target.headingError(nodeIdx), targetYawRadius);
            egoSupport = localRectangleSupport( ...
                model.egoHalfLength, model.egoHalfWidth, normal, ...
                egoYaw, egoYawRadius);
            slope = localSupportSlopeBoundOnRange(model, normal, ...
                slopeRange);
            supportValue = normal.'*centre+targetSupport+egoSupport;
            rows = 2*(facetIdx-1)+(1:2);
            for signIdx = 1:2
                secantSign = 3.0-2.0*signIdx;
                entry.facetMatrix(rows(signIdx), :) = ...
                    normal.'*stateMatrix(1:2, :) ...
                    - secantSign*slope*stateMatrix(3, :);
                entry.facetOffset(rows(signIdx)) = ...
                    normal.'*stateOffset(1:2) ...
                    - secantSign*slope*(stateOffset(3)-nominal(3)) ...
                    - supportValue-source.tightening;
            end
            entry.facetMaximum(facetIdx) = localBoxMaximum( ...
                entry.facetMatrix(rows, :), entry.facetOffset(rows), ...
                common.reachCentre, common.reachHalfWidth);
            entry.facetRegion(facetIdx) = localRegionCode(normal);
        end
        entry.nodeMaximum = max(entry.facetMaximum);
        entry.window = source.window;
        entry.allowance = 0.0;
        if entry.window
            entry.allowance = (windowTop-nodeIdx+1)*disturbanceBound;
        end
        if nodeIdx == 1 || ~any(entry.facetMatrix(:))
            entry.factSource = "constant";
        elseif entry.nodeMaximum >= 0.0
            entry.imposed = true;
        elseif entry.window && entry.nodeMaximum >= -entry.allowance
            entry.factSource = "unsatisfiable";
        else
            entry.factSource = "infeasible";
            entry.certifiedInfeasible = true;
        end
        disjunction(nodeIdx) = entry;
    end
end

function value = localRectangleSupport(halfLength, halfWidth, normal, ...
        yaw, yawRadius)
% The support of an oriented rectangle about its centre, in the unit
% direction n, maximized over the declared yaw uncertainty:
%
%   h(psi) = l |cos(alpha - psi)| + w |sin(alpha - psi)|,
%   value  = max_{|delta| <= r} h(yaw + delta),   alpha = angle(n).
%
% h is pi-periodic and, between its kinks at multiples of pi/2, equals
% R cos(theta -+ phi) with R = hypot(l, w) and phi = atan2(w, l): its
% maxima are at theta = +-phi + k pi, where it reaches R. The maximum
% over an interval is therefore the larger endpoint value, or R when a
% maximizer falls inside. Exact, and the direction-wise treatment of a
% yaw radius that an axis-aligned box can only approximate.
    alpha = atan2(normal(2), normal(1));
    low = alpha-yaw-yawRadius;
    high = alpha-yaw+yawRadius;
    value = max(localSupportAtAngle(halfLength, halfWidth, low), ...
        localSupportAtAngle(halfLength, halfWidth, high));
    if yawRadius <= 0.0
        return;
    end
    phase = atan2(halfWidth, halfLength);
    for phaseSign = [-1.0, 1.0]
        maximizer = phaseSign*phase ...
            + pi*ceil((low-phaseSign*phase)/pi);
        if maximizer <= high
            value = hypot(halfLength, halfWidth);
            return;
        end
    end
end

function value = localSupportAtAngle(halfLength, halfWidth, angle)
    value = halfLength*abs(cos(angle))+halfWidth*abs(sin(angle));
end

function slope = localSupportSlopeBound(halfLength, halfWidth, normal, ...
        yawLow, yawHigh)
% THE CONSERVATIVE AFFINE BOUND ON THE EGO'S YAW. Stage 2 freezes n but
% not the ego's heading, which the decision moves; the support
% h(psi) = l |cos(alpha - psi)| + w |sin(alpha - psi)| is not affine in
% psi, so the row charges
%
%   h(psi) <= h(psibar) + slope * |psi - psibar|,
%
% two affine rows (sigma = +-1) whose minimum is the bound. This
% returns the exact Lipschitz constant of h over the interval the
% program admits: max |dh/dpsi| there. With theta = alpha - psi,
% dh/dtheta = -l sgn(cos) sin + w sgn(sin) cos is sinusoidal of
% amplitude R = hypot(l, w) between the kinks at multiples of pi/2, so
% its magnitude is maximized at an interval endpoint, at a kink (where
% it is l or w) or at the sinusoid's own peak (where it is R) when that
% falls inside. Exact, hence never worse than the retired box bound
% (which charged w along the station axis and l along the lateral one,
% and their sum for a diagonal normal); for a diagonal normal at the
% declared heading domain it is measurably smaller.
    alpha = atan2(normal(2), normal(1));
    low = alpha-yawHigh;
    high = alpha-yawLow;
    radius = hypot(halfLength, halfWidth);
    if ~(high > low)
        slope = radius;
        return;
    end
    kinks = ceil(low/(pi/2))*(pi/2):(pi/2):high;
    edges = unique([low, kinks, high]);
    slope = 0.0;
    for pieceIdx = 1:numel(edges)-1
        pieceLow = edges(pieceIdx);
        pieceHigh = edges(pieceIdx+1);
        middle = 0.5*(pieceLow+pieceHigh);
        cosineSign = localNonzeroSign(cos(middle));
        sineSign = localNonzeroSign(sin(middle));
        % dh/dtheta = cosineCoefficient cos(theta) + sineCoefficient sin(theta)
        cosineCoefficient = halfWidth*sineSign;
        sineCoefficient = -halfLength*cosineSign;
        value = max( ...
            abs(cosineCoefficient*cos(pieceLow) ...
                + sineCoefficient*sin(pieceLow)), ...
            abs(cosineCoefficient*cos(pieceHigh) ...
                + sineCoefficient*sin(pieceHigh)));
        peak = atan2(sineCoefficient, cosineCoefficient);
        for shift = -2:2
            candidate = peak+shift*pi;
            if candidate > pieceLow && candidate < pieceHigh
                value = radius;
            end
        end
        slope = max(slope, value);
    end
end

function value = localNonzeroSign(value)
    if value < 0.0
        value = -1.0;
    else
        value = 1.0;
    end
end

function range = localHeadingRange(model, prediction, nodeIdx, ...
        nominalHeading)
% The interval the ego's heading error can take at this node: the
% declared heading domain widened by the node's estimate radius, and
% by the linearization's own heading (a probe may sit outside the
% domain). The slope bound must hold over every value the program
% admits AND at the point it is anchored.
    bound = model.cfg.model.headingDomainRadius ...
        + prediction.egoStateErrorBound(3, nodeIdx);
    range = [min(-bound, nominalHeading), max(bound, nominalHeading)];
end

function code = localRegionCode(normal)
% Which way the separating direction faces, for the diagnostics and for
% the escape trigger: 1 behind, 2 ahead, 3 left, 4 right, 5 rear-left,
% 6 rear-right, 7 front-left, 8 front-right. The tolerance is angular
% (about 5 degrees) rather than exact: on the true configuration
% polygon the edge normals carry the ego's and the target's own
% headings, so a normal is almost never exactly axis-aligned, and what
% the trigger asks is whether a row has any lateral coefficient worth
% the name.
    tolerance = 0.087;
    if abs(normal(2)) <= tolerance
        code = 1+double(normal(1) > 0.0);
    elseif abs(normal(1)) <= tolerance
        code = 3+double(normal(2) < 0.0);
    else
        code = 5+2*double(normal(1) > 0.0)+double(normal(2) < 0.0);
    end
end

function tightening = localTargetTightening(model, prediction, terminal)
% Per-node budget of the target rows, charged in the direction of
% separation: the clearance margin (the executable d_min), the ego and
% target position radii, and the curvature sagitta - the largest
% deviation of a rectangle edge from the chord between its corners when
% the path frame is curved, 1/2 kappa_max (l_e^2 + l_T^2). The YAW
% radii are not here: they are taken directionally, inside the support
% of each rectangle (localRectangleSupport), which is exact where an
% inflated bounding box was conservative. Tail nodes add the lateral
% drift of the certified band, gamma_d times the heading bound.
    nodeCount = prediction.nodeCount;
    tightening = struct("base", zeros(1, nodeCount));
    if ~model.hasTarget
        return;
    end
    horizon = model.targetHorizon;
    sagitta = 0.5*max(abs(model.lane.segmentCurvature)) ...
        * (model.egoHalfLength^2+model.targetHalfLength^2);
    for nodeIdx = 1:nodeCount
        egoRadii = prediction.egoStateErrorBound(:, nodeIdx);
        positionRadius = egoRadii(1) ...
            + sum(horizon.targetPositionErrorBound(:, nodeIdx));
        tightening.base(nodeIdx) = model.cfg.collision.clearanceMargin ...
            + positionRadius+sagitta;
        if nodeIdx > prediction.headNodeCount
            tightening.base(nodeIdx) = tightening.base(nodeIdx) ...
                + terminal.lateralDrift;
        end
    end
end

function families = localRoadFamilies(model, prediction, nominalState, ...
        terminal)
% STAGE 1 for the road boundaries: each is a half-plane obstacle in
% path coordinates whose dual is trivial - the separating direction is
% the boundary's inward normal, n = [0; -1] for a left boundary (the ego
% stays right of it), n = [0; 1] for a right one, with support
% h_O(n) = -+d_b. The boundary curve is sampled over its finite
% parameter range, projected onto the lane, and its lateral offset
% interpolated at every schedule station. The tightening carries the
% clearance margin, the position and yaw radii, the declared fit budget
% and the boundary's lateral variation over the stations the plan can
% reach about the schedule (the speed domain over the elapsed time). A
% node whose reachable stations are not covered by the finite section
% is uncovered: perceptionLimited boundaries carry no row there, strict
% ones are a declared input error.
    boundaries = model.road.boundaries;
    boundaryCount = numel(boundaries);
    nodeCount = prediction.nodeCount;
    headNodeCount = prediction.headNodeCount;
    families = repmat(localEmptyFamily("road", "", nodeCount, ...
        prediction.planCount), boundaryCount, 1);
    if boundaryCount == 0
        return;
    end
    cfg = model.cfg;
    speedDeviation = cfg.model.speedMaximum-model.plannedSpeedFloor;
    terminalReach = speedDeviation*(headNodeCount-1)*model.sampleTime;
    for boundaryIdx = 1:boundaryCount
        boundary = boundaries(boundaryIdx);
        families(boundaryIdx).id = string(boundary.boundaryId);
        [boundaryStation, boundaryLateral] = localBoundaryPath( ...
            boundary, model.lane);
        if mean(boundaryLateral) > 0.0
            normal = [0.0; -1.0];
        else
            normal = [0.0; 1.0];
        end
        for nodeIdx = 1:nodeCount
            station = prediction.scheduleStation(nodeIdx);
            radii = prediction.egoStateErrorBound(:, nodeIdx);
            if nodeIdx <= headNodeCount
                reach = speedDeviation*(nodeIdx-1)*model.sampleTime ...
                    + radii(1)+model.egoHalfLength;
                range = station+[-reach, reach];
            else
                % A tail node's station lies between the terminal
                % node's reach and that reach advanced by the fastest
                % admissible tail, whatever the schedule's own braking
                % profile did.
                tailTime = (nodeIdx-headNodeCount)*model.sampleTime;
                terminalStation = prediction.scheduleStation(headNodeCount);
                range = [terminalStation-terminalReach, ...
                    terminalStation+terminalReach ...
                        + cfg.model.speedMaximum*tailTime] ...
                    + [-1.0, 1.0]*(radii(1)+model.egoHalfLength);
            end
            tolerance = cfg.road.parameterRangeTolerance;
            if range(2) < boundaryStation(1)-tolerance ...
                    || range(1) > boundaryStation(end)+tolerance
                continue;
            end
            covered = range(1) >= boundaryStation(1)-tolerance ...
                && range(2) <= boundaryStation(end)+tolerance;
            if ~covered
                if boundary.coveragePolicy == "perceptionLimited"
                    continue;
                end
                error("collisionAvoidanceController:" ...
                        + "roadBoundaryCoverageGap", ...
                    "The stations the plan can reach at node %d are " ...
                    + "only partly covered by finite road boundary " ...
                    + "%s. Supply a segment covering them; the " ...
                    + "controller does not extend a fitted curve.", ...
                    nodeIdx, boundary.boundaryId);
            end
            offsetHere = interp1(boundaryStation, boundaryLateral, ...
                min(max(station, boundaryStation(1)), boundaryStation(end)));
            inRange = boundaryStation >= range(1) ...
                & boundaryStation <= range(2);
            variation = max(abs(boundaryLateral(inRange)-offsetHere));
            if isempty(variation)
                variation = 0.0;
            end
            nominal = nominalState(:, nodeIdx);
            [egoYaw, egoYawRadius, slopeRange] = localEgoYawData( ...
                model, prediction, terminal, nodeIdx, nominal(3));
            node = localEmptyObstacleNode(prediction.planCount);
            node.covered = true;
            node.normal = normal;
            node.regionCode = localRegionCode(normal);
            % The same geometry as the target family: the ego's own
            % support in the boundary's inward normal, at the
            % linearization's heading and over its yaw radius, with the
            % exact slope bound for the decision's heading (over the
            % certified band, with no slope, at a tail node).
            node.egoSupport = localRectangleSupport( ...
                model.egoHalfLength, model.egoHalfWidth, normal, ...
                egoYaw, egoYawRadius);
            node.supportValue = normal(2)*offsetHere+node.egoSupport;
            node.headingCoefficient = localSupportSlopeBoundOnRange( ...
                model, normal, slopeRange);
            node.nominalHeading = nominal(3);
            node.tightening = cfg.collision.clearanceMargin+radii(2) ...
                + boundary.normalDistanceErrorBound+variation;
            if nodeIdx > headNodeCount
                node.tightening = node.tightening+terminal.lateralDrift;
            end
            node.dualDistance = normal.'*nominal(1:2)-node.supportValue;
            node.nominalMargin = node.dualDistance-node.tightening;
            node.outside = node.dualDistance > 0.0;
            families(boundaryIdx).nodes(nodeIdx) = node;
        end
    end
end

function [station, lateral] = localBoundaryPath(boundary, lane)
% Sample the finite local-quadratic boundary and project it onto the
% lane: its stations and lateral offsets, sorted by station.
    sampleCount = max(2, ceil(diff(boundary.parameterRange)/0.5)+1);
    parameter = linspace(boundary.parameterRange(1), ...
        boundary.parameterRange(2), sampleCount);
    station = zeros(1, sampleCount);
    lateral = zeros(1, sampleCount);
    for sampleIdx = 1:sampleCount
        point = boundary.origin ...
            + boundary.longitudinalDirection*parameter(sampleIdx) ...
            + boundary.lateralDirection ...
                * polyval(boundary.coefficients, parameter(sampleIdx));
        projection = laneProjection(point, lane);
        station(sampleIdx) = projection.station;
        lateral(sampleIdx) = projection.lateralPosition;
    end
    [station, order] = sort(station);
    lateral = lateral(order);
    [station, keep] = unique(station, "stable");
    lateral = lateral(keep);
end

% ====================================================================
% Stage 2: the affine separation rows
% ====================================================================

function [families, matrix, bound, rowFamily, rowNode, rowWindow, ...
        rowAllowance, rowSlackNode, certifiedInfeasible] = ...
        localSeparationRows(families, prediction, common, collisionCfg, ...
        obstacleMode, linearizationInput)
% The row of Eq. (13) at every IMPOSED node of every family, with the
% stage-1 dual frozen, as the pair sigma = +-1 (Lemma 1's |ePsi|):
%
%   g^sigma_k(u) = n_k' p_k(u) - c_k sigma ePsi_k(u) - h_k - tau_k >= 0,
%
% p_k, ePsi_k from the condensed prediction, c_k the heading
% coefficient, h_k the support value. marginMatrix / marginOffset hold
% g^sigma_k over the INPUT columns (the plan's linearized margin, what
% is reported); the inequality rows are the same in the kernel's form
% A z <= b.
%
% THE GATE, node by node. The largest margin over the input
% reach box (localBoxMaximum, exact for the box by the minimax
% theorem; the box contains the admissible set) decides: nonnegative,
% the row is imposed; negative by at most the node's ALLOWANCE at a
% node of the near window (authority - the largest change of the
% margin a reach-box step can make, localRowAuthority - below
% factWindowFactor times the disturbance d), the node is a fact,
% evaluated for the report; negative by more, or outside the window,
% no admissible plan satisfies the row and the candidate is certified
% infeasible. The allowance of window node k with top node W is
% (W - k + 1) d: a fact was last a hard row when it stood at the top
% of the window, and one sample of mismatch moves a margin by at most
% d - a wrong-class candidate misses by metres, mismatch by
% centimetres. Node 1 and any node whose rows do not depend on the
% input are facts of the measured state. The friction polygon, which
% the box does not see, can take the capacity away at the near nodes;
% the controller's fact relaxation finds those rows when the program
% says so.
%
% ELASTIC MODE carries every covered node from node 2 on, with no gate
% and no certificate: the ladder reads the slack, not the feasibility,
% and a fact there would only hide how far the iterate still is from
% the class it is moving into. rowSlackNode names, for every TARGET
% row, the node whose slack it shares (0 for road rows, which stay
% hard in both modes).
    layout = common.layout;
    elastic = obstacleMode == "elastic";
    certifiedMode = obstacleMode == "certified";
    referenceInput = min(max(linearizationInput(:), ...
        common.reachCentre-common.reachHalfWidth), ...
        common.reachCentre+common.reachHalfWidth);
    disturbanceBound = collisionCfg.disturbanceBound;
    windowBound = collisionCfg.factWindowFactor*disturbanceBound;
    % The window's top node is a property of the dynamics alone (the
    % authority of the position rows over the reach box), so it is the
    % same for every family: read it off the first covered family.
    windowTop = localWindowTop(families, prediction, common, windowBound);
    totalRows = 0;
    for familyIdx = 1:numel(families)
        totalRows = totalRows+2*sum([families(familyIdx).nodes.covered]);
        totalRows = totalRows ...
            + 2*sum([families(familyIdx).nodes.terminalSegmentEnforced]);
    end
    matrix = zeros(totalRows, layout.decisionCount);
    bound = zeros(totalRows, 1);
    rowFamily = strings(totalRows, 1);
    rowNode = zeros(totalRows, 1);
    rowWindow = false(totalRows, 1);
    rowAllowance = zeros(totalRows, 1);
    rowSlackNode = zeros(totalRows, 1);
    rowIdx = 0;
    certifiedInfeasible = false;
    for familyIdx = 1:numel(families)
        nodes = families(familyIdx).nodes;
        for nodeIdx = 1:numel(nodes)
            node = nodes(nodeIdx);
            if ~node.covered
                continue;
            end
            stateMatrix = prediction.egoStateMatrix(:, :, nodeIdx);
            stateOffset = prediction.egoStateOffset(:, nodeIdx);
            positionRow = node.normal.'*stateMatrix(1:2, :);
            positionOffset = node.normal.'*stateOffset(1:2);
            for signIdx = 1:2
                secantSign = 3.0-2.0*signIdx;
                node.marginMatrix(signIdx, :) = positionRow ...
                    - secantSign*node.headingCoefficient*stateMatrix(3, :);
                node.marginOffset(signIdx) = positionOffset ...
                    - secantSign*node.headingCoefficient ...
                        * (stateOffset(3)-node.nominalHeading) ...
                    - node.supportValue-node.tightening;
            end
            node.incumbentMargin = min(node.marginMatrix ...
                * referenceInput+node.marginOffset);
            node.boxMaximum = localBoxMaximum( ...
                node.marginMatrix, node.marginOffset, ...
                common.reachCentre, common.reachHalfWidth);
            node.capacity = node.boxMaximum-node.incumbentMargin;
            node.authority = localRowAuthority( ...
                node.marginMatrix, common.reachHalfWidth);
            node.window = node.authority < windowBound;
            node.allowance = 0.0;
            if node.window
                node.allowance = (windowTop-nodeIdx+1)*disturbanceBound;
            end
            if certifiedMode && node.terminalSegmentEnforced ...
                    && ~node.terminalInvariant
                certifiedInfeasible = true;
            end
            if certifiedMode
                % Every covered CBF row is hard. Constant stage-0 rows are
                % retained, so an initially unsafe or otherwise infeasible
                % problem reports controller failure rather than buying a
                % violation. The final rest node is the augmented invariant
                % terminal set.
                node.imposed = true;
                node.factSource = "";
            elseif nodeIdx == 1 || ~any(node.marginMatrix(:))
                node.imposed = false;
                node.factSource = "constant";
            elseif elastic && families(familyIdx).name == "collision"
                node.imposed = true;
                node.elastic = true;
            elseif node.boxMaximum >= 0.0
                node.imposed = true;
            elseif node.window && node.boxMaximum >= -node.allowance
                node.imposed = false;
                node.factSource = "unsatisfiable";
            else
                node.imposed = false;
                node.factSource = "infeasible";
                certifiedInfeasible = true;
            end
            if node.imposed
                for signIdx = 1:2
                    rowIdx = rowIdx+1;
                    matrix(rowIdx, layout.planIndex) = ...
                        -node.marginMatrix(signIdx, :);
                    bound(rowIdx) = node.marginOffset(signIdx);
                    rowFamily(rowIdx) = families(familyIdx).name;
                    rowNode(rowIdx) = nodeIdx;
                    rowWindow(rowIdx) = node.window;
                    rowAllowance(rowIdx) = node.allowance;
                    if node.elastic
                        rowSlackNode(rowIdx) = nodeIdx;
                    end
                end
            end
            if certifiedMode && node.terminalSegmentEnforced
                % Keep the terminal Frenet station on the polyline
                % segment whose Cartesian map defines the invariant
                % support row. These are hard terminal-domain rows and
                % never receive a safety slack.
                stationRow = stateMatrix(1, :);
                stationOffset = stateOffset(1);
                stationScale = max(1.0, ...
                    node.terminalStationUpper-node.terminalStationLower);
                rowIdx = rowIdx+1;
                matrix(rowIdx, layout.planIndex) = ...
                    stationRow/stationScale;
                bound(rowIdx) = ...
                    (node.terminalStationUpper-stationOffset)/stationScale;
                rowFamily(rowIdx) = "terminalSegment";
                rowNode(rowIdx) = nodeIdx;
                rowIdx = rowIdx+1;
                matrix(rowIdx, layout.planIndex) = ...
                    -stationRow/stationScale;
                bound(rowIdx) = ...
                    (stationOffset-node.terminalStationLower)/stationScale;
                rowFamily(rowIdx) = "terminalSegment";
                rowNode(rowIdx) = nodeIdx;
            end
            families(familyIdx).nodes(nodeIdx) = node;
        end
    end
    matrix = matrix(1:rowIdx, :);
    bound = bound(1:rowIdx);
    rowFamily = rowFamily(1:rowIdx);
    rowNode = rowNode(1:rowIdx);
    rowWindow = rowWindow(1:rowIdx);
    rowAllowance = rowAllowance(1:rowIdx);
    rowSlackNode = rowSlackNode(1:rowIdx);
end

function windowTop = localWindowTop(families, prediction, common, ...
        windowBound)
% The largest node index whose position rows' authority over the reach
% box is below the window bound. The authority of a face row depends
% on the node's condensed map and the reach half-widths only (up to
% the mix of the two sign rows), so the station row stands for every
% family.
    windowTop = 1;
    nodeCount = size(prediction.egoStateOffset, 2);
    for nodeIdx = 2:nodeCount
        stateMatrix = prediction.egoStateMatrix(:, :, nodeIdx);
        lateralRows = [stateMatrix(2, :); stateMatrix(2, :)];
        stationRows = [stateMatrix(1, :); stateMatrix(1, :)];
        authority = min( ...
            localRowAuthority(lateralRows, common.reachHalfWidth), ...
            localRowAuthority(stationRows, common.reachHalfWidth));
        if authority < windowBound
            windowTop = nodeIdx;
        else
            break;
        end
    end
    if isempty(families)
        return;
    end
end

function authority = localRowAuthority(marginMatrix, halfWidth)
% The largest change of the margin min_sigma(row_sigma u) a step within
% the reach box can make, irrespective of where the plan is: max over
% |du| <= halfWidth of min_sigma(row_sigma du) = min over lambda of the
% halfWidth-weighted 1-norm of the mixed row (the minimax theorem on a
% box). A property of the node and the dynamics - it grows like the
% fourth power of the node index under the Euler map - and what
% defines the near window.
    weights = linspace(0.0, 1.0, 41);
    authority = inf;
    for weight = weights
        mixedRow = weight*marginMatrix(1, :)+(1.0-weight)*marginMatrix(2, :);
        authority = min(authority, abs(mixedRow)*halfWidth(:));
    end
end

function value = localBoxMaximum(marginMatrix, marginOffset, centre, ...
        halfWidth)
% The largest value of min_sigma (row_sigma u + offset_sigma) over the
% box |u - centre| <= halfWidth: by the minimax theorem for a bilinear
% function on a box, min over lambda in [0, 1] of the maximum of the
% mixed row lambda row_+ + (1 - lambda) row_-, which on a box is its
% value at the centre plus its halfWidth-weighted 1-norm. The two rows
% differ only in the sign of the heading term, so the minimum lies
% where the heading coefficient partly cancels the position
% coefficient; a grid over lambda over-estimates the minimum, which
% keeps the value an upper bound on the true maximum.
    weights = linspace(0.0, 1.0, 41);
    value = inf;
    for weight = weights
        mixedRow = weight*marginMatrix(1, :)+(1.0-weight)*marginMatrix(2, :);
        mixedOffset = weight*marginOffset(1)+(1.0-weight)*marginOffset(2);
        value = min(value, mixedRow*centre(:) ...
            + abs(mixedRow)*halfWidth(:)+mixedOffset);
    end
end

% ====================================================================
% CLF rows and the cruise equilibrium
% ====================================================================

function clf = localClfData(prediction, model, nominalInput, layout, ...
        equilibrium)
% Relaxed predictive-CLF rows on every transition of the horizon.
%
% In path coordinates the cruise error at node k is the state itself
% against the cruise equilibrium,
%
%   e_k(u) = [d; ePsi; vx - vRef; vy - vy_eq; r - r_eq]
%          = errorMatrix(:, :, k+1) u + errorOffset(:, k+1),
%
% with (vy_eq, r_eq) the cruise EQUILIBRIUM of the declared model at
% the node's schedule curvature and the published longitudinal bias
% (cruiseEquilibrium). Each transition demands the Riccati-certified
% decrease
%
%   V(e_{k+1}) - V(e_k) <= -fraction * W(e_k) + delta_k,
%   V(e) = e' P e,   W(e) = e' (Q + K' R K) e,
%
% both quadratics replaced by their first-order expansion about the
% nominal error (exact at node 0, whose error is the measured state's).
% The row is written as A z <= b with the relaxation column -1.
    certificate = localClfCertificate(model);
    lyapunovMatrix = certificate.lyapunovMatrix;
    decreaseMatrix = model.cfg.clf.decreaseRateFraction ...
        * certificate.decreaseMatrix;
    horizonSteps = model.horizonSteps;
    nodeCount = horizonSteps+1;
    planCount = layout.planCount;
    errorDimension = numel(certificate.errorStateOrder);
    errorMatrix = zeros(errorDimension, planCount, nodeCount);
    errorOffset = zeros(errorDimension, nodeCount);
    nominalError = zeros(errorDimension, nodeCount);
    nominalControl = nominalInput(:);
    for nodeIdx = 1:nodeCount
        errorMatrix(:, :, nodeIdx) = ...
            prediction.egoStateMatrix(2:6, :, nodeIdx);
        errorOffset(:, nodeIdx) = prediction.egoStateOffset(2:6, nodeIdx) ...
            - [0.0; 0.0; model.referenceSpeed; ...
                equilibrium.lateralVelocity(nodeIdx); ...
                equilibrium.yawRate(nodeIdx)];
        nominalError(:, nodeIdx) = errorMatrix(:, :, nodeIdx) ...
            * nominalControl+errorOffset(:, nodeIdx);
    end

    constraintMatrix = zeros(horizonSteps, layout.decisionCount);
    constraintBound = zeros(horizonSteps, 1);
    for rowIdx = 1:horizonSteps
        currentError = nominalError(:, rowIdx);
        nextError = nominalError(:, rowIdx+1);
        currentGradient = 2.0*(lyapunovMatrix*currentError);
        nextGradient = 2.0*(lyapunovMatrix*nextError);
        decreaseGradient = 2.0*(decreaseMatrix*currentError);
        currentValue = currentError.'*lyapunovMatrix*currentError;
        nextValue = nextError.'*lyapunovMatrix*nextError;
        decreaseValue = currentError.'*decreaseMatrix*currentError;
        constraintMatrix(rowIdx, layout.planIndex) = ...
            nextGradient.'*errorMatrix(:, :, rowIdx+1) ...
            + (decreaseGradient-currentGradient).' ...
                * errorMatrix(:, :, rowIdx);
        constraintMatrix(rowIdx, layout.relaxationIndex(rowIdx)) = -1.0;
        constraintBound(rowIdx) = -(nextValue ...
            + nextGradient.'*(errorOffset(:, rowIdx+1)-nextError) ...
            - currentValue+decreaseValue ...
            + (decreaseGradient-currentGradient).' ...
                * (errorOffset(:, rowIdx)-currentError));
    end

    clf = struct( ...
        "certificate", certificate, ...
        "lyapunovMatrix", lyapunovMatrix, ...
        "decreaseMatrix", decreaseMatrix, ...
        "errorMatrix", errorMatrix, ...
        "errorOffset", errorOffset, ...
        "nominalError", nominalError, ...
        "equilibriumInput", equilibrium.input, ...
        "constraintMatrix", constraintMatrix, ...
        "constraintBound", constraintBound, ...
        "initialValue", nominalError(:, 1).' ...
            * lyapunovMatrix*nominalError(:, 1));
end

function [matrix, bound, rowNode] = localClfTrustRegionRows( ...
        clf, model, layout, scale)
% HARD trust region of the CLF linearization: at every node k >= 1 and
% every error channel i,
%
%   |e_k,i(u) - ebar_k,i| <= trustRegionScale * errorScale_i,
%
% nondimensional. The CLF rows replace V(e_{k+1}) - V(e_k) + f W(e_k)
% by its tangent at the nominal error; the tangent of a convex
% quadratic is below it, so along -grad V the linear model credits a
% decrease that grows without bound while the true V is a bowl. Left
% unbounded, the optimizer buys that credit to cancel a genuine
% increase elsewhere (braking) and the plan overshoots through the
% origin - measured: a 5.35 m lateral excursion at node 48 with zero
% relaxation and an exact residual of 220 (TWO_STAGE_QP.md). Bounding
% the step from the nominal bounds the linearization error to
% Delta'P Delta per node and turns the receding-horizon loop into the
% damped SQP iteration it is described as; the CLF gradient's sign
% flips at the origin, so a bounded step cannot run away over
% samples. A channel whose row is identically zero at a node (the pose
% at node 1 is a fact of the measured state) carries no row. Inf
% removes the family. The ladder's iterates carry their own scale
% (they carry none at all): they are linearization probes, and
% the geometry of stage 1 is exact wherever it is evaluated, so only
% the CLF tangent constrains how far one of them may step.
    horizonSteps = layout.horizonSteps;
    errorDimension = size(clf.errorMatrix, 1);
    matrix = zeros(2*errorDimension*horizonSteps, layout.decisionCount);
    bound = zeros(size(matrix, 1), 1);
    rowNode = zeros(size(matrix, 1), 1);
    if ~isfinite(scale)
        matrix = matrix(1:0, :);
        bound = bound(1:0);
        rowNode = rowNode(1:0);
        return;
    end
    cfg = model.cfg;
    radius = scale*[ ...
        cfg.clf.lateralPositionErrorScale; ...
        cfg.clf.headingErrorScale; ...
        cfg.clf.speedErrorScale; ...
        cfg.clf.lateralVelocityErrorScale; ...
        cfg.clf.yawRateErrorScale];
    rowIdx = 0;
    for nodeIdx = 2:horizonSteps+1
        for channelIdx = 1:errorDimension
            errorRow = clf.errorMatrix(channelIdx, :, nodeIdx);
            if ~any(errorRow)
                continue;
            end
            centre = clf.nominalError(channelIdx, nodeIdx) ...
                - clf.errorOffset(channelIdx, nodeIdx);
            rowIdx = rowIdx+1;
            matrix(rowIdx, layout.planIndex) = errorRow/radius(channelIdx);
            bound(rowIdx) = (centre+radius(channelIdx))/radius(channelIdx);
            rowNode(rowIdx) = nodeIdx;
            rowIdx = rowIdx+1;
            matrix(rowIdx, layout.planIndex) = -errorRow/radius(channelIdx);
            bound(rowIdx) = (radius(channelIdx)-centre)/radius(channelIdx);
            rowNode(rowIdx) = nodeIdx;
        end
    end
    matrix = matrix(1:rowIdx, :);
    bound = bound(1:rowIdx);
    rowNode = rowNode(1:rowIdx);
end

function certificate = localClfCertificate(model)
% Riccati CLF certificate of the path-frame cruise error.
%
% The error state [d; ePsi; vx - vRef; vy; r] is the [d; ePsi; vx; vy;
% r] block of the Frenet stage map at the straight reference cruise
% (kappa = 0) - the station row does not feed back into it. The
% discrete Riccati solution P and gain K certify, for the
% unconstrained linearized error dynamics,
%
%   V(e+) - V(e) = -e' (Q + K' R K) e,
%
% the direction-wise decrease the rows demand (scaled by the
% configured fraction). Input saturation and nonlinearity make the
% certificate local; the relaxation absorbs the states where the
% demanded contraction is not achievable.
    persistent memoKey memoCertificate
    cfg = model.cfg;
    [minimumAcceleration, maximumAcceleration] = ...
        longitudinalAccelerationBounds(cfg);
    accelerationScale = max( ...
        abs(minimumAcceleration), abs(maximumAcceleration));
    key = struct( ...
        "referenceSpeed", max(model.referenceSpeed, ...
            cfg.clf.certificateSpeedFloor), ...
        "sampleTime", model.sampleTime, ...
        "errorScale", [ ...
            cfg.clf.lateralPositionErrorScale; ...
            cfg.clf.headingErrorScale; ...
            cfg.clf.speedErrorScale; ...
            cfg.clf.lateralVelocityErrorScale; ...
            cfg.clf.yawRateErrorScale], ...
        "inputWeight", [ ...
            cfg.clf.frontWheelSteeringAngleWeight; ...
            cfg.clf.longitudinalAccelerationWeight], ...
        "inputScale", [ ...
            cfg.model.frontWheelSteeringAngleMaximum; ...
            accelerationScale], ...
        "vehicle", cfg.vehicle, ...
        "corneringStiffness", cfg.tire.corneringStiffness);
    if ~isempty(memoKey) && isequaln(key, memoKey)
        certificate = memoCertificate;
        return;
    end
    [stageMatrixA, stageMatrixB] = ltvBicycleStageMatrices( ...
        0.0, key.referenceSpeed, key.sampleTime, cfg);
    errorIndex = 2:6;
    errorStateMatrix = stageMatrixA(errorIndex, errorIndex);
    errorInputMatrix = stageMatrixB(errorIndex, :);
    stateWeight = diag(1.0./key.errorScale.^2);
    inputWeight = diag(key.inputWeight./key.inputScale.^2);
    try
        [feedbackGain, lyapunovMatrix] = dlqr( ...
            errorStateMatrix, errorInputMatrix, stateWeight, inputWeight);
    catch riccatiException
        error("collisionAvoidanceController:invalidFormulation", ...
            "The CLF Riccati synthesis at the reference cruise " ...
            + "failed: %s", riccatiException.message);
    end
    decreaseMatrix = stateWeight+feedbackGain.'*inputWeight*feedbackGain;
    decreaseMatrix = 0.5*(decreaseMatrix+decreaseMatrix.');
    decreaseEigenvalue = min(real(eig(decreaseMatrix, lyapunovMatrix)));
    certificate = struct( ...
        "lyapunovMatrix", lyapunovMatrix, ...
        "feedbackGain", feedbackGain, ...
        "decreaseMatrix", decreaseMatrix, ...
        "certifiedDecreaseRate", ...
            min(max(decreaseEigenvalue, 1.0e-6), 1.0), ...
        "errorStateOrder", ["lateralError"; "headingError"; ...
            "speedError"; "lateralVelocity"; "yawRateError"]);
    memoKey = key;
    memoCertificate = certificate;
end

function equilibrium = cruiseEquilibrium( ...
        referenceSpeed, curvature, accelerationBias, cfg)
% cruiseEquilibrium Steady cruise target of the declared model.
%
% Given the demanded cruise speed, the local path curvature, and the
% estimator's longitudinal model bias, returns the state and input
% that make the DECLARED model stationary in the path frame. Steady
% cornering of the linear-cornering bicycle at speed v and yaw rate
% r = curvature*v distributes the centripetal demand over the axles by
% the moment balance, Fyf = lr/L m v r, Fyr = lf/L m v r; the rear slip
% fixes the sideslip and the front slip the steering angle (the
% classic understeer form), and the longitudinal equilibrium cancels
% the bias and the centripetal cross term of vxdot = a + vy r + b.
% Aiming the CLF at this fixed point instead of the kinematic guess is
% what makes the closed loop stationary at zero error with no
% integrator.
    if referenceSpeed <= 0.0
        error("collisionAvoidanceController:invalidInput", ...
            "cruiseEquilibrium requires a positive reference speed.");
    end
    mass = cfg.vehicle.m;
    lf = cfg.vehicle.lf;
    lr = cfg.vehicle.lr;
    wheelbase = lf+lr;
    corneringStiffness = double(cfg.tire.corneringStiffness(:));
    if isscalar(corneringStiffness)
        corneringStiffness = repmat(corneringStiffness, 2, 1);
    end
    yawRate = curvature*referenceSpeed;
    lateralForceTotal = mass*referenceSpeed*yawRate;
    frontLateralForce = lr/wheelbase*lateralForceTotal;
    rearLateralForce = lf/wheelbase*lateralForceTotal;
    rearSlipAngle = -rearLateralForce/corneringStiffness(2);
    lateralVelocity = referenceSpeed*rearSlipAngle+lr*yawRate;
    frontSlipAngle = frontLateralForce/corneringStiffness(1);
    steeringAngle = frontSlipAngle ...
        + (lateralVelocity+lf*yawRate)/referenceSpeed;
    longitudinalAcceleration = -accelerationBias-lateralVelocity*yawRate;
    equilibrium = struct( ...
        "referenceSpeed", referenceSpeed, ...
        "curvature", curvature, ...
        "lateralVelocity", lateralVelocity, ...
        "yawRate", yawRate, ...
        "steeringAngle", steeringAngle, ...
        "longitudinalAcceleration", longitudinalAcceleration);
end

% ====================================================================
% Objective, bounds, and the hard model rows
% ====================================================================

function [hessian, linear, constant] = localObjective( ...
        model, equilibriumInput, layout)
% Minimum intervention: the deviation of the whole input plan from the
% cruise equilibrium input, per stage and channel at the Riccati input
% weights over the amplitude scales, plus the price of the CLF
% relaxations. quadprog's convention 0.5 z'Hz + f'z + constant.
    cfg = model.cfg;
    [inputWeight, inputScale] = localInputWeightAndScale(cfg);
    hessian = zeros(layout.decisionCount);
    linear = zeros(layout.decisionCount, 1);
    constant = 0.0;
    for stageIdx = 1:layout.horizonSteps
        for inputIdx = 1:layout.inputDimension
            columnIdx = (stageIdx-1)*layout.inputDimension+inputIdx;
            weight = model.sampleTime*inputWeight(inputIdx) ...
                / inputScale(inputIdx)^2;
            reference = equilibriumInput(inputIdx, stageIdx);
            hessian(columnIdx, columnIdx) = 2.0*weight;
            linear(columnIdx) = -2.0*weight*reference;
            constant = constant+weight*reference^2;
        end
    end
    % The tail accelerations are proof-side - the witness that rest is
    % reachable - and carry no price: a curvature six decades below
    % the head's keeps the Hessian positive definite and, among the
    % admissible tails, selects the gentlest.
    tailCurvature = 1.0e-6*model.sampleTime*inputWeight(2)/inputScale(2)^2;
    for tailIdx = layout.tailIndex
        hessian(tailIdx, tailIdx) = 2.0*tailCurvature;
    end
    % The CLF relaxation has only its configured linear price. It carries
    % no quadratic curvature.
    for relaxationIdx = layout.relaxationIndex
        linear(relaxationIdx) = cfg.clf.relaxationWeight;
    end
end

function [inputWeight, inputScale] = localInputWeightAndScale(cfg)
    [minimumAcceleration, maximumAcceleration] = ...
        longitudinalAccelerationBounds(cfg);
    accelerationScale = max( ...
        abs(minimumAcceleration), abs(maximumAcceleration));
    inputScale = [ ...
        cfg.model.frontWheelSteeringAngleMaximum; accelerationScale];
    inputWeight = [ ...
        cfg.clf.frontWheelSteeringAngleWeight; ...
        cfg.clf.longitudinalAccelerationWeight];
end

function [lowerBound, upperBound] = localInputBox(model)
% Physical input box of [deltaF; a] over the horizon.
    cfg = model.cfg;
    [accelerationMinimum, accelerationMaximum] = ...
        longitudinalAccelerationBounds(cfg);
    steeringLimit = cfg.model.frontWheelSteeringAngleMaximum;
    lowerBound = repmat([-steeringLimit; accelerationMinimum], ...
        model.horizonSteps, 1);
    upperBound = repmat([steeringLimit; accelerationMaximum], ...
        model.horizonSteps, 1);
end

function interval = localPlannedSpeedInterval(prediction, model)
% The interval the hard speed rows put the PLANNED speed vx_k in at
% every node, and the radius that separates it from the realized one:
%
%   plannedLower_k <= vx_k(z) <= plannedUpper_k,
%   plannedUpper_k = speedMaximum - radius_k,
%   plannedLower_k = min(floor, releaseSpeed_k - radius_k) + radius_k,
%
% so the realized speed stays inside [floor_k, speedMaximum]. The floor
% is the planned minimum, lowered at every node to the speed the
% LEAST-BRAKING ADMISSIBLE release reaches there - maximum acceleration
% with the schedule steering, through the model. Below that speed the
% row would be a fact no input can change, and a hard row there is
% either true or infeasible, never a constraint.
    cfg = model.cfg;
    horizonSteps = model.horizonSteps;
    speedFloor = model.plannedSpeedFloor;
    speedMaximum = cfg.model.speedMaximum;
    [~, accelerationMaximum] = longitudinalAccelerationBounds(cfg);
    release = repmat(accelerationMaximum, 1, horizonSteps);
    releaseSteering = prediction.referenceInput(1, :);
    releaseInput = [releaseSteering; release];
    releasePlan = [releaseInput(:); zeros(prediction.tailSteps, 1)];
    nodeCount = horizonSteps+1;
    interval = struct( ...
        "plannedLower", zeros(1, nodeCount), ...
        "plannedUpper", zeros(1, nodeCount), ...
        "radius", zeros(1, nodeCount), ...
        "width", max(speedMaximum-speedFloor, 1.0));
    for nodeIdx = 1:nodeCount
        releaseSpeed = ...
            prediction.egoStateMatrix(4, :, nodeIdx)*releasePlan ...
            + prediction.egoStateOffset(4, nodeIdx);
        radius = prediction.egoStateErrorBound(4, nodeIdx);
        interval.radius(nodeIdx) = radius;
        interval.plannedLower(nodeIdx) = ...
            min(speedFloor, releaseSpeed-radius)+radius;
        interval.plannedUpper(nodeIdx) = speedMaximum-radius;
    end
end

function [matrix, bound] = localSpeedDomainRows(prediction, model, ...
        layout, interval)
% Hard longitudinal-speed rows from the first step on, the interval of
% localPlannedSpeedInterval nondimensionalized by its width.
    horizonSteps = model.horizonSteps;
    intervalWidth = interval.width;
    matrix = zeros(2*horizonSteps, layout.decisionCount);
    bound = zeros(2*horizonSteps, 1);
    for nodeIdx = 2:horizonSteps+1
        speedRow = prediction.egoStateMatrix(4, :, nodeIdx);
        speedOffset = prediction.egoStateOffset(4, nodeIdx);
        rowIdx = 2*(nodeIdx-2)+1;
        matrix(rowIdx, layout.planIndex) = speedRow/intervalWidth;
        bound(rowIdx) = ...
            (interval.plannedUpper(nodeIdx)-speedOffset)/intervalWidth;
        matrix(rowIdx+1, layout.planIndex) = -speedRow/intervalWidth;
        bound(rowIdx+1) = ...
            (speedOffset-interval.plannedLower(nodeIdx))/intervalWidth;
    end
end

function [matrix, bound] = localHeadingDomainRows( ...
        prediction, model, layout)
% Hard validity rows of the heading linearization from the third node
% on: |ePsi| <= headingDomainRadius net of the measured yaw radius,
% nondimensional. The first two nodes are facts of the measured state.
    cfg = model.cfg;
    domainRadius = cfg.model.headingDomainRadius;
    horizonSteps = model.horizonSteps;
    matrix = zeros(2*(horizonSteps-1), layout.decisionCount);
    bound = zeros(2*(horizonSteps-1), 1);
    for nodeIdx = 3:horizonSteps+1
        headingRow = prediction.egoStateMatrix(3, :, nodeIdx);
        headingOffset = prediction.egoStateOffset(3, nodeIdx);
        radius = min(prediction.egoStateErrorBound(3, nodeIdx), ...
            0.5*domainRadius);
        rowIdx = 2*(nodeIdx-3)+1;
        matrix(rowIdx, layout.planIndex) = headingRow/domainRadius;
        bound(rowIdx) = (domainRadius-radius-headingOffset)/domainRadius;
        matrix(rowIdx+1, layout.planIndex) = -headingRow/domainRadius;
        bound(rowIdx+1) = (domainRadius-radius+headingOffset)/domainRadius;
    end
end
