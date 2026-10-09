function results = verifyObserverRobustConnection(outputPath)
%verifyObserverRobustConnection Check the observer-to-PCBF derivation.
% Finite algebraic examples, exact extremizers and counterexamples support
% OBSERVER_ROBUST_PCBF_THEORY.tex. They are not a vehicle safety certificate,
% a validated interval proof, or a closed-loop scenario campaign.

    arguments
        outputPath (1, 1) string = ""
    end

    previousPath = path;
    previousRandom = rng;
    cleanup = onCleanup(@() localRestore(previousPath, previousRandom));
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root, "estimator"));
    rng(20261003, "twister");
    methods = {@localComparison, @localEllipsoid, @localCovariantMotion, @localRelativeYawBound, ...
        @localRelativeGeometry, @localRobustVertexRows, @localWholeHold, ...
        @localIntersection, @localPredictionVersusEstimation, ...
        @localOutputFeedbackTube, @localRemainderDomain, ...
        @localClfSupply, @localClfCone, ...
        @localVanishingTie, @localCausalAction};
    checks = repmat(struct("name", "", "passed", false, "metrics", struct, ...
        "failure", ""), numel(methods), 1);
    for index = 1:numel(methods)
        checks(index).name = string(func2str(methods{index}));
        try
            checks(index).metrics = methods{index}();
            checks(index).passed = true;
        catch exception
            checks(index).failure = string(exception.identifier)+": "+exception.message;
        end
    end
    results = struct("scope", "finite mathematical checks; not a vehicle certificate", ...
        "randomSeed", 20261003, "matlabRelease", string(version("-release")), ...
        "checks", checks, "passed", sum([checks.passed]), "total", numel(checks));
    if strlength(outputPath) > 0
        folder = fileparts(outputPath);
        if strlength(folder) > 0 && ~isfolder(folder)
            mkdir(folder);
        end
        file = fopen(outputPath, "w");
        assert(file >= 0, "verifyObserverRobustConnection:output", "Cannot write results.");
        closeFile = onCleanup(@() fclose(file));
        fprintf(file, "%s\n", jsonencode(results, PrettyPrint=true));
        clear closeFile;
    end
    fprintf("Observer robust connection: %d / %d finite checks passed.\n", ...
        results.passed, results.total);
    if ~all([checks.passed])
        error("verifyObserverRobustConnection:failed", "%s", ...
            strjoin([checks(~[checks.passed]).failure], newline));
    end
end

function localRestore(previousPath, previousRandom)
    path(previousPath);
    rng(previousRandom);
end

function metrics = localComparison()
    largestResidual = 0;
    smallestGap = Inf;
    for rates = [1.7, 1.7; 0.8, 1.7]
        stage = struct("velocityDecayRate", rates(1), ...
            "targetDecayRate", rates(2), "targetVelocityCoupling", 0.6, ...
            "input", [0.03; 0.05]);
        certificate = nrmmObserverCertificate("core",stage);
        matrix = certificate.matrix;
        initial = [0.4; 0.7];
        for time = linspace(0, 4, 81)
            first = exp(-rates(1)*time);
            second = exp(-rates(2)*time);
            if rates(1) == rates(2)
                coupling = stage.targetVelocityCoupling*time*first;
            else
                coupling = stage.targetVelocityCoupling*(first-second)/(rates(2)-rates(1));
            end
            transition = [first, 0; coupling, second];
            largestResidual = max(largestResidual, norm(transition-expm(matrix*time), Inf));
            envelope = transition*initial+matrix\((transition-eye(2))*stage.input);
            actual = transition*(0.7*initial)+matrix\((transition-eye(2))*(0.6*stage.input));
            smallestGap = min(smallestGap, min(envelope-actual));
        end
        assert(all(certificate.weights > 0));
        assert(norm(certificate.weights.'*matrix+ones(1, 2), Inf) < 1e-12);
    end
    assert(largestResidual < 1e-12 && smallestGap >= -1e-12);
    metrics = struct("exponentialResidual", largestResidual, "minimumContainmentGap", smallestGap);
end

function metrics = localEllipsoid()
    observerMetric = [2, 0.2, -0.1; 0.2, 1.4, 0.15; -0.1, 0.15, 1.1];
    frequency = 4;
    scale = diag([1, 1/frequency, 1/frequency^2]);
    metric = kron(scale*observerMetric*scale, eye(2));
    inverseMetric = metric\eye(6);
    inverseObserver = observerMetric\eye(3);
    radius = 0.12;
    residual = 0;
    for component = 1:3
        for angle = linspace(-pi, pi, 31)
            direction = zeros(6, 1);
            direction(2*component-1:2*component) = [cos(angle); sin(angle)];
            support = radius*sqrt(direction.'*inverseMetric*direction);
            extremizer = radius*inverseMetric*direction/sqrt(direction.'*inverseMetric*direction);
            expected = radius*frequency^(component-1)*sqrt(inverseObserver(component, component));
            residual = max([residual, abs(support-expected), ...
                abs(extremizer.'*metric*extremizer-radius^2), abs(direction.'*extremizer-support)]);
        end
    end
    assert(residual < 1e-12);
    metrics = struct("exactExtremizerResidual", residual);
end

function metrics = localCovariantMotion()
    step = 1e-5;
    time = 0.7;
    yawRate = 0.23;
    [velocity, acceleration] = localTargetMotion(time, yawRate);
    [beforeVelocity, beforeAcceleration] = localTargetMotion(time-step, yawRate);
    [afterVelocity, afterAcceleration] = localTargetMotion(time+step, yawRate);
    rotationGenerator = [0, -1; 1, 0];
    speed = norm(velocity);
    tangential = velocity.'*acceleration/speed;
    targetRate = (rotationGenerator*velocity).'*acceleration/speed^2;
    jerk = -targetRate^2*velocity+3*tangential*targetRate/speed*rotationGenerator*velocity;
    residual = max(norm((afterVelocity-beforeVelocity)/(2*step)+yawRate*rotationGenerator*velocity-acceleration), ...
        norm((afterAcceleration-beforeAcceleration)/(2*step)+yawRate*rotationGenerator*acceleration-jerk));
    beta = asin(1.5*(rotationGenerator*velocity).'*acceleration/speed^3);
    assert(residual < 1e-8 && abs(beta-asin(1.5*0.02)) < 1e-12);
    metrics = struct("transportDerivativeResidual", residual, "reconstructedSideslipRad", beta);
end

function [velocity, acceleration] = localTargetMotion(time, egoRate)
    speed = 8+0.4*time;
    course = 0.5+0.02*(8*time+0.2*time^2);
    tangent = [cos(course); sin(course)];
    generator = [0, -1; 1, 0];
    rotation = localRotation(-egoRate*time);
    velocity = rotation*(speed*tangent);
    acceleration = rotation*(0.4*tangent+0.02*speed^2*generator*tangent);
end

function metrics = localRelativeYawBound()
    velocity = [8; 1];
    acceleration = [0.5; 1.2];
    velocityRadius = 0.1;
    accelerationRadius = 0.1;
    axleDistance = 1.5;
    maximumSideslip = 0.3;
    minimumSpeed = norm(velocity)-velocityRadius;
    maximumAcceleration = norm(acceleration)+accelerationRadius;
    sideslipRadius = axleDistance/cos(maximumSideslip) ...
        *(4*maximumAcceleration/minimumSpeed^3*velocityRadius ...
        +accelerationRadius/minimumSpeed^2);
    headingRadius = asin(velocityRadius/norm(velocity))+sideslipRadius;
    generator = [0, -1; 1, 0];
    beta = @(q, s) asin(min(max(axleDistance*(generator*q).'*s/norm(q)^3, ...
        -sin(maximumSideslip)), sin(maximumSideslip)));
    nominal = atan2(velocity(2), velocity(1))-beta(velocity, acceleration);
    largestError = 0;
    for index = 1:10000
        direction = randn(2, 1);
        q = velocity+velocityRadius*rand*direction/norm(direction);
        direction = randn(2, 1);
        s = acceleration+accelerationRadius*rand*direction/norm(direction);
        actual = atan2(q(2), q(1))-beta(q, s);
        error = abs(atan2(sin(actual-nominal), cos(actual-nominal)));
        largestError = max(largestError, error);
    end
    assert(largestError <= headingRadius);
    metrics = struct("samples", 10000, "relativeYawBoundRad", headingRadius, ...
        "largestSampledRelativeYawErrorRad", largestError);
end

function metrics = localRelativeGeometry()
    ego = [2, 2, -2, -2; 1, -1, -1, 1]+[0.3; 0];
    target = [2.2, 2.2, -2.2, -2.2; 0.9, -0.9, -0.9, 0.9];
    relative = [8; 1.4];
    relativeYaw = -0.2;
    normal = [0.8; 0.6];
    expected = min(normal.'*(relative+localRotation(relativeYaw)*target))-max(normal.'*ego);
    residual = 0;
    for index = 1:200
        yaw = 2*pi*rand-pi;
        translation = 100*randn(2, 1);
        rotation = localRotation(yaw);
        egoWorld = translation+rotation*ego;
        targetWorld = translation+rotation*(relative+localRotation(relativeYaw)*target);
        worldNormal = rotation*normal;
        gap = min(worldNormal.'*targetWorld)-max(worldNormal.'*egoWorld);
        residual = max(residual, abs(gap-expected));
    end
    assert(residual < 1e-11);
    metrics = struct("commonPoseCancellationResidualMeters", residual);
end

function metrics = localRobustVertexRows()
    normal = [0.8; 0.6];
    anchorPosition = [6; 1];
    anchorYaw = 0.2;
    vertices = [2.2, 2.2, -2.2, -2.2; 0.9, -0.9, -0.9, 0.9];
    egoSupport = 2*abs(normal(1))+abs(normal(2));
    uncertaintyRadius = [0.12; 0.07; 0.035];
    yawTrust = 0.10;
    generator = [0, -1; 1, 0];
    smallestGap = Inf;
    trials = 10000;
    for index = 1:trials
        correction = [0.4; 0.4; yawTrust].*(2*rand(3, 1)-1);
        error = uncertaintyRadius.*(2*rand(3, 1)-1);
        for vertex = vertices
            jacobian = [normal; normal.'*localRotation(anchorYaw)*generator*vertex];
            nominal = normal.'*(anchorPosition+localRotation(anchorYaw)*vertex)-egoSupport;
            support = abs(jacobian).'*uncertaintyRadius;
            remainder = 0.5*norm(vertex)*(yawTrust+uncertaintyRadius(3))^2;
            lower = nominal+jacobian.'*correction-support-remainder;
            actual = normal.'*(anchorPosition+correction(1:2)+error(1:2) ...
                +localRotation(anchorYaw+correction(3)+error(3))*vertex)-egoSupport;
            smallestGap = min(smallestGap, actual-lower);
        end
    end
    assert(smallestGap >= -1e-12);
    metrics = struct("samples", trials, "minimumEnclosureReserveMeters", smallestGap);
end

function metrics = localWholeHold()
    holdTime = 0.05;
    margin = 0.20;
    lipschitz = 4;
    time = linspace(0, holdTime, 1001);
    nodes = [0; holdTime/2; holdTime];
    nearestNode = min(abs(time-nodes), [], 1);
    uncertified = margin-lipschitz*nearestNode;
    reserve = lipschitz*holdTime/4;
    certified = margin+reserve-lipschitz*nearestNode;
    assert(min(uncertified) < margin-0.049 && min(certified) >= margin-1e-12);
    metrics = struct("nodeOnlyMinimumMeters", min(uncertified), ...
        "holdReserveMeters", reserve, "certifiedMinimumMeters", min(certified));
end

function metrics = localIntersection()
    corner = [1; sqrt(3)/2];
    lensTop = [0.5; sqrt(3)/2];
    assert(norm(lensTop) <= 1+1e-12 && norm(lensTop-[1; 0]) <= 1+1e-12);
    assert(norm(corner) > 1 && norm(corner-[1; 0]) < 1);
    metrics = struct("boundingBoxCornerOutsidePriorBy", norm(corner)-1, ...
        "intersectionRetainsTruth", true);
end

function metrics = localPredictionVersusEstimation()
    time = 2;
    initialPositionRadius = 0.1;
    speedRadius = 0.2;
    reachableRadius = initialPositionRadius+time*speedRadius;
    incorrectRadius = initialPositionRadius*exp(-2*time);
    assert(reachableRadius > 200*incorrectRadius);
    metrics = struct("reachablePositionRadiusMeters", reachableRadius, ...
        "invalidObserverShrinkRadiusMeters", incorrectRadius);
end

function metrics = localOutputFeedbackTube()
    stateMatrix = [1, 0.1; 0, 1];
    inputMatrix = [0.005; 0.1];
    gain = [-2, -1.1];
    stateRadius = [0.1; 0.2];
    observerRadius = [0.03; 0.04];
    noiseRadius = [0.001; 0.002];
    successorRadius = abs(stateMatrix+inputMatrix*gain)*stateRadius ...
        +abs(inputMatrix*gain)*observerRadius+noiseRadius;
    points = dec2bin(0:63)-'0';
    residual = 0;
    smallestReserve = Inf;
    wrongSignDifference = 0;
    for signs = (2*points-1).'
        deviation = stateRadius.*signs(1:2);
        observerError = observerRadius.*signs(3:4);
        noise = noiseRadius.*signs(5:6);
        issuedInput = gain*(deviation-observerError);
        actual = stateMatrix*deviation+inputMatrix*issuedInput+noise;
        enclosed = (stateMatrix+inputMatrix*gain)*deviation-inputMatrix*gain*observerError+noise;
        wrong = (stateMatrix+inputMatrix*gain)*deviation+inputMatrix*gain*observerError+noise;
        residual = max(residual, norm(actual-enclosed, Inf));
        wrongSignDifference = max(wrongSignDifference, norm(actual-wrong, Inf));
        smallestReserve = min(smallestReserve, min(successorRadius-abs(actual)));
    end
    assert(residual < 1e-12 && smallestReserve >= -1e-12 && wrongSignDifference > 0.01);
    metrics = struct("exactDynamicsResidual", residual, ...
        "minimumBoxReserve", smallestReserve, "wrongSignDiscrepancy", wrongSignDifference);
end

function metrics = localRemainderDomain()
    anchorState = 0.3;
    anchorInput = 0.4;
    stateTrust = 0.4;
    inputTrust = 0.3;
    stateError = 0.05;
    inputError = 0.04;
    stateIncrement = (stateTrust+stateError)*(2*rand(1, 10000)-1);
    inputIncrement = (inputTrust+inputError)*(2*rand(1, 10000)-1);
    exact = sin(anchorState+stateIncrement)+0.1*(anchorInput+inputIncrement).^2;
    affine = sin(anchorState)+0.1*anchorInput^2+cos(anchorState)*stateIncrement ...
        +0.2*anchorInput*inputIncrement;
    correct = 0.5*(stateTrust+stateError)^2+0.1*(inputTrust+inputError)^2;
    incorrect = 0.5*stateError^2+0.1*inputError^2;
    maximumError = max(abs(exact-affine));
    assert(maximumError <= correct && maximumError > 10*incorrect);
    metrics = struct("maximumSampledRemainder", maximumError, ...
        "wholeDomainBound", correct, "uncertaintyOnlyBound", incorrect);
end

function metrics = localClfSupply()
    contraction = 0.7;
    rate = 0.8;
    disturbance = 0.12;
    supply = rate/(rate-contraction^2)*disturbance^2;
    extremizer = contraction*disturbance/(rate-contraction^2);
    states = [linspace(0, 4, 10001), extremizer];
    reserve = rate*states.^2+supply-(contraction*states+disturbance).^2;
    assert(min(reserve) >= -1e-12 && abs(reserve(end)) < 1e-12);
    decay = 0.2;
    value = 2;
    for index = 1:300
        value = (1-decay)*value+supply;
    end
    assert(abs(value-supply/decay) < 1e-12);
    metrics = struct("minimumYoungReserve", min(reserve), "issSupply", supply, ...
        "persistentSupplyLimit", value);
end

function metrics = localClfCone()
    metric = diag([2, 1]);
    stageMetric = diag([1, 0.5]);
    decrease = 0.2;
    reduced = metric-decrease*stageMetric;
    currentCenter = [1; 0.3];
    currentRadius = [0.03; 0.02];
    nextCenter = [0.5; 0.1];
    nextRadius = [0.02; 0.01];
    currentNormError = norm(sqrt(reduced)*currentRadius);
    lower = max(0, norm(sqrt(reduced)*currentCenter)-currentNormError)^2;
    upper = (norm(sqrt(metric)*nextCenter)+norm(sqrt(metric)*nextRadius))^2;
    assert(upper <= lower);
    signs = (2*(dec2bin(0:15)-'0')-1).';
    largestResidual = -Inf;
    for sign = signs
        current = currentCenter+currentRadius.*sign(1:2);
        successor = nextCenter+nextRadius.*sign(3:4);
        residual = successor.'*metric*successor-current.'*metric*current ...
            +decrease*current.'*stageMetric*current;
        largestResidual = max(largestResidual, residual);
    end
    assert(largestResidual <= 0);
    metrics = struct("socLowerCurrentValue", lower, "socUpperSuccessorValue", upper, ...
        "largestCornerClfResidual", largestResidual);
end

function metrics = localVanishingTie()
    epsilon = 0.01;
    constantWeightInput = epsilon/(1+epsilon);
    constantWeightSlack = constantWeightInput^2;
    assert(constantWeightSlack > 0);
    decrease = 0.2;
    anchor = 0.6;
    trust = 2;
    largestSlackExcess = -Inf;
    largestDissipationResidual = -Inf;
    for state = [0, logspace(-5, 0, 51)]
        weight = epsilon*state^2/trust^2;
        lower = -state-sqrt(1-decrease)*abs(state);
        upper = -state+sqrt(1-decrease)*abs(state);
        zeroSlackChoice = min(max(anchor, lower), upper);
        positiveSlackStationary = (weight*anchor-state)/(1+weight);
        candidates = [zeroSlackChoice, positiveSlackStationary, lower, upper, anchor-trust, anchor+trust];
        candidates = candidates(candidates >= anchor-trust & candidates <= anchor+trust);
        slacks = max(0, (state+candidates).^2-(1-decrease)*state^2);
        objectives = slacks+weight*(candidates-anchor).^2;
        [~, chosen] = min(objectives);
        slack = slacks(chosen);
        input = candidates(chosen);
        largestSlackExcess = max(largestSlackExcess, slack-epsilon*state^2);
        largestDissipationResidual = max(largestDissipationResidual, ...
            (state+input)^2-state^2+(decrease-epsilon)*state^2);
    end
    assert(largestSlackExcess <= 1e-12 && largestDissipationResidual <= 1e-12);
    metrics = struct("fixedTieSlackAtNominal", constantWeightSlack, ...
        "vanishingTieMaximumSlackExcess", largestSlackExcess, ...
        "vanishingTieMaximumDissipationResidual", largestDissipationResidual);
end

function metrics = localCausalAction()
    possibleStates = [-1, 1];
    desiredRadius = 0.1;
    trueStateFeedback = -possibleStates;
    assert(all(abs(possibleStates+trueStateFeedback) <= desiredRadius));
    firstInputInterval = [-desiredRadius, desiredRadius]-possibleStates(1);
    secondInputInterval = [-desiredRadius, desiredRadius]-possibleStates(2);
    commonLower = max(firstInputInterval(1), secondInputInterval(1));
    commonUpper = min(firstInputInterval(2), secondInputInterval(2));
    assert(commonLower > commonUpper);
    metrics = struct("commonActionSetEmpty", true, ...
        "individualTrueStateActionsExist", true, "intervalInfeasibilityGap", commonLower-commonUpper);
end

function rotation = localRotation(angle)
    rotation = [cos(angle), -sin(angle); sin(angle), cos(angle)];
end
