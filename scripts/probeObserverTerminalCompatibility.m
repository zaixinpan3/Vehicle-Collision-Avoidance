function results = probeObserverTerminalCompatibility(campaignDirectory, outputPath)
%probeObserverTerminalCompatibility Stress the saved nominal terminal cores.
% This offline experiment applies the existing nominal feedback to an
% erroneous estimate while the true vehicle is exactly at its cruise trim.
% Errors lie in the box admitted by the saved controller formulation. The
% experiment does not assert that every box point is an attainable observer
% state, and does not execute controls in a scenario or certify a new core.

    arguments
        campaignDirectory (1, 1) string
        outputPath (1, 1) string = ""
    end

    previousPath = path;
    cleanup = onCleanup(@() path(previousPath));
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root, "controller"), fullfile(root, "config"));
    cases = struct([]);
    for speed = [8, 15]
        source = fullfile(campaignDirectory, "speed"+speed+"-headOn-formulation.mat");
        data = load(source, "captured");
        model = data.captured.primary{1}.model;
        cfg = model.cfg;
        seed = model.terminal;
        [next, stateJacobian, inputJacobian] = nonlinearBicycleModel.sample( ...
            seed.base, seed.reference.input, cfg);
        assert(abs(next(3)) < 1e-12, ...
            "probeObserverTerminalCompatibility:trim", "This probe uses a straight cruise trim.");
        augmentedState = [stateJacobian, zeros(6, 2); zeros(2, 8)];
        augmentedInput = [inputJacobian; eye(2)];
        closedLoop = augmentedState+augmentedInput*seed.gain;
        quotientInjection = seed.quotientFactor*augmentedInput(4:8, :)*seed.gain(:, 1:6);
        radii = sum(abs(model.uncertainty.egoGenerator), 2);
        trials = struct([]);
        for fraction = [0.001, 0.01, 1]
            for signValue = [-1, 1]
                observerError = zeros(6, 1); % true state minus estimate
                observerError(5) = signValue*fraction*radii(5);
                input = seed.reference.input-seed.gain(:, 1:6)*observerError;
                successor = nonlinearBicycleModel.sample(seed.base, input, cfg);
                intrinsic = [successor(4:6)-seed.base(4:6); input-seed.reference.input];
                normValue = norm(seed.quotientFactor*intrinsic);
                linearNorm = norm(quotientInjection*observerError);
                trial = struct("lateralVelocityErrorMetersPerSecond", observerError(5), ...
                    "fractionOfPublishedRadius", fraction, "input", input, ...
                    "nonlinearQuotientNorm", normValue, "linearQuotientNorm", linearNorm, ...
                    "ratioToNominalCoreRadius", normValue/seed.radius, ...
                    "outsideEveryRigidlyPlacedNominalCore", normValue > seed.radius);
                trials = [trials; trial]; %#ok<AGROW>
            end
        end
        % At zero estimation error, the same numerical trim stays in the core.
        nominalIntrinsic = [next(4:6)-seed.base(4:6); zeros(2, 1)];
        nominalNorm = norm(seed.quotientFactor*nominalIntrinsic);
        assert(nominalNorm <= seed.radius);
        row = struct("speedMetersPerSecond", speed, "source", source, ...
            "nominalCoreRadius", seed.radius, "nominalSuccessorQuotientNorm", nominalNorm, ...
            "publishedEgoBoxRadius", radii, ...
            "closedLoopSpectralRadius", max(abs(eig(closedLoop))), ...
            "localClosedLoopWeightedNorm", norm(seed.factor*closedLoop/seed.factor, 2), ...
            "existingNonlinearContractionBound", seed.contractionBound, ...
            "outsideCoreCount", sum([trials.outsideEveryRigidlyPlacedNominalCore]), "trials", trials);
        cases = [cases; row]; %#ok<AGROW>
    end
    results = struct("scope", "offline nominal-feedback extrapolation; not a controller validation", ...
        "matlabRelease", string(version("-release")), "cases", cases, ...
        "checks", "Both exact trims remain inside; twelve admitted-box perturbations are evaluated without assuming their outcome.");
    if strlength(outputPath) > 0
        folder = fileparts(outputPath);
        if strlength(folder) > 0 && ~isfolder(folder), mkdir(folder); end
        file = fopen(outputPath, "w");
        assert(file >= 0, "probeObserverTerminalCompatibility:output", "Cannot write results.");
        closeFile = onCleanup(@() fclose(file));
        fprintf(file, "%s\n", jsonencode(results, PrettyPrint=true));
        clear closeFile;
    end
    fprintf("Terminal compatibility: 2 nominal trims and 12 estimate-error probes completed.\n");
end
