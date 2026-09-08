function preparation = prepareCollisionAvoidanceController(ego, road, cfg)
%prepareCollisionAvoidanceController Exercise the held-flow and conic kernels.
% Three independent empty-target admissions are discarded. The supplied ego
% and road are used verbatim; preparation invents no target-motion or exit
% contracts and applies no commands. Explicit empty certificate state leaves
% the running controller untouched. Preparation is not a deadline guarantee.
    timer = tic;
    nativePath = fullfile(fileparts(fileparts(mfilename("fullpath"))),"solver","bicycle");
    if isfolder(nativePath),addpath(nativePath);end
    cfg = collisionAvoidanceControllerConfig(cfg);
    if isfield(ego, "targetEstimates"), ego = rmfield(ego, "targetEstimates"); end
    samples = zeros(1, 3);
    certified = false(1, 3);
    failures = strings(1, 3);
    for repetition = 1:3
        sampleTimer = tic;
        try
            [~, ~, problem] = collisionAvoidanceController(ego, [], road, cfg, []);
            certified(repetition) = problem.metadata.planCertified;
        catch exception
            if ~any(string(exception.identifier) == ["collisionAvoidanceController:noCertifiedContinuation", ...
                    "collisionAvoidanceController:invalidUncertaintyChart"])
                rethrow(exception);
            end
            failures(repetition) = string(exception.identifier);
        end
        samples(repetition) = toc(sampleTimer);
    end
    preparation = struct("performed", true, "elapsedSeconds", toc(timer), ...
        "callSeconds", samples, "attemptedCalls", numel(samples), ...
        "discardedCommandCount", nnz(certified), "probeCertified", certified, ...
        "failureIdentifier", failures, "allProbesCertified", all(certified), ...
        "computationalThreads", maxNumCompThreads, ...
        "nativeRolloutAvailable",exist("bicycleNominalKernelMex","file")==3, ...
        "nativeLinearizationAvailable",exist("bicycleLinearizationKernelMex","file")==3, ...
        "nativeGeometryAvailable",exist("avoidanceCellRowsKernelMex","file")==3, ...
        "scope", "Before periodic sampling; independent empty-target admissions; no commands applied");
end
