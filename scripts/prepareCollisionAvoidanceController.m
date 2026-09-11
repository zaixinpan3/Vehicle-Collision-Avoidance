function preparation = prepareCollisionAvoidanceController(ego, road, cfg, target)
%prepareCollisionAvoidanceController Exercise the held-flow and conic kernels.
% Optional target probes use the supplied exact scene and discard commands.
% Without a target, only native paths are prepared. No missing observation is
% synthesized and no probe replaces a running certificate.
    if nargin < 4, target = []; end
    timer = tic;
    nativePath = fullfile(fileparts(fileparts(mfilename("fullpath"))),"solver","bicycle");
    if isfolder(nativePath),addpath(nativePath);end
    cfg = collisionAvoidanceControllerConfig(cfg);
    if isfield(ego, "targetEstimates"), ego = rmfield(ego, "targetEstimates"); end
    probeCount = 3*~isempty(target);
    samples = zeros(1, probeCount);
    certified = false(1, probeCount);
    failures = strings(1, probeCount);
    for repetition = 1:probeCount
        sampleTimer = tic;
        try
            [~, ~, problem] = collisionAvoidanceController(ego, target, road, cfg, []);
            certified(repetition) = problem.metadata.planCertified;
        catch exception
            if ~startsWith(string(exception.identifier), "collisionAvoidanceController:")
                rethrow(exception);
            end
            failures(repetition) = string(exception.identifier);
        end
        samples(repetition) = toc(sampleTimer);
    end
    preparation = struct("performed", true, "elapsedSeconds", toc(timer), ...
        "callSeconds", samples, "attemptedCalls", numel(samples), ...
        "discardedCommandCount", nnz(certified), "probeCertified", certified, ...
        "failureIdentifier", failures, "allProbesCertified", ~isempty(certified) && all(certified), ...
        "computationalThreads", maxNumCompThreads, ...
        "nativeRolloutAvailable",exist("bicycleNominalKernelMex","file")==3, ...
        "nativeLinearizationAvailable",exist("bicycleLinearizationKernelMex","file")==3, ...
        "nativeGeometryAvailable",exist("avoidanceCellRowsKernelMex","file")==3, ...
        "scope", "Before periodic sampling; explicitly supplied target probes; no commands applied");
end
