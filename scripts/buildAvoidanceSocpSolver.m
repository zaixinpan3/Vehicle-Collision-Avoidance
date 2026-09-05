function information = buildAvoidanceSocpSolver()
%buildAvoidanceSocpSolver Build the native predictive SOCP backend on Linux.
% Run once before controller experiments. Requires Git, Rust/Cargo and a
% MATLAB C++ MEX compiler. The pinned Apache-2.0 Clarabel dependency and
% generated binaries stay under solver/; no network/build work occurs in
% the controller sample. Cargo dependencies are locked by the repository.

    if ~strcmp(computer("arch"), "glnxa64")
        error("buildAvoidanceSocpSolver:unsupportedPlatform", ...
            "This build recipe is validated for 64-bit Linux MATLAB.");
    end
    root = fileparts(fileparts(mfilename("fullpath")));
    dependency = fullfile(root, "solver", "clarabel");
    revision = "0de6259a3edfd5cc041ec42b2148599ce63e73cb";
    if ~isfolder(dependency)
        localRun("git clone https://github.com/oxfordcontrol/Clarabel.cpp.git " ...
            + localQuote(dependency));
        localRun("git -C "+localQuote(dependency)+" checkout --detach "+revision);
    end
    actual = strtrim(localRun("git -C "+localQuote(dependency)+" rev-parse HEAD"));
    if actual ~= revision
        error("buildAvoidanceSocpSolver:dependencyVersion", ...
            "Expected Clarabel.cpp %s; found %s in %s.", revision, actual, dependency);
    end
    localRun("git -C "+localQuote(dependency)+" submodule update --init --recursive");
    wrapper = fullfile(dependency, "rust_wrapper");
    copyfile(fullfile(root, "config", "clarabelCargo.lock"), fullfile(wrapper, "Cargo.lock"));
    localRun("cargo build --release --locked --jobs 4 --color never --manifest-path " ...
        + localQuote(fullfile(wrapper, "Cargo.toml")));
    output = fullfile(dependency, "matlab");
    if ~isfolder(output), mkdir(output); end
    addpath(output);
    clear solveHardCbfClf laneProjection laneFrameCertificate
    clear solveAvoidanceSocpMex projectLanePolylineMex laneFrameBoundsMex
    solverMessage = localBuild(fullfile(output, "solveAvoidanceSocpMex."+mexext), ...
        "-R2018a", "-I"+fullfile(dependency, "include"), ...
        fullfile(root, "controller", "solveAvoidanceSocpMex.cpp"), ...
        fullfile(wrapper, "target", "release", "libclarabel_c.a"), ...
        "-ldl", "-lpthread", "-lm", "-outdir", output);
    projectionMessage = localBuild(fullfile(output, "projectLanePolylineMex."+mexext), ...
        "-R2018a", fullfile(root, "controller", "projectLanePolylineMex.cpp"), ...
        "-outdir", output);
    frameMessage = localBuild(fullfile(output, "laneFrameBoundsMex."+mexext), ...
        "-R2018a", fullfile(root, "controller", "laneFrameBoundsMex.cpp"), "-outdir", output);
    projection = projectLanePolylineMex([0.5; 2.0], [0.0, 0.0], ...
        [1.0, 0.0], 1.0, 0.0, [1.0, 0.0]);
    assert(isequal(projection, [0.5; 0.0; 0.0; 0.5; 2.0]), ...
        "buildAvoidanceSocpSolver:projectionSmokeTest", "The native projection smoke test failed.");
    frame = laneFrameBoundsMex(0.5, 0.25, 2.0, [0.0, 0.0], 1.0, 0.0, [1.0, 0.0]);
    assert(isequal(frame, [0; 0; 1; 0; 0; 1; 0.25; 0.75; 0; 0; 0]), ...
        "buildAvoidanceSocpSolver:frameSmokeTest", "The native frame smoke test failed.");
    [point, check] = solveAvoidanceSocpMex(sparse(2, 2, 2, 2, 2), [0.0; 0.0], ...
        sparse([0.0, 0.0; -2.0, 0.0; 0.0, -1.0]), [1.0; -2.0; -2.0], ...
        [0; 0; 3], [1.0e-9, 1.0e-9, 400]);
    assert(any(check.status == [1, 4]) && max(abs(point-[1.0; 1.0])) < 1.0e-5, ...
        "buildAvoidanceSocpSolver:smokeTest", "The native SOCP smoke test failed.");
    information = struct("clarabelCppRevision", revision, ...
        "clarabelRustRevision", strtrim(localRun("git -C " ...
            + localQuote(fullfile(dependency, "Clarabel.rs"))+" rev-parse HEAD")), ...
        "binary", fullfile(output, "solveAvoidanceSocpMex."+mexext), ...
        "projectionBinary", fullfile(output, "projectLanePolylineMex."+mexext), ...
        "frameBinary", fullfile(output, "laneFrameBoundsMex."+mexext), ...
        "smokeTestPassed", true, ...
        "postLinkInspectionMessage", [solverMessage; projectionMessage; frameMessage], ...
        "matlabVersion", string(version));
    disp(information);
end

function postLinkMessage = localBuild(binary, varargin)
    % Never allow the subsequent load/smoke test to pass using a stale binary.
    if isfile(binary), delete(binary); end
    postLinkMessage = "";
    try
        mex(varargin{:});
    catch exception
        % R2026a's post-link inspection on this host can reject a valid ELF.
        % The independent load-and-execute smoke tests remain mandatory.
        if string(exception.identifier) ~= "MATLAB:mex:Error" ...
                || ~contains(exception.message, "is not a MEX file")
            rethrow(exception);
        end
        postLinkMessage = string(exception.message);
    end
    rehash;
end

function output = localRun(command)
    % External host tools must not load MATLAB's bundled C++/curl libraries.
    % Change only the child environment, leaving the MATLAB session intact.
    [status, output] = system("env -u LD_LIBRARY_PATH -u LD_PRELOAD "+command);
    output = string(output);
    if status ~= 0
        error("buildAvoidanceSocpSolver:commandFailed", "%s", output);
    end
end

function quoted = localQuote(value)
    mark = char(39);
    replacement = [mark, char(34), mark, char(34), mark];
    quoted = string([mark, strrep(char(value), mark, replacement), mark]);
end
