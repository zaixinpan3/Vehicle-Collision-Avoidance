function information=buildPredictiveConicSolver()
%buildPredictiveConicSolver Link the sparse conic QP adapter to Clarabel 0.11.1.
% Build the external Clarabel.cpp checkout first, with CMake in build/.
% Vendored sources and generated binaries remain under solver/.
    root=fileparts(fileparts(mfilename('fullpath')));
    dependency=fullfile(root,'solver','clarabel');output=fullfile(root,'solver','controller');
    cache=fullfile(dependency,'build','CMakeCache.txt');
    library=fullfile(dependency,'rust_wrapper','target','release','libclarabel_c.so');
    assert(isfile(cache) && isfile(library),'buildPredictiveConicSolver:missingDependency', ...
        'Build Clarabel.cpp under solver/clarabel with CMake before compiling this adapter.');
    definitions={};configuration=fileread(cache);
    features={'FAER_SPARSE','PARDISO_MKL','PARDISO_PANUA','SERDE'};
    for k=1:numel(features)
        if contains(configuration,['CLARABEL_FEATURE_',features{k},':BOOL=ON'])
            definitions{end+1}=['-DFEATURE_',features{k}]; %#ok<AGROW>
        end
    end
    assert(contains(configuration,'CLARABEL_FEATURE_SDP:STRING=none') ...
        && ~contains(configuration,'CLARABEL_FEATURE_PARDISO_MKL:BOOL=ON') ...
        && ~contains(configuration,'CLARABEL_FEATURE_PARDISO_PANUA:BOOL=ON'), ...
        'buildPredictiveConicSolver:unsupportedBuild','Use the non-SDP, non-Pardiso Clarabel build.');
    if ~isfolder(output),mkdir(output);end
    clear predictiveConicSolverMex;
    binary=fullfile(output,"predictiveConicSolverMex."+mexext);
    if isfile(binary),delete(binary);end
    manifestWarning=false;
    try
        mex('-R2018a',definitions{:},['-I',fullfile(dependency,'include')], ...
            fullfile(root,'scripts','native','predictiveConicSolverMex.cpp'),library, ...
            ['LDFLAGS=$LDFLAGS -Wl,-rpath,',fileparts(library)], ...
            '-ldl','-lpthread','-lm','-outdir',output,'-output','predictiveConicSolverMex');
    catch exception
        % R2026a's post-link manifest validator also rejects a minimal MEX
        % on this host. Accept only that specific diagnostic after loading
        % the newly built binary and solving an independent known problem.
        if ~strcmp(exception.identifier,'MATLAB:mex:Error') ...
                || ~contains(exception.message,"'ENOTMEX'") || ~isfile(binary)
            rethrow(exception);
        end
        manifestWarning=true;
    end
    addpath(output);rehash;
    [point,flag]=predictiveConicSolverMex(sparse(1),-2,sparse([1;-1]),[1;1],2,1,[100;5;1e-8;1e-8;1e-8]);
    assert(flag>0 && abs(point-1)<1e-7,'buildPredictiveConicSolver:smokeFailure', ...
        'The compiled solver did not solve the independent smoke problem.');
    if manifestWarning
        warning('buildPredictiveConicSolver:manifestValidation', ...
            'The host MEX manifest validator failed; the new binary loaded and passed its QP smoke test.');
    end
    information=struct('directory',output,'binary',"predictiveConicSolverMex."+mexext, ...
        'matlabVersion',string(version),'dependency',dependency,'manifestWarning',manifestWarning);
end
