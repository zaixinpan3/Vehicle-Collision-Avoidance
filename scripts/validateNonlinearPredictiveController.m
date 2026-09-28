function summary = validateNonlinearPredictiveController(outputDirectory)
%validateNonlinearPredictiveController Reproduce the nonlinear rebuild checks.
% Writes compact test, analyzer and independent replay results to outputDirectory.
% The selected suites exercise the nonlinear controller and its shared kernels;
% this is not the complete repository suite of affine/estimator/perception studies.
    arguments
        outputDirectory (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));previous=pwd;
    cleanup=onCleanup(@()cd(previous));cd(root);
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    addpath('controller','config','scripts');
    native=string(tempname);mkdir(native);
    nativeCleanup=onCleanup(@()localRemoveNative(native));
    buildFialaIntervalVerifier(native);addpath(native);
    suites={'tests/nonlinearPredictiveSafetyTest.m','tests/fialaIntervalKernelTest.m', ...
        'tests/fialaFeedbackSampleTest.m','tests/modifiedFialaTireTest.m', ...
        'tests/collisionAvoidanceControllerConfigTest.m','tests/controllerSourceBudgetTest.m'};
    results=runtests(suites);entries=table(results);
    entries=entries(:,{'Name','Passed','Failed','Incomplete','Duration'});
    summary=struct('matlabVersion',version,'tests',{table2struct(entries)});
    localWrite(fullfile(outputDirectory,'tests.json'),summary);
    assertSuccess(results);
    files={'controller/collisionAvoidanceController.m','controller/nonlinearBicycleModel.m', ...
        'controller/nonlinearSafetyCertificate.m','controller/solveNonlinearPredictivePlan.m', ...
        'controller/fialaCertificate.m','config/collisionAvoidanceControllerConfig.m', ...
        'scripts/buildFialaIntervalVerifier.m','scripts/prepareCollisionAvoidanceController.m', ...
        'scripts/runNonlinearPredictiveSafetyValidation.m','scripts/validateNonlinearPredictiveController.m', ...
        'tests/nonlinearPredictiveSafetyTest.m','tests/controllerSourceBudgetTest.m'};
    analysis=cell(size(files));
    for index=1:numel(files)
        analysis{index}=struct('file',files{index},'findings',checkcode(files{index},'-config=factory','-id'));
    end
    localWrite(fullfile(outputDirectory,'code-analysis.json'),[analysis{:}]);
    runNonlinearPredictiveSafetyValidation(Scenarios=["recovery","circular","solverFailure"], ...
        Frames=8,OutputFile=fullfile(outputDirectory,'short-replays.json'));
    runNonlinearPredictiveSafetyValidation(Scenarios="storedPolicy",Frames=210, ...
        OutputFile=fullfile(outputDirectory,'stored-policy.json'));
    runNonlinearPredictiveSafetyValidation(Scenarios="oncoming",Frames=32, ...
        OutputFile=fullfile(outputDirectory,'oncoming.json'));
end

function localWrite(path,value)
    file=fopen(path,'w');assert(file>=0);cleanup=onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(value,PrettyPrint=true));
end

function localRemoveNative(path)
    clear fialaIntervalMex fialaFeedbackSampleMex nonlinearSafetyMex
    rmpath(path);
    if isfolder(path),rmdir(path,'s');end
end
