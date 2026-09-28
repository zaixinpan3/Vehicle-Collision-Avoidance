function summary = validateNonlinearPredictiveController(outputDirectory)
%validateNonlinearPredictiveController Reproduce the nominal PCBF/CLF/SCvx checks.
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
    suites={'tests/nonlinearPredictiveSafetyTest.m','tests/modifiedFialaTireTest.m', ...
        'tests/collisionAvoidanceControllerConfigTest.m','tests/controllerSourceBudgetTest.m'};
    results=runtests(suites);entries=table(results);
    entries=entries(:,{'Name','Passed','Failed','Incomplete','Duration'});
    summary=struct('matlabVersion',version,'tests',{table2struct(entries)});
    localWrite(fullfile(outputDirectory,'tests.json'),summary);
    assertSuccess(results);
    files={'controller/collisionAvoidanceController.m','controller/nonlinearBicycleModel.m', ...
        'controller/predictiveSafetyGeometry.m','controller/solveNonlinearPredictivePlan.m', ...
        'config/collisionAvoidanceControllerConfig.m','scripts/prepareCollisionAvoidanceController.m', ...
        'scripts/runNonlinearPredictiveSafetyValidation.m','scripts/validateNonlinearPredictiveController.m', ...
        'tests/nonlinearPredictiveSafetyTest.m','tests/controllerSourceBudgetTest.m'};
    analysis=cell(size(files));
    for index=1:numel(files)
        analysis{index}=struct('file',files{index},'findings',checkcode(files{index},'-config=factory','-id'));
    end
    localWrite(fullfile(outputDirectory,'code-analysis.json'),[analysis{:}]);
    runNonlinearPredictiveSafetyValidation(Scenarios=["recovery","circular","solverFailure"], ...
        Frames=40,OutputFile=fullfile(outputDirectory,'short-replays.json'));
    runNonlinearPredictiveSafetyValidation(Scenarios="storedPolicy",Frames=160, ...
        OutputFile=fullfile(outputDirectory,'stored-policy.json'));
    runNonlinearPredictiveSafetyValidation(Scenarios=["oncoming","turningTarget"],Frames=160, ...
        OutputFile=fullfile(outputDirectory,'oncoming.json'));
end

function localWrite(path,value)
    file=fopen(path,'w');assert(file>=0);cleanup=onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(value,PrettyPrint=true));
end
