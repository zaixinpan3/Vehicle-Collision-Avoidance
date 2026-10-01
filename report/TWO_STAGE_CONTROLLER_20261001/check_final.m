root=fileparts(fileparts(fileparts(mfilename('fullpath'))));cd(root);
output=tempname;mkdir(output);
files={'scripts/runNonlinearPredictiveSafetyValidation.m','scripts/validateNonlinearPredictiveController.m', ...
    'tests/nonlinearPredictiveSafetyTest.m','tests/terminalContinuationTest.m', ...
    'tests/freePoseTerminalTest.m','tests/movingTargetFlowTest.m','tests/controllerKernelTest.m', ...
    'tests/collisionAvoidanceControllerConfigTest.m'};
findings=cell(size(files));
for index=1:numel(files)
    findings{index}=struct('file',files{index},'findings',checkcode(files{index},'-config=factory','-id'));
end
file=fopen(fullfile(output,'final-code-analysis.json'),'w');
fprintf(file,'%s\n',jsonencode([findings{:}],PrettyPrint=true));fclose(file);
disp(version);
