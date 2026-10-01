root=fileparts(fileparts(fileparts(mfilename('fullpath'))));cd(root);
output=tempname;mkdir(output);
results=runtests({'tests/twoStagePredictiveControlTest.m','tests/clfNominalRecoveryTest.m'});
writetable(table(results),fullfile(output,'targeted.csv'));
save(fullfile(output,'targeted.mat'),'results');
assertSuccess(results);
