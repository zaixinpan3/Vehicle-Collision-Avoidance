cd('/home/zai/Downloads/ResearchProjects/collisionAvoidance');
results=runtests('tests');
summary=table(string({results.Name}).',[results.Passed].',[results.Failed].',[results.Incomplete].',[results.Duration].', ...
    'VariableNames',{'Name','Passed','Failed','Incomplete','Seconds'});
writetable(summary,'/home/zai/.cache/collisionAvoidance/unified-clf-implementation-20261001/assessment/tests.csv');
save('/home/zai/.cache/collisionAvoidance/unified-clf-implementation-20261001/assessment/tests.mat','results');
fprintf('FINAL-TESTS total=%d passed=%d failed=%d incomplete=%d\n',numel(results),nnz([results.Passed]),nnz([results.Failed]),nnz([results.Incomplete]));
assertSuccess(results);
