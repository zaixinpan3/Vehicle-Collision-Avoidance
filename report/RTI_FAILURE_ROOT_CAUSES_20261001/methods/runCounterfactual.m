% Closed-loop counterfactuals; set cases (cell of {speed,name,variant}) and frames.
addpath('/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/methods');
for c=1:numel(cases)
    speed=cases{c}{1};name=cases{c}{2};variant=cases{c}{3};
    out=sprintf('/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/cf/%s-%s-%d.mat',variant,name,speed);
    t=tic;record=replayCaseCF(speed,name,frames,out,[],"/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/cf/"+variant);
    f=record.frame;e=[f.error1];
    fprintf('CF-DONE %s %s %d frames=%d wall=%.1f minClearance=%.4f final e=[%s] failure=%s\n',variant,name,speed,numel(f),toc(t), ...
        min([f.clearance]),num2str(e(:,end).',' %.4g'),record.failure);
end
