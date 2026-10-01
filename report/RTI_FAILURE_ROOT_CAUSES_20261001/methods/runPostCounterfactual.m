% Post-encounter counterfactuals; set cases {speed,name,variant} and frames before running.
addpath('/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/methods');
tol=[.1;pi/180;.1;.05;.01];
for c=1:numel(cases)
    speed=cases{c}{1};name=cases{c}{2};variant=cases{c}{3};
    out=sprintf('/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/cf/%s-%s-%d.mat',variant,name,speed);
    t=tic;record=replayCaseCF2(speed,name,frames,out,[],"/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/cf/"+variant,struct());
    f=record.frame;e=[f.error1];time=[f.time]+0.05;inside=all(abs(e)<=tol,1) & time>=8;
    recovered=NaN;start=NaN;
    for k=1:numel(time)
        if ~inside(k),start=NaN;elseif isnan(start),start=time(k);end
        if inside(k) && time(k)-start>=5-1e-10,recovered=time(k);break;end
    end
    fprintf('POST-DONE %s %s %d frames=%d wall=%.1f minClearance=%.4f recoveredAt=%.2f final e=[%s] failure=%s\n',variant,name,speed,numel(f),toc(t), ...
        min([f.clearance]),recovered,num2str(e(:,end).',' %.4g'),record.failure);
end
