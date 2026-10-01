% Collision-encounter counterfactuals: set cases {speed,name,variantDir,extraConfig,label} and frames.
addpath('/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/methods');
for c=1:numel(cases)
    speed=cases{c}{1};name=cases{c}{2};variant=cases{c}{3};extra=cases{c}{4};label=cases{c}{5};
    out=sprintf('/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/cf/%s-%s-%d.mat',label,name,speed);
    override="";if strlength(variant)>0,override="/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/cf/"+variant;end
    t=tic;record=replayCaseCF2(speed,name,frames,out,[],override,extra);f=record.frame;
    [m,k]=min([f.clearance]);
    fprintf('CCF-DONE %s %s %d frames=%d wall=%.1f minClearance=%.5f at t=%.2f maxPcbf=%.3g failure=%s\n',label,name,speed,numel(f),toc(t),m,f(k).time,max([f.primaryOptimum]),record.failure);
end
