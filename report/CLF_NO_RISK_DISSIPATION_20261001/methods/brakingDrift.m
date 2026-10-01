% Braking-channel drift in the 15-m/s head-on no-risk phase.
source='/home/zai/.cache/collisionAvoidance/margin-5cm-20261001/source';
addpath(fullfile(source,'controller'),fullfile(source,'config'));
load('/home/zai/.cache/collisionAvoidance/clf-dissipation-20261001/replay/headOn-15-full.mat','record');f=record.frame;cfg=record.configuration;
trim=terminalContinuation.build(cfg,0).reference.input;n=numel(f);
ab=arrayfun(@(e)e.anchorInputs(2,1),f);ub=arrayfun(@(e)e.input(2),f);V0=[f.clfInitialValue];rho=[f.clfSlack];
fprintf('trim braking %.4f\n',trim(2));
k0=find([f.time]>=40 & ab<trim(2)-0.1,1);
fprintf('first late frame with anchor braking < trim-0.1: t=%.2f\n',f(k0).time);
for k=k0-12:2:k0+6
    d=f(k).plan(2,:)-f(k).anchorInputs(2,:);
    fprintf('t=%6.2f V0=%9.3g rho=%9.3g anchor b: first %+.4f mean %+.4f last %+.4f | issued %+.4f | plan b: first %+.4f second %+.4f mean %+.4f last %+.4f | corrections sum %+.4f max|.| %.4f\n', ...
        f(k).time,V0(k),rho(k),f(k).anchorInputs(2,1),mean(f(k).anchorInputs(2,:)),f(k).anchorInputs(2,end),ub(k), ...
        f(k).plan(2,1),f(k).plan(2,2),mean(f(k).plan(2,:)),f(k).plan(2,end),sum(d),max(abs(d)));
end
k=k0-4;fprintf('braking anchor profile at t=%.2f (every 5th stage): %s\n',f(k).time,num2str(f(k).anchorInputs(2,1:5:end),' %+.3f'));
fprintf('braking plan profile   at t=%.2f (every 5th stage): %s\n',f(k).time,num2str(f(k).plan(2,1:5:end),' %+.3f'));
fprintf('speed anchor profile   at t=%.2f (every 5th node): %s\n',f(k).time,num2str(f(k).anchorStates(4,1:5:end)-15,' %+.3f'));
fprintf('speed plan profile     at t=%.2f (every 5th node): %s\n',f(k).time,num2str(f(k).affineStates(4,1:5:end)-15,' %+.3f'));
