load('/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/replay/headOn-15-full.mat','record');f=record.frame;
for k=find([f.time]>=66.5 & [f.time]<=71.0)
    if mod(k,3)~=0,continue;end
    fprintf('t=%6.2f ev=%+7.4f v=%6.3f u=[%+8.5f %+8.5f] anchor=[%+8.5f %+8.5f] u-a=[%+7.4f %+7.4f] u1-a1=[%+7.4f %+7.4f] V0=%9.4g V1aff=%9.4g V1act=%9.4g flags=%s exitStep=%g endpoint=%.3g\n', ...
        f(k).time,f(k).error0(3),f(k).state(4),f(k).input,f(k).anchorInputs(:,1),f(k).input-f(k).anchorInputs(:,1),f(k).plan(:,2)-f(k).anchorInputs(:,2), ...
        f(k).clfInitialValue,f(k).clfNextValue,f(k).actualClfNextValue,mat2str(f(k).stageFlags),f(k).encounterExit,f(k).endpointTerminal);
end
