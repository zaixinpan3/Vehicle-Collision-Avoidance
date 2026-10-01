% Usage: set speed, name, frames before running.
addpath('/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/methods');
out=sprintf('/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/replay/%s-%d.mat',name,speed);
if ~exist('captureTimes','var'),captureTimes=[];end
t=tic;record=replayCase(speed,name,frames,out,captureTimes);
f=record.frame;
fprintf('REPLAY %s %d: %d frames in %.1f s\n',name,speed,numel(f),toc(t));
for k=1:numel(f)
    fprintf('t=%5.2f init=%-19s pcbf=%9.3g flags=%s clr=%8.4f affMin=%8.4f@%2d rollMin=%8.4f@%2d posErr=%7.4f exit=%g u=[%7.4f %7.4f] anchor0=[%7.4f %7.4f] V0=%9.4g V1aff=%9.4g V1act=%9.4g\n', ...
        f(k).time,f(k).initialization,f(k).primaryOptimum,mat2str(f(k).stageFlags),f(k).clearance,f(k).affineMargin,f(k).affineMarginStage, ...
        f(k).rolloutMargin,f(k).rolloutMarginStage,f(k).rolloutPositionError,f(k).encounterExit,f(k).input,f(k).anchorInputs(:,1), ...
        f(k).clfInitialValue,f(k).clfNextValue,f(k).actualClfNextValue);
end
