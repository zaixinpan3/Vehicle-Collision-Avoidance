% Full-length instrumented replays; set cases (cell of {speed,name}) before running.
addpath('/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/methods');
for c=1:numel(cases)
    speed=cases{c}{1};name=cases{c}{2};
    out=sprintf('/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/replay/%s-%d-full.mat',name,speed);
    t=tic;record=replayCase(speed,name,1600,out);
    fprintf('REPLAY-DONE %s %d frames=%d wall=%.1f\n',name,speed,numel(record.frame),toc(t));
end
