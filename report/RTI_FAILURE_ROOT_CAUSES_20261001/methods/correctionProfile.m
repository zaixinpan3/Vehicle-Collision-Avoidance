% Steering correction profile over the horizon at one frame (set file, t0).
load(file,'record');f=record.frame;k=find(abs([f.time]-t0)<1e-9,1);
d=f(k).plan(1,:)-f(k).anchorInputs(1,:);
fprintf('%s t=%.2f: steering corrections by stage (rad):\n',file,t0);
fprintf('%s\n',num2str(d,'%+.4f '));
fprintf('sum=%+.4f, sum of stages 2..end=%+.4f; endpoint heading plan-anchor=%+.4f rad; endpoint yaw rate plan-anchor=%+.5f; plan heading change over horizon %+.3f vs anchor %+.3f\n', ...
    sum(d),sum(d(2:end)),f(k).affineStates(3,end)-f(k).anchorStates(3,end),f(k).affineStates(6,end)-f(k).anchorStates(6,end), ...
    f(k).affineStates(3,end)-f(k).affineStates(3,1),f(k).anchorStates(3,end)-f(k).anchorStates(3,1));
dx=f(k).affineStates-f(k).anchorStates;
fprintf('heading deviation plan-anchor at stages 1,5,10,20,40,end: %s\n',num2str(dx(3,[2,6,11,21,41,end]),'%+.4f '));
