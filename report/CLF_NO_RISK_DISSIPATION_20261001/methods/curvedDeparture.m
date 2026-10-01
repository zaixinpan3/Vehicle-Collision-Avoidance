% 8- and 15-m/s curvedHeadOn: the ego reaches the curved path and then leaves it.
% Prints the no-risk history against the curved trim (the CLF reference) and the
% straight trim (the terminal reference and the appended tail input).
D='/home/zai/.cache/collisionAvoidance/clf-dissipation-20261001';
source='/home/zai/.cache/collisionAvoidance/margin-5cm-20261001/source';
addpath(fullfile(source,'controller'),fullfile(source,'config'));
for speed=[8,15]
    load(fullfile(D,'replay',sprintf('curvedHeadOn-%d-full.mat',speed)),'record');f=record.frame;cfg=record.configuration;
    curved=nonlinearBicycleModel.cruise(cfg,.005).input;straight=terminalContinuation.build(cfg,0).reference.input;
    fprintf('curvedHeadOn %d m/s: curved trim steering %.4f rad, straight trim %.4f rad (difference %.4f)\n',speed,curved(1),straight(1),curved(1)-straight(1));
    fprintf('    t     e_y     e_psi    e_v   | issued-curved anchor-curved | rho>0  edge | free\n');
    for t=[4,6,8,10,15,20,25,30,40,50,60,70,79.95]
        k=find(abs([f.time]-t)<1e-9,1);e=f(k).error0;d=f(k).plan(1,1)-f(k).anchorInputs(1,1);
        fprintf('  %5.2f %7.2f %+7.3f %+6.2f |   %+8.4f      %+8.4f    | %5d %5d | %d\n',t,e(1),e(2),e(3), ...
            f(k).input(1)-curved(1),f(k).anchorInputs(1,1)-curved(1),f(k).clfSlack>1e-6*max(1,f(k).clfInitialValue),abs(d)>0.0749,f(k).encounterExit==0);
    end
    window=find([f.time]>=20 & [f.encounterExit]==0);
    fprintf('  20-80 s free frames: median issued-curved %+.4f rad, median anchor-curved %+.4f rad, median e_psi %+.4f rad, mean de_y/dt %+.3f m/s\n', ...
        median(arrayfun(@(k)f(k).input(1),window))-curved(1),median(arrayfun(@(k)f(k).anchorInputs(1,1),window))-curved(1), ...
        median(arrayfun(@(k)f(k).error0(2),window)),(f(window(end)).error0(1)-f(window(1)).error0(1))/(f(window(end)).time-f(window(1)).time));
end
