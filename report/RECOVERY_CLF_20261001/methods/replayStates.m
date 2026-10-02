% Recovery feedback from no-risk states that occurred in the 5-cm baseline replays
% (every 10th free frame of the 14 cases), with the repository functions.
repo='/home/zai/.cache/collisionAvoidance/recovery-clf-20261001/source';
addpath(fullfile(repo,'controller'),fullfile(repo,'config'));
D='/home/zai/.cache/collisionAvoidance/clf-dissipation-20261001/replay';files=dir(fullfile(D,'*-full.mat'));rows={};
tolerance=[.1;pi/180;.1;.05;.01];
for i=1:numel(files)
    load(fullfile(files(i).folder,files(i).name),'record');f=record.frame;cfg=record.configuration;
    cfg.recovery=collisionAvoidanceControllerConfig(struct('referenceSpeed',record.speed)).recovery;
    x=f(1).state;ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6));
    [~,lane]=readControllerInputs(ego,[],record.road,cfg);frame=predictiveSafetyGeometry.roadFrame(lane,[]);
    reference=nonlinearBicycleModel.cruise(cfg,frame(4));terminal=nonlinearBicycleModel.recoveryTerminal(cfg,frame(4));
    k=find([f.encounterExit]==0);k=k(1:10:end);
    for j=k
        x=f(j).state;u=f(max(1,j-1)).input;t=tic;
        [value,~,steps,converged]=nonlinearBicycleModel.recoveryValue(x,u,lane,reference,terminal,cfg);
        inside=0;entered=NaN;failure="";
        for step=1:round(150/cfg.controller.sampleTime)
            try
                u=nonlinearBicycleModel.recoveryInput(x,u,lane,reference,cfg,terminal);x=nonlinearBicycleModel.sample(x,u,cfg);
            catch exception
                failure=string(exception.identifier);break;
            end
            if all(abs(nonlinearBicycleModel.error(x,lane,reference))<=tolerance),inside=inside+1;else,inside=0;end
            if inside*cfg.controller.sampleTime>=5-1e-9,entered=step*cfg.controller.sampleTime-5;break;end
        end
        rows(end+1,:)={string(record.scenario),record.speed,f(j).time,f(j).error0(1),f(j).error0(2),value,steps*cfg.controller.sampleTime,converged,entered,failure,toc(t)}; %#ok<AGROW>
    end
end
T=cell2table(rows,'VariableNames',["scenario","speed","time","lateral","heading","value","rolloutSeconds","valueConverged","enteredToleranceAt","failure","wallSeconds"]);
writetable(T,'/home/zai/.cache/collisionAvoidance/recovery-clf-20261001/replay-states.csv');
fprintf('%d recorded no-risk states: value converged %d; entered and held tolerances %d (latest %.2f s); domain failures %d\n', ...
    height(T),nnz(T.valueConverged),nnz(isfinite(T.enteredToleranceAt)),max(T.enteredToleranceAt),nnz(T.failure~=""));
fprintf('lateral offsets %.1f..%.1f m; rollout median %.1f s, max %.1f s\n',min(T.lateral),max(T.lateral),median(T.rolloutSeconds),max(T.rolloutSeconds));
