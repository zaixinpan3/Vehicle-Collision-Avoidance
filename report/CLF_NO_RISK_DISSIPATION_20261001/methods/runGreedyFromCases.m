% Idealized one-step CLF policies started from each case's first no-risk state.
% Set cases {speed,name} and policies before running.
addpath('/home/zai/.cache/collisionAvoidance/clf-dissipation-20261001/methods');
source='/home/zai/.cache/collisionAvoidance/margin-5cm-20261001/source';
addpath(fullfile(source,'controller'),fullfile(source,'config'),fullfile(source,'scripts'));
for c=1:numel(cases)
    speed=cases{c}{1};name=cases{c}{2};
    load(sprintf('/home/zai/.cache/collisionAvoidance/clf-dissipation-20261001/replay/%s-%d-full.mat',name,speed),'record');f=record.frame;
    k=find([f.encounterExit]==0 & [f.time]>=2,1);
    if isempty(k),k=find([f.time]>=10,1);note="encounter never ends; target ignored";else,note="first no-risk frame";end
    x0=f(k).state;u0=f(max(1,k-1)).input;
    for policy=policies
        t=tic;res=greedyClf(speed,record.road,x0,u0,policy,1600);res.scenario=name;res.startTime=f(k).time;res.note=note;
        save(sprintf('/home/zai/.cache/collisionAvoidance/clf-dissipation-20261001/greedy/%s-%s-%d.mat',policy,name,speed),'res');
        fprintf('GREEDY %s %s %d start t=%.2f (%s) e0=[%s] -> steps %d, recovered after %g s, max|ey| %.1f, min speed %.2f, max|vy| %.2f, failure %s (%.0f s)\n', ...
            policy,name,speed,f(k).time,note,num2str(f(k).error0.',' %.3g'),res.steps,res.recoveredAfter,res.maxAbsLateral, ...
            min(res.X(4,:)),max(abs(res.X(5,:))),res.failure,toc(t));
    end
end
