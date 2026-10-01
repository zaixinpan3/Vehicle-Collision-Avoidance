root='/home/zai/Downloads/ResearchProjects/collisionAvoidance';out='/home/zai/.cache/collisionAvoidance/unbounded-steering-20261001';old='/home/zai/.cache/collisionAvoidance/eleven-failure-causes-20261001';cd(root);addpath('controller','config','scripts');results=struct([]);
data=load(fullfile(old,'problems.mat'),'cases');
for index=1:numel(data.cases)
    c=data.cases{index};saved=c.continuation;[~,~,road,cfg]=collisionThreatScenario(c.scenario,localConfig(saved.result.configuration));
    entry=localReplay(c.speed,c.scenario,saved.ego,saved.target,road,cfg,saved.prior);results=[results,entry];
end
save(fullfile(out,'late-replay.mat'),'results');
data=load(fullfile(old,'early-problems.mat'),'cases');
for index=1:numel(data.cases)
    c=data.cases{index};entry=localReplay(c.speed,c.scenario,c.ego,c.target,c.road,localConfig(c.capture.model.cfg),c.prior);results=[results,entry];
end
file=fopen(fullfile(out,'replay.json'),'w');fprintf(file,'%s\n',jsonencode(results));fclose(file);
function cfg=localConfig(cfg)
    cfg.model=rmfield(cfg.model,{'frontWheelSteeringAngleMaximum','frontWheelSteeringRateMaximum'});
    cfg=collisionAvoidanceControllerConfig(cfg);
end
function entry=localReplay(speed,name,ego,target,road,cfg,prior)
    % Only migrate the version and removed model fields. Retain the exact
    % old anchor, target epoch, input memory and terminal core for isolation.
    prior.version=62;prior.context{4}=cfg.model;
    entry=struct('speed',speed,'scenario',name,'time',ego.stateTime,'returned',false,'failure',"", ...
        'seconds',NaN,'flags',[],'input',[],'anchorInput',[],'maximumSteeringRadians',NaN, ...
        'maximumSteeringStepRadians',NaN,'pcbfSlack',NaN,'clfSlack',NaN, ...
        'clfInitialValue',NaN,'clfNextValue',NaN,'actualClfNextValue',NaN,'replayFailure',"");
    timer=tic;
    try
        [command,inputs,p]=collisionAvoidanceController(ego,target,road,cfg,prior);entry.seconds=toc(timer);entry.returned=true;
        entry.flags=[p.metadata.search.stages.exitFlag];entry.input=command.actuatorInput;
        entry.anchorInput=p.model.linearization.inputs(:,1);entry.maximumSteeringRadians=max(abs(inputs(1,:)));
        entry.maximumSteeringStepRadians=max(abs(diff([ego.heldActuatorInput(1),inputs(1,:)])));
        entry.pcbfSlack=p.metadata.search.primaryOptimum;entry.clfSlack=p.solution.clfSlack;entry.clfInitialValue=p.solution.clfInitialValue;entry.clfNextValue=p.solution.clfNextValue;
        try
            [~,states]=ode45(@(~,x)nonlinearBicycleModel.derivative(x,entry.input,cfg),[0,.05],p.model.initialState,odeset('RelTol',1e-11,'AbsTol',1e-12));
            error=nonlinearBicycleModel.error(states(end,:).',p.model.lane,p.model.nominalReference);entry.actualClfNextValue=norm(p.model.nominalReference.factor*error)^2;
        catch e,entry.replayFailure=string(e.identifier)+": "+string(e.message);end
    catch e,entry.seconds=toc(timer);entry.failure=string(e.identifier)+": "+string(e.message);end
    fprintf('REPLAY %g %s %.2f returned=%d flags=[%s] input=[%s] maxSteer=%.6g rho=%.6g failure=%s\n',speed,name,ego.stateTime,entry.returned,num2str(entry.flags),num2str(entry.input.'),entry.maximumSteeringRadians,entry.clfSlack,entry.failure);
end
