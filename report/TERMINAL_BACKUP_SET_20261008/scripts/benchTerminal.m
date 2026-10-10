function benchTerminal(label,extraPath)
% Time terminalSafeSet.member on representative cases; extraPath (optional)
% is prepended so another terminalSafeSet.m takes precedence.
root='/home/zai/Downloads/ResearchProjects/collisionAvoidance';
addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'),fullfile(root,'solver','controller'));
if nargin>1,addpath(extraPath);end
cfg8=struct('referenceSpeed',8,'controller',struct('horizonSteps',8));
cases={ 'receding lead, settled',        [0;0;0;8;0;0], [20;0;0;10;0;0;1.6;2.4;.95;0;0], 0;
        'braking-lead estimate, settled',[0;0;0;8;0;0], [6.2;0;.0038;7.9;0;0;1.6;2.4;.95;0;0], 0;
        'slow lead 40 m, settled',       [0;0;0;8;0;0], [40;0;0;7.5;0;0;1.6;2.4;.95;0;0], 0;
        'crosser, offset 1 m ego',       [0;1;0;8;0;0], [20;-6;-pi/2;8;0;0;1.6;2.4;.95;0;0], 0;
        'head-on ahead 40 m, offset 1 m',[0;1;0;8;0;0], [40;3.66;pi;8;0;0;1.6;2.4;.95;0;0], 0;
        'circling target ahead',         [0;.5;0;8;0;0],[30;0;0;8;0;.05;1.6;2.4;.95;0;0], 0;
        'curve head-on, offset 1 m',     [0;1;0;8;0;0], [40;3.66;pi;8;0;0;1.6;2.4;.95;0;0], .005};
fprintf('%s\n',label);
for k=1:size(cases,1)
    model=localModel(cases{k,2},cases{k,3},cases{k,4},cfg8);
    context=terminalSafeSet.context(model);
    [member,~,info]=terminalSafeSet.member(context,model.initialState,0);
    n=200;timer=tic;
    for i=1:n,context=terminalSafeSet.context(model);terminalSafeSet.member(context,model.initialState,0);end
    fprintf('  %-32s member %d %-22s end %6.2f  %7.2f ms/call\n',cases{k,1},member,info.reason,info.exitSeconds,toc(timer)/n*1e3);
end
end
function model=localModel(x,q,curvature,override)
    cfg=collisionAvoidanceControllerConfig(override);if ~isfield(cfg.terminal,'horizonSeconds'),cfg.terminal.horizonSeconds=60;end
    if curvature==0
        road=struct('centerline',[-100,0;1000,0],'lateralClearance',[8.5344;12.192]);
    else
        road=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',curvature,'length',400),'lateralClearance',[8.5344;12.192]);
    end
    ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6), ...
        'longitudinalVelocity',x(4),'stateTime',0,'heldActuatorInput',[0;0]);
    [~,lane,roadOut]=readControllerInputs(ego,[],road,cfg);
    frame=predictiveSafetyGeometry.roadFrame(lane,roadOut);
    reference=nonlinearBicycleModel.cruise(cfg,frame(4));
    model=struct('cfg',cfg,'initialState',x,'target',q,'targetEpoch',q,'sampleIndex',0, ...
        'lane',lane,'road',roadOut,'nominalReference',reference);
end
