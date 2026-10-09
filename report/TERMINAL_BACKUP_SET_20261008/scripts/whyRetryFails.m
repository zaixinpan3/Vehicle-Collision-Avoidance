function whyRetryFails(file)
% The cone-side retry of a failing first frame: its length, lateral and speed
% ranges, and along the rollout the right-lane backup's CLF ratio and the
% reason its set rejects the state.
    study='/home/zai/.cache/collisionAvoidance/terminal_backup_20261008';
    root='/home/zai/Downloads/ResearchProjects/collisionAvoidance';
    addpath(study,fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'),fullfile(root,'solver','controller'));
    s=load(file,'snapshot');s=s.snapshot;
    [model,previous]=debugControllerModel(s.egoInput,s.targetEstimate,s.road,s.configuration,s.previousState);
    evalin('base','clear seedRetry seedAnchor');
    timer=tic;solvePredictiveControlDiag(model,previous,timer);
    a=evalin('base','seedAnchor');fprintf('%s\n first rollout: side %+d holds %d reached %d lateral [%.2f %.2f] speed [%.2f %.2f]\n', ...
        file,a.seedSide,size(a.inputs,2),a.seedReachedTerminalSet,min(a.states(2,:)),max(a.states(2,:)),min(a.states(4,:)),max(a.states(4,:)));
    if ~evalin('base','exist(''seedRetry'',''var'')'),fprintf(' no retry\n');return;end
    r=evalin('base','seedRetry');
    fprintf(' retry: side %+d holds %d reached %d lateral [%.2f %.2f] speed [%.2f %.2f]\n', ...
        r.seedSide,size(r.inputs,2),r.seedReachedTerminalSet,min(r.states(2,:)),max(r.states(2,:)),min(r.states(4,:)),max(r.states(4,:)));
    context=terminalSafeSet.context(model);
    [~,m]=min(abs([context.modes.lateralOffset]-(-3.6576)));
    for node=[20 40 60 80 100 150 200 300 400 512]
        x=r.states(:,node+1);
        [value,station]=terminalSafeSet.coordinates(context,x,m);
        [value,station]=terminalSafeSet.coordinates(context,x,m);
        [ok,context,info]=terminalSafeSet.clear(context,value,station,node,m);
        fprintf('  node %3d: y %.2f yaw %+.3f v %.2f vy %+.2f r %+.3f | right-lane ratio %.3g ok %d (%s, margin %.2f)\n', ...
            node,x(2),x(3),x(4),x(5),x(6),value/context.modes(m).levelMaximum,ok,info.reason,info.margin);
    end
end
