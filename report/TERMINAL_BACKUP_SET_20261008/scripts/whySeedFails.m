function whySeedFails(file)
% The startup seed of a failing first frame: its length, whether it reached
% the terminal set, its endpoint, and per backup lane the endpoint's CLF
% ratio (value / certified level) and membership.
    study='/home/zai/.cache/collisionAvoidance/terminal_backup_20261008';
    root='/home/zai/Downloads/ResearchProjects/collisionAvoidance';
    addpath(study,fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'),fullfile(root,'solver','controller'));
    s=load(file,'snapshot');s=s.snapshot;
    [model,previous]=debugControllerModel(s.egoInput,s.targetEstimate,s.road,s.configuration,s.previousState);
    [~,search]=solvePredictiveControlDiag(model,previous);
    a=evalin('base','seedAnchor');y=a.states(:,end);count=size(a.inputs,2);
    fprintf('%s\n seed holds %d reached %d; endpoint x %.1f y %.2f yaw %.3f v %.2f; terminated %s\n',file,count, ...
        a.seedReachedTerminalSet,y(1),y(2),y(3),y(4),search.terminationReason);
    lateral=a.states(2,:);fprintf(' seed lateral position: min %.2f max %.2f\n',min(lateral),max(lateral));
    context=terminalSafeSet.context(model);
    for m=1:numel(context.modes)
        [value,station]=terminalSafeSet.coordinates(context,y,m);
        [ok,context,info]=terminalSafeSet.clear(context,value,station,count,m);
        fprintf(' lane d=%+.2f: ratio %.3g member %d (%s)\n',context.modes(m).lateralOffset, ...
            value/context.modes(m).levelMaximum,ok,info.reason);
    end
end
