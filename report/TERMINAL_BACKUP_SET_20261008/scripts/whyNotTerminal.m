function whyNotTerminal(file)
% For a failing first frame: the target estimate, and per backup lane whether
% a settled ego (level 0) at the station reached after 'node' holds of the
% backup completes the encounter, and why not.
    study='/home/zai/.cache/collisionAvoidance/terminal_backup_20261008';
    root='/home/zai/Downloads/ResearchProjects/collisionAvoidance';
    addpath(study,fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'));
    s=load(file,'snapshot');s=s.snapshot;
    [model,previous]=debugControllerModel(s.egoInput,s.targetEstimate,s.road,s.configuration,s.previousState);
    q=model.target;x=model.initialState;
    fprintf('%s\n',file);
    fprintf(' ego v %.3f m/s; target x %.2f y %.3f yaw %.5f V %.3f A %.3f beta %.5f\n',x(4),q(1),q(2),q(3),q(4),q(5),q(6));
    context=terminalSafeSet.context(model);h=model.cfg.controller.sampleTime;
    for m=1:numel(context.modes)
        [~,station]=terminalSafeSet.coordinates(context,x,m);
        line=sprintf(' lane d=%+.2f:',context.modes(m).lateralOffset);
        for node=[0 40 100 200]
            [ok,context,info]=terminalSafeSet.clear(context,0,station+context.modes(m).pathSpeed*node*h,node,m);
            line=[line sprintf('  node %d %s(%s %.1fs)',node,string(ok),info.reason,info.exitSeconds)]; %#ok<AGROW>
        end
        fprintf('%s\n',line);
    end
    [solution,search]=solvePredictiveControl(model,previous);
    a=search.acceptance;
    fprintf(' solve: returned %d, %s; candidateReason %s; reason %s\n',~isempty(solution),search.terminationReason,a.candidateReason,a.reason);
end
