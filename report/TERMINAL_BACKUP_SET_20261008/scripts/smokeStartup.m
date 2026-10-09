function smokeStartup(file)
% Solve one failing first frame with the working-tree controller: seed side,
% retry, terminal set reached, solution returned, and time.
    study='/home/zai/.cache/collisionAvoidance/terminal_backup_20261008';
    root='/home/zai/Downloads/ResearchProjects/collisionAvoidance';
    addpath(study,fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'),fullfile(root,'solver','controller'));
    s=load(file,'snapshot');s=s.snapshot;
    [model,previous]=debugControllerModel(s.egoInput,s.targetEstimate,s.road,s.configuration,s.previousState);
    timer=tic;[solution,search]=solvePredictiveControl(model,previous,timer);elapsed=toc(timer);
    [~,name]=fileparts(file);
    fprintf('%s: seedSide %+d retried %d reached %d holds %d returned %d (%s) lane %+.2f end %s; %.1f s (init %.1f s)\n', ...
        name,search.seedSide,search.seedRetried,search.seedReachedTerminalSet,search.anchorHolds,~isempty(solution), ...
        search.terminationReason,search.acceptance.terminalLaneOffset,search.acceptance.terminalEnd,elapsed,search.initializationSeconds);
end
