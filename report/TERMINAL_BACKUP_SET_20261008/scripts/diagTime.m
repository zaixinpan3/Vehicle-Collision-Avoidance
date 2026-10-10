function diagTime(file,label,extraPath)
% Time one recorded failing frame with the working-tree controller, optionally
% with another terminalSafeSet.m first on the path.
study='/home/zai/.cache/collisionAvoidance/terminal_backup_20261008';
root='/home/zai/Downloads/ResearchProjects/collisionAvoidance';
addpath(fullfile(root,'report','TERMINAL_BACKUP_SET_20261008','scripts'),fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'),fullfile(root,'solver','controller'));
if nargin>2,addpath(extraPath,'/home/zai/.cache/collisionAvoidance/terminal_backup_20261008/variantH/config');end
s=load(file,'snapshot');s=s.snapshot;
cfg=s.configuration;if nargin>2 && ~isfield(cfg.terminal,'horizonSeconds'),cfg.terminal.horizonSeconds=60;end
[model,previous]=debugControllerModel(s.egoInput,s.targetEstimate,s.road,cfg,s.previousState);
timer=tic;[solution,search]=solvePredictiveControl(model,previous,timer);elapsed=toc(timer);
fprintf('%s: frame %d returned %d (%s) init %.2f s total %.2f s anchorHolds %d seedReached %g\n',label,s.frame, ...
    ~isempty(solution),search.terminationReason,search.initializationSeconds,elapsed,search.anchorHolds,search.seedReachedTerminalSet);
end
