function rows = auditEstimatorInLoop(sourceRoot,campaign,outputFile)
%auditEstimatorInLoop Truth audit of the estimator's published bounds (manuscript sync).
% Runs scripts/replayEstimatorValidity of the source tree sourceRoot (a git
% archive of the commit under test, with a solver link) on the seed-20261003
% runs of a campaign folder whose files are named speed<V>-<scenario>.json,
% and writes the returned rows to outputFile as JSON. The replay re-runs the
% estimator on the recorded truth; the controller is not called.
    cd(sourceRoot);addpath(fullfile(sourceRoot,'scripts'));
    rows=replayEstimatorValidity(string(campaign));
    fid=fopen(outputFile,'w');fprintf(fid,'%s\n',jsonencode(rows,'PrettyPrint',true));fclose(fid);
end
