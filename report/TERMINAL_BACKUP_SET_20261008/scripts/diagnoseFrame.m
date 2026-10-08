function diagnoseFrame(file)
% Rebuild one failing frame from a failure snapshot and print the search.
    root='/home/zai/Downloads/ResearchProjects/collisionAvoidance';
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'));
    s=load(file,'snapshot');s=s.snapshot;
    [model,previous]=debugControllerModel(s.egoInput,s.targetEstimate,s.road,s.configuration,s.previousState);
    fprintf('frame %d target q = %s\n',s.frame,mat2str(model.target(1:6).',4));
    [solution,search]=solvePredictiveControl(model,previous);
    fprintf('returned %d reason %s anchorHolds %d seedReached %g returnedHolds %d\n',~isempty(solution), ...
        search.terminationReason,search.anchorHolds,search.seedReachedTerminalSet,search.returnedHolds);
    a=search.acceptance;
    fprintf('acceptance: step %g candidateFeasible %d candidateReason %s reason %s level %.4f iterations %d fullGap %.3f\n', ...
        a.step,a.candidateFeasible,a.candidateReason,a.reason,a.terminalLevel,a.iterations,a.fullGap);
    for k=1:numel(search.attempts)
        b=search.attempts(k);
        fprintf(' attempt %d %s %s scale %.3f\n',k,b.initialization,b.terminationReason,b.inputTrustScale);
    end
end
