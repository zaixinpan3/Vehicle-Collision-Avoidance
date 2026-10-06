function diagnoseFailure(file)
    root='/home/zai/Downloads/ResearchProjects/collisionAvoidance';
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'));
    s=load(file,'snapshot');s=s.snapshot;
    [model,previous]=debugControllerModel(s.egoInput,s.targetEstimate,s.road,s.configuration,s.previousState);
    [solution,search]=solvePredictiveControl(model,previous);
    fprintf('%s frame %d returned %d reason %s\n',file,s.frame,~isempty(solution),search.terminationReason);
    if isfield(search,'acceptance'),disp(search.acceptance);end
    fprintf('anchorCertified %d appended %d\n',isfield(search,'anchorCertified')&&search.anchorCertified,search.appendedTerminalSteps*isfield(search,'appendedTerminalSteps'));
    for k=1:numel(search.attempts)
        a=search.attempts(k);flags=[a.stages.exitFlag];
        fprintf(' attempt %d %s %s scale %.3f flags %s\n',k,a.initialization,a.terminationReason,a.inputTrustScale,mat2str(flags));
    end
    % terminal-set view of the shifted anchor
    context=terminalSafeSet.context(model);
    if isstruct(previous)
        inputs=previous.inputTrajectory(:,2:end);x=model.initialState;
        for k=1:size(inputs,2),[~,x]=terminalSafeSet.holdStates(x,inputs(:,k),model.cfg);end
        [member,context,info]=terminalSafeSet.member(context,x,size(inputs,2));
        [level,~,linfo]=terminalSafeSet.level(context,x,size(inputs,2));
        fprintf('shifted endpoint: member %d V %.4f reason %s level %.4f exit %.2f margin %.3f\n',member,info.value,info.reason,level,info.exitSeconds,info.margin);
    end
end
