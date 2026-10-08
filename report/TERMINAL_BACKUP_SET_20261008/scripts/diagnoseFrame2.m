function diagnoseFrame2(file)
% Stage-level view of one failing frame, and the anchor's lateral profile.
    root='/home/zai/Downloads/ResearchProjects/collisionAvoidance';
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'));
    s=load(file,'snapshot');s=s.snapshot;
    [model,previous]=debugControllerModel(s.egoInput,s.targetEstimate,s.road,s.configuration,s.previousState);
    [~,search]=solvePredictiveControl(model,previous);
    disp(fieldnames(search.stages).');
    for k=1:numel(search.stages)
        g=search.stages(k);
        fprintf('stage %d: %s exit %g value %g numeric %d\n',k,string(g.objective),g.exitFlag,g.value,g.numericalSolve);
        if isfield(g,'solverInfo'),disp(g.solverInfo);end
    end
    if isstruct(previous)
        inputs=previous.inputTrajectory(:,2:end);x=model.initialState;lat=zeros(1,size(inputs,2));
        for k=1:size(inputs,2),[~,x]=terminalSafeSet.holdStates(x,inputs(:,k),model.cfg);p=laneGeometry.project(x(1:2),model.lane);lat(k)=p.lateralPosition;end
        fprintf('previous plan: %d holds, lateral at 1s %.2f 3s %.2f 6s %.2f end %.2f, end speed %.2f\n',numel(lat),lat(min(end,20)),lat(min(end,60)),lat(min(end,120)),lat(end),x(4));
        context=terminalSafeSet.context(model);[member,~,info]=terminalSafeSet.member(context,x,numel(lat));
        fprintf('shifted endpoint member %d lane %.2f reason %s exit %.2f\n',member,info.lateralOffset,info.reason,info.exitSeconds);
    end
end
