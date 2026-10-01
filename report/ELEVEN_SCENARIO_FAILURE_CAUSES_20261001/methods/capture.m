root='/home/zai/.cache/collisionAvoidance/eleven-failure-causes-20261001';
source='/home/zai/.cache/collisionAvoidance/direct-two-stage-scenarios-20261001/source';
cd(source);addpath(root,'scripts','controller','config');cases=cell(1,11);j=0;
for speed=[8,15]
    for name=["headOn","acceleratingHeadOn","brakingLead","crossing","turningCrossing","curvedHeadOn","curvedCrossing"]
        if (speed==8 && any(name==["crossing","curvedHeadOn"])) || (speed==15 && name=="curvedHeadOn"),continue;end
        data=load(fullfile(fileparts(source),'campaign',"speed"+speed,name+".mat"),'continuation');c=data.continuation;
        [~,~,road,cfg]=collisionThreatScenario(name,c.result.configuration);captured=[];
        try,captureController(c.ego,c.target,road,cfg,c.prior);
        catch exception
            assert(strcmp(exception.identifier,'diagnostic:captured'),getReport(exception,'extended'));
        end
        assert(~isempty(captured));j=j+1;
        cases{j}=struct('speed',speed,'scenario',name,'continuation',c,'capture',captured);
        fprintf('CAPTURE %g %s variables=%d rows=%d terminalRadius=%.9g mismatch=%.9g\n', ...
            speed,name,numel(captured.problem.lower),numel(captured.problem.b), ...
            captured.model.terminal.radius,norm(captured.model.initialState-captured.anchor.states(:,1),inf));
    end
end
save(fullfile(root,'problems.mat'),'cases','-v7.3');
