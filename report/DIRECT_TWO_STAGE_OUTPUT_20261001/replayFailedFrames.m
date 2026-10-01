root='/home/zai/Downloads/ResearchProjects/collisionAvoidance';
output='/home/zai/.cache/collisionAvoidance/direct-two-stage-20261001';
inputs='/home/zai/.cache/collisionAvoidance/two-stage-scenarios-20261001/campaign';
cd(root);addpath('controller','config','scripts','solver/controller');entries=struct([]);
for speed=[8,15]
    for name=["headOn","acceleratingHeadOn","brakingLead","crossing","turningCrossing","curvedHeadOn","curvedCrossing"]
        data=load(fullfile(inputs,"speed"+speed,name+".mat"),'continuation');c=data.continuation;
        [~,~,road,cfg]=collisionThreatScenario(name,c.result.configuration);
        entry=struct('speed',speed,'scenario',name,'returned',false,'solverFlags',[], ...
            'solverCalls',0,'solverConverged',false,'affineValidationPerformed',false, ...
            'input',[],'pcbfSlack',NaN,'clfSlack',NaN,'failure',"");
        try
            [command,plan,prediction]=collisionAvoidanceController(c.ego,c.target,road,cfg,c.prior);
            meta=prediction.metadata;entry.returned=meta.optimizationReturned;
            entry.solverFlags=[meta.search.stages.exitFlag];entry.solverCalls=meta.solverCallCount;
            entry.solverConverged=meta.optimizationConverged;
            entry.affineValidationPerformed=meta.affineValidationPerformed;
            entry.input=command.actuatorInput;entry.pcbfSlack=prediction.solution.safety;
            entry.clfSlack=prediction.solution.clfSlack;
            assert(isequal(command.actuatorInput,plan(:,1)));
            assert(~meta.affineValidationPerformed && ~meta.nonlinearValidationPerformed);
            assert(isnan(meta.hardConstraintResidual) && isnan(meta.minimumCollisionMargin));
        catch exception
            entry.failure=string(exception.identifier)+": "+string(exception.message);
        end
        if isempty(entries),entries=entry;else,entries(end+1)=entry;end
        fprintf('%g %s returned=%d flags=[%s] %s\n',speed,name,entry.returned,num2str(entry.solverFlags),entry.failure);
    end
end
file=fopen(fullfile(output,'failed-frame-replay.json'),'w');fprintf(file,'%s\n',jsonencode(entries));fclose(file);
expected=[true,true,true,false,true,false,true,true,true,true,true,true,false,true];
assert(isequal([entries.returned],expected));
assert(all([entries(expected).solverCalls]==2));
assert(all(arrayfun(@(x)isequal(x.solverFlags,[1,-7]),entries(expected))));
assert(all(contains([entries(~expected).failure],'pcbfNoNumericalResult')));
tests=load(fullfile(output,'tests.mat'),'results');results=tests.results;
summary=struct('matlabTests',numel(results),'matlabPassed',sum([results.Passed]), ...
    'matlabFailed',sum([results.Failed]),'matlabIncomplete',sum([results.Incomplete]), ...
    'savedFailedFrames',14,'returnedFrames',sum([entries.returned]), ...
    'noNumericalResultFrames',sum(~[entries.returned]),'fullCampaignRun',false);
file=fopen(fullfile(output,'validation.json'),'w');fprintf(file,'%s\n',jsonencode(summary));fclose(file);
fprintf('%s\n',jsonencode(summary));
