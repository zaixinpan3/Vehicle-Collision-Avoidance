function summary=benchmarkMatlabControllerFixtures(fixturesDirectory,outputFile,options)
%benchmarkMatlabControllerFixtures Time identical inputs against saved decisions.
% Use a fresh MATLAB -singleCompThread process for each controller directory.
% A frozen external source copy permits paired baseline/current comparisons;
% no historical controller implementation is stored in this repository.
% Warmup is excluded, but target-admission fixtures are measured explicitly.
    arguments
        fixturesDirectory (1,1) string
        outputFile (1,1) string
        options.ControllerDirectory (1,1) string = ""
        options.Warmups (1,1) double {mustBeInteger,mustBeNonnegative} = 5
        options.Repetitions (1,1) double {mustBeInteger,mustBePositive} = 21
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'), ...
        fullfile(root,'scripts'),fullfile(root,'solver','clarabel','matlab'));
    source=options.ControllerDirectory;
    if source=="",source=fullfile(root,'controller');end
    addpath(source,'-begin');
    assert(strcmp(which('collisionAvoidanceController'),fullfile(source,'collisionAvoidanceController.m')));
    previousThreads=maxNumCompThreads(1);cleanup=onCleanup(@()maxNumCompThreads(previousThreads));
    profile off;
    replay=jsondecode(fileread(fullfile(fixturesDirectory,'replay.json')));
    summary=struct('scope',"Warm MATLAB controller-only replay of identical saved inputs", ...
        'source',source,'matlabVersion',string(version),'warmups',options.Warmups, ...
        'repetitions',options.Repetitions,'fixtures',{{}});
    for index=1:numel(replay.fixtures)
        item=replay.fixtures(index);loaded=load(item.file,'fixture');fixture=loaded.fixture;
        for warmup=1:options.Warmups,localInvoke(fixture);end
        samples=cell(options.Repetitions,1);
        for repetition=1:options.Repetitions
            samples{repetition}=localInvoke(fixture);
        end
        seconds=cellfun(@(sample)sample.seconds,samples);
        entry=struct('definition',item.definition,'samples',{samples}, ...
            'medianMs',1000*median(seconds),'maximumMs',1000*max(seconds), ...
            'over50Ms',nnz(seconds>.05),'decisionsExactlyEqual',true);
        summary.fixtures{end+1}=entry;
        fid=fopen(outputFile,'w');assert(fid>=0,'Cannot open benchmark output.');
        closer=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(summary));clear closer
        fprintf('%s: median/max %.3f/%.3f ms; decision unchanged\n', ...
            item.definition.name,entry.medianMs,entry.maximumMs);
    end
end

function sample=localInvoke(fixture)
    timer=tic;
    try
        [command,~,problem]=collisionAvoidanceController( ...
            fixture.ego,fixture.target,fixture.road,fixture.cfg,fixture.stored);
        seconds=toc(timer);
    catch exception
        seconds=toc(timer);
        if ~fixture.expectedFailure,rethrow(exception);end
        assert(strcmp(exception.identifier,'collisionAvoidanceController:optimizationFailed'));
        sample=struct('seconds',seconds,'failed',true,'message',string(exception.message));return;
    end
    assert(~fixture.expectedFailure,'A previously failed admission unexpectedly succeeded.');
    assert(isequaln(problem.decision,fixture.expectedDecision),'Complete controller decision changed.');
    assert(problem.metadata.planCertified,'Returned decision is not certified.');
    sample=struct('seconds',seconds,'failed',false,'input',command.actuatorInput, ...
        'phase',problem.metadata.runtime,'horizonSteps',problem.metadata.horizonSteps, ...
        'solverCalls',problem.metadata.admissionSearch.nativeSolves);
end
