function summary = profileCapturedControllerFrame(captureFile, outputDirectory)
%profileCapturedControllerFrame Profile a recovered frame with fixed solver work.
% The fixed iteration cap reproduces the recorded work without allowing the
% profiler's overhead to change the wall-clock stopping decision.
    arguments
        captureFile (1,1) string
        outputDirectory (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    data=load(captureFile,'capture');capture=data.capture;
    cfg=capture.fixedWorkConfiguration;
    warmTimer=tic;
    [~,~,warm]=collisionAvoidanceController(capture.ego,capture.target, ...
        capture.road,cfg,capture.previousState);
    warmSeconds=toc(warmTimer);localVerify(warm,capture);
    runs=cell(1,3);
    for index=1:numel(runs)
        frameTimer=tic;
        [~,~,problem]=collisionAvoidanceController(capture.ego,capture.target, ...
            capture.road,cfg,capture.previousState);
        elapsed=toc(frameTimer);localVerify(problem,capture);
        runs{index}=localResult(problem,elapsed);
    end
    frameTimer=tic;
    [~,~,deadline]=collisionAvoidanceController(capture.ego,capture.target, ...
        capture.road,capture.configuration,capture.previousState);
    deadlineResult=localResult(deadline,toc(frameTimer));
    profile clear
    profile on
    profilerStatus=profile('status');frameTimer=tic;
    [~,~,profiled]=collisionAvoidanceController(capture.ego,capture.target, ...
        capture.road,cfg,capture.previousState);
    profiledSeconds=toc(frameTimer);
    profile off
    profileInfo=profile('info');localVerify(profiled,capture);
    summary=struct('frame',capture.frame,'scenario',capture.scenario, ...
        'originalSeconds',capture.originalSeconds,'warmupSeconds',warmSeconds, ...
        'fixedWorkRuns',[runs{:}],'deadlineReplay',deadlineResult, ...
        'profiledRun',localResult(profiled,profiledSeconds),'profilerStatus',profilerStatus);
    localWrite(fullfile(outputDirectory,'profile-summary.json'),summary);
    localWrite(fullfile(outputDirectory,'function-profile.json'),profileInfo.FunctionTable);
    save(fullfile(outputDirectory,'function-profile.mat'),'profileInfo','profiled','summary');
    findings=struct('capture',checkcode(fullfile(root,'scripts','captureLongestControllerFrame.m'), ...
        '-config=factory','-id'),'profile',checkcode(mfilename('fullpath'),'-config=factory','-id'));
    localWrite(fullfile(outputDirectory,'code-analysis.json'),findings);
end

function localVerify(problem,capture)
    assert(problem.metadata.solverCallCount==capture.originalTrace.solverCalls, ...
        'Fixed-work solver call count changed.');
    difference=problem.inputTrajectory-capture.fixedWorkSolution.inputs;
    assert(max(abs(difference),[],'all')<1e-9,'Fixed-work input trajectory changed.');
end

function result=localResult(problem,elapsed)
    search=problem.metadata.search;
    result=struct('seconds',elapsed,'solverCalls',search.solverCalls, ...
        'iterations',numel(search.sequentialIterations),'converged',search.converged, ...
        'terminationReason',search.terminationReason,'safetySlack',problem.solution.safety, ...
        'hardResidual',problem.solution.hard,'clfSlack',problem.solution.clfSlack, ...
        'firstInput',problem.inputTrajectory(:,1));
end

function localWrite(path,value)
    file=fopen(path,'w');assert(file>=0);cleanup=onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(value,PrettyPrint=true));
end
