function result = profileForceFreeAdmission(inputDirectory, outputDirectory)
%profileForceFreeAdmission Measure saved first-admission phases without plant execution.
% Three warm replay samples and one instrumented profile are observational.
    arguments
        inputDirectory (1,1) string
        outputDirectory (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    loaded=load(fullfile(inputDirectory,'original-functional.mat'),'unboundedRuntime');
    trial=loaded.unboundedRuntime;
    ego=trial.attempts.controllerEgoEstimate{1};
    road=trial.attempts.roadPerception{1}.roadGeometry;
    target=trial.attempts.targetEstimate{1};
    cfg=trial.controllerConfiguration;
    cfg.solver.jointFunction=@timedSolver;
    calls=struct('seconds',{},'variables',{},'rows',{},'cones',{},'iterations',{});
    result=struct();
    for replay=1:3
        replayTimer=tic;
        [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
        result.replay(replay).seconds=toc(replayTimer);
        result.replay(replay).phases=problem.metadata.runtime;
        result.replay(replay).certified=problem.metadata.planCertified;
    end
    result.solverCalls=calls;
    for replay=1:3
        assert(result.replay(replay).seconds>=sum([calls(2*replay-1:2*replay).seconds]));
    end
    profile on;
    [~,~,profiledProblem]=collisionAvoidanceController(ego,target,road,cfg,[]);
    profile off;
    profileData=profile('info');
    result.profiledAdmissionCertified=profiledProblem.metadata.planCertified;
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    save(fullfile(outputDirectory,'runtime.mat'),'result','profileData');
    file=fopen(fullfile(outputDirectory,'runtime-summary.json'),'w');assert(file>=0);
    cleanup=onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(result,PrettyPrint=true));
    disp(result.replay);disp(result.solverCalls);

    function solve=timedSolver(~,program)
        solveTimer=tic;solve=program.defaultSolver();seconds=toc(solveTimer);
        iterations=NaN;
        if isfield(solve.output,'iterations'),iterations=solve.output.iterations;end
        calls(end+1)=struct('seconds',seconds,'variables',numel(program.q), ...
            'rows',size(program.A,1),'cones',program.cones,'iterations',iterations);
    end
end
