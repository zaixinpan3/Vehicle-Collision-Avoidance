function summary=runTaperedAdmissionValidation(directory,options)
%runTaperedAdmissionValidation Warm timing and capability of restricted admission.
% Run modes sequentially in fresh single-thread MATLAB processes. Warm every
% scenario before measuring; keep failed attempts and diagnostic budgets visible.
    arguments
        directory (1,1) string
        options.Mode (1,1) string {mustBeMember(options.Mode,["replay","sensitivity","campaign"])} = "replay"
        options.FixtureDirectory (1,1) string = "/home/zai/.cache/collisionAvoidance/longest-frame-profile-20260921"
        options.SampleCount (1,1) double {mustBePositive,mustBeInteger} = 600
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'), ...
        fullfile(root,'tests'),fullfile(root,'solver','clarabel','matlab'));
    originalThreads=maxNumCompThreads(1);cleanup=onCleanup(@()maxNumCompThreads(originalThreads));
    if ~isfolder(directory),mkdir(directory);end
    summary=struct('matlabVersion',string(version),'mode',options.Mode, ...
        'scope',"Warm declared-model measurements; unchanged hard checks; no WCET claim",'entries',{{}});
    variants=[1,32,16;.8,32,16;1,16,8;.8,16,8];
    names=["plateauFine","taperFine","plateauCoarse","taperCoarse"];
    if options.Mode=="replay"
        for name=["measured-longest","circular-crossing-longest"]
            loaded=load(fullfile(options.FixtureDirectory,name+'-fixture.mat'),'fixture');fixture=loaded.fixture;
            for round=1:2
                order=1:4;if round==2,order=4:-1:1;end
                for index=order
                    fixture.cfg=localConfig(fixture.cfg,variants(index,:));
                    for warmup=1:5,localInvoke(fixture);end
                    samples=cell(11,1);decision=[];
                    for repetition=1:11
                        [sample,current]=localInvoke(fixture);
                        assert(sample.certified,'Replay unexpectedly rejected admission.');
                        if repetition==1,decision=current;end
                        assert(isequaln(current,decision),'Repeated decision changed.');
                        if index==1
                            assert(max(abs(current-fixture.expectedDecision))<=1e-9,'Legacy replay changed.');
                        end
                        samples{repetition}=sample;
                    end
                    entry=struct('fixture',name,'variant',names(index),'round',round,'samples',{samples});
                    summary.entries{end+1}=entry;
                    localSave(directory,summary);
                    fprintf('%s %s round %d median/max %.3f/%.3f ms\n',name,names(index),round, ...
                        1000*median(cellfun(@(s)s.seconds,samples)),1000*max(cellfun(@(s)s.seconds,samples)));
                end
            end
        end
    elseif options.Mode=="sensitivity"
        cases=localCases();
        for index=1:size(cases,1)
            value=cases(index,:);
            [ego,target,road,cfg]=encounterTestFixture.circularCrossing(value(1));
            cfg.referenceSpeed=value(2);state=ltvBicycleModel.cruiseEquilibrium(value(1),cfg);
            ego.yaw=state(3);ego.speed=state(4);ego.lateralVelocity=state(5);ego.yawRate=state(6);
            [position,heading]=laneGeometry.referencePose(value(4),0,road.referenceCurve);
            normal=sign(value(1))*[-sin(heading);cos(heading)];
            target.targetPositionInertial=position-value(5)*normal;
            target.targetVelocityInertial=value(3)*normal;
            target.targetHeadingInertial=heading+sign(value(1))*pi/2;
            fixture=struct('ego',ego,'target',target,'road',road,'cfg',cfg,'stored',[]);
            for variant=[1,4]
                fixture.cfg=localConfig(cfg,variants(variant,:));
                [sample,~]=localInvoke(fixture);
                entry=struct('curvature',value(1),'egoSpeed',value(2),'targetSpeed',value(3), ...
                    'targetStation',value(4),'targetOffset',value(5),'variant',names(variant),'sample',sample);
                summary.entries{end+1}=entry;
                fprintf('case %d %s accepted %d\n',index,names(variant),sample.certified);
            end
            localSave(directory,summary);
        end
    else
        for curvature=[0,.01]
            for scenario=["stationary","oncoming","crossing"]
                label=sprintf('kappa-%g-%s',curvature,scenario);
                % Warm fresh admission and continuation for this exact branch.
                for warmup=1:2
                    folder=fullfile(directory,'warmup',label+"-"+warmup);
                    localTrial(scenario,curvature,140,30,folder);
                end
                for repetition=1:2
                    for strict=[false,true]
                        budget=30;mode="diagnostic";
                        if strict,budget=.05;mode="strict";end
                        folder=fullfile(directory,mode,label+"-"+repetition);
                        item=localTrial(scenario,curvature,options.SampleCount,budget,folder);
                        item.repetition=repetition;item.mode=mode;
                        summary.entries{end+1}=item;
                        localSave(directory,summary);
                    end
                end
            end
        end
    end
    localSave(directory,summary);
end

function cfg=localConfig(cfg,parameters)
    cfg=collisionAvoidanceControllerConfig(cfg);
    cfg.admission.temporalShoulderFraction=parameters(1);
    cfg.admission.normalCount=parameters(2);cfg.admission.amplitudeCells=parameters(3);
end

function [sample,decision]=localInvoke(fixture)
    decision=[];timer=tic;
    try
        [~,~,problem]=collisionAvoidanceController(fixture.ego,fixture.target,fixture.road,fixture.cfg,fixture.stored);
        seconds=toc(timer);
    catch exception
        seconds=toc(timer);
        if ~strcmp(exception.identifier,'collisionAvoidanceController:optimizationFailed'),rethrow(exception);end
        sample=struct('seconds',seconds,'certified',false,'message',string(exception.message));return;
    end
    decision=problem.decision;
    fullPlan=false;jointResidual=NaN;
    if isfield(problem.metadata.admissionSearch,'usedFullPlanAdmission')
        fullPlan=problem.metadata.admissionSearch.usedFullPlanAdmission;
        jointResidual=max(problem.metadata.jointCertificateResidual);
    end
    sample=struct('seconds',seconds,'certified',problem.metadata.planCertified, ...
        'phase',problem.metadata.runtime,'horizonSteps',problem.metadata.horizonSteps, ...
        'solverCalls',problem.metadata.solverCallCount, ...
        'hasTarget',problem.metadata.hasTarget,'fullPlanAdmission',fullPlan, ...
        'maximumJointResidual',jointResidual, ...
        'maximumPhysicalResidual',max(problem.program.physicalMatrix*decision-problem.program.physicalBound));
end

function cases=localCases()
    cases=[[-.012;-.01;-.008;.008;.01;.012],repmat([8,4,15,7.5],6,1)];
    for curvature=[-.015,-.005,.005,.015]
        cases(end+1,:)=[curvature,8,4,15,7.5];
    end
    for signValue=[-1,1]
        for speed=[7,9],cases(end+1,:)=[signValue*.01,speed,4,15,7.5];end
        for speed=[3,3.5,4.5,5],cases(end+1,:)=[signValue*.01,8,speed,15,7.5];end
        for station=[13,17],cases(end+1,:)=[signValue*.01,8,4,station,7.5];end
        for offset=[6.5,8.5],cases(end+1,:)=[signValue*.01,8,4,15,offset];end
    end
end

function item=localTrial(scenario,curvature,count,deadline,directory)
    artifact=fullfile(directory,scenario+'-exact-state.mat');
    try
        report=runExactStateRecursiveFeasibilityScenario(Scenario=scenario,RoadCurvature=curvature, ...
            SampleCount=count,DeadlineSeconds=deadline,SearchTimeLimitSeconds=30,OutputDirectory=directory);
    catch exception
        if ~isfile(artifact),rethrow(exception);end
        loaded=load(artifact,'report');report=loaded.report;
    end
    item=struct('scenario',scenario,'curvature',curvature,'budgetSeconds',deadline, ...
        'completed',report.completed,'executedHolds',report.executedHolds,'requestedHolds',count, ...
        'minimumSampledBodyGap',report.minimumSampledBodyGap,'allCertified',all(report.hardCertificateVerified), ...
        'fullPlanAdmissions',sum(report.restorationSolverCallCount), ...
        'firstFrameSeconds',report.runtime.frameSeconds(1),'maximumSeconds',report.runtime.maximumSeconds, ...
        'deadlineMisses',report.runtime.deadlineMisses,'failureMessage',report.failureMessage,'artifact',artifact);
end

function localSave(directory,summary)
    handle=fopen(fullfile(directory,summary.mode+'.json'),'w');assert(handle>=0);
    cleanup=onCleanup(@()fclose(handle));fprintf(handle,'%s\n',jsonencode(summary,PrettyPrint=true));
end
