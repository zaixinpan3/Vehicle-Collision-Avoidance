function summary=runFixedDirectionValidation(directory,options)
%runFixedDirectionValidation Audit fixed-direction trajectory optimization.
% Execute timing modes without concurrent MATLAB workloads. Warmup, failed
% attempts, independent footprint audits and sensitivity calls stay distinct.
    arguments
        directory (1,1) string
        options.Mode (1,1) string {mustBeMember(options.Mode,["replay","campaign","sensitivity"])} = "replay"
        options.FixtureDirectory (1,1) string = "/home/zai/.cache/collisionAvoidance/longest-frame-profile-20260921"
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'), ...
        fullfile(root,'tests'),fullfile(root,'solver','clarabel','matlab'));
    previousThreads=maxNumCompThreads(1);cleanup=onCleanup(@()maxNumCompThreads(previousThreads));
    if ~isfolder(directory),mkdir(directory);end
    summary=struct('matlabVersion',string(version),'mode',options.Mode, ...
        'scope',"Fixed separation directions followed by full hard-constrained trajectory optimization",'entries',{{}});
    if options.Mode=="replay"
        for name=["measured-longest","circular-crossing-longest"]
            loaded=load(fullfile(options.FixtureDirectory,name+'-fixture.mat'),'fixture');fixture=loaded.fixture;
            % Import scenario parameters, not a retired optimizer setting.
            fixture.cfg.jointCertificate=rmfield(fixture.cfg.jointCertificate,'positionScale');
            defaults=collisionAvoidanceControllerConfig();fixture.cfg.admission=defaults.admission;
            fixture.cfg=collisionAvoidanceControllerConfig(fixture.cfg);
            for round=1:2
                for warmup=1:5,localInvoke(fixture);end
                samples=cell(11,1);decision=[];
                for repetition=1:11
                    [sample,problem]=localInvoke(fixture);
                    assert(sample.certified && sample.solverCalls==1 && ~sample.initializerIssued);
                    if repetition==1,decision=problem.decision;end
                    assert(isequaln(problem.decision,decision),'Repeated complete decision changed.');
                    samples{repetition}=sample;
                end
                % Inspect the actual optimization dimensions outside timing.
                program=formulateAvoidanceProblem(problem.model);
                [seed,angles]=solveHardCbfClf.fluidInitialize(program,fixture.cfg);
                conic=avoidanceStageQp.fixedDirections(program,seed,angles,fixture.cfg);
                direction=seed(program.layout.planIndex)-program.anchorPlan;
                displacement=problem.decision(program.layout.planIndex)-program.anchorPlan;
                departure=norm(displacement-direction*((direction.'*displacement)/(direction.'*direction)));
                angleError=max(abs(angles-problem.program.jointCertificate.angles));
                assert(angleError==0 && departure>1e-4);
                entry=struct('fixture',name,'round',round,'samples',{samples}, ...
                    'variables',numel(conic.q),'constraints',size(conic.A,1), ...
                    'coneSizes',conic.cones,'controlLineDepartureNorm',departure, ...
                    'fixedAnglesMaximumError',angleError, ...
                    'seedObjective',.5*seed.'*program.P*seed+program.q.'*seed, ...
                    'optimizedObjective',.5*decision.'*program.P*decision+program.q.'*decision);
                summary.entries{end+1}=entry;localSave(directory,summary);
                fprintf('%s round %d median/max %.3f/%.3f ms; control-line departure %.6g\n',name,round, ...
                    1000*median(cellfun(@(x)x.seconds,samples)),1000*max(cellfun(@(x)x.seconds,samples)),departure);
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
            [sample,~]=localInvoke(fixture);
            summary.entries{end+1}=struct('curvature',value(1),'egoSpeed',value(2), ...
                'targetSpeed',value(3),'targetStation',value(4),'targetOffset',value(5),'sample',sample);
            localSave(directory,summary);fprintf('case %d accepted %d\n',index,sample.certified);
        end
    else
        for curvature=[0,.01]
            for scenario=["stationary","oncoming","crossing"]
                label=sprintf('kappa-%g-%s',curvature,scenario);
                for warmup=1:2
                    localTrial(scenario,curvature,140,30,fullfile(directory,'warmup',label+"-"+warmup));
                end
                for repetition=1:2
                    for strict=[false,true]
                        budget=30;mode="diagnostic";
                        if strict,budget=.05;mode="strict";end
                        item=localTrial(scenario,curvature,600,budget,fullfile(directory,mode,label+"-"+repetition));
                        item.repetition=repetition;item.mode=mode;
                        summary.entries{end+1}=item;localSave(directory,summary);
                    end
                end
            end
        end
    end
    localSave(directory,summary);
end

function [sample,problem]=localInvoke(fixture)
    problem=struct();timer=tic;
    try
        [~,~,problem]=collisionAvoidanceController(fixture.ego,fixture.target,fixture.road,fixture.cfg,fixture.stored);
        seconds=toc(timer);
    catch exception
        seconds=toc(timer);
        if ~strcmp(exception.identifier,'collisionAvoidanceController:optimizationFailed'),rethrow(exception);end
        sample=struct('seconds',seconds,'certified',false,'message',string(exception.message));return;
    end
    search=problem.metadata.admissionSearch;
    fixedAngles=true;issued=false;jointResidual=NaN;
    if isfield(search,'fixedCertificateAngles')
        fixedAngles=isequal(search.fixedCertificateAngles,problem.program.jointCertificate.angles);
        issued=search.issuedAdmissionWitness;jointResidual=max(problem.metadata.jointCertificateResidual);
    end
    sample=struct('seconds',seconds,'certified',problem.metadata.planCertified, ...
        'phase',problem.metadata.runtime,'horizonSteps',problem.metadata.horizonSteps, ...
        'solverCalls',problem.metadata.solverCallCount,'hasTarget',problem.metadata.hasTarget, ...
        'initializerIssued',issued,'directionsUnchanged',fixedAngles, ...
        'maximumJointResidual',jointResidual, ...
        'maximumPhysicalResidual',max(problem.program.physicalMatrix*problem.decision-problem.program.physicalBound));
end

function cases=localCases()
    cases=zeros(30,5);cases(1:10,:)=[[-.012;-.01;-.008;.008;.01;.012;-.015;-.005;.005;.015],repmat([8,4,15,7.5],10,1)];
    cursor=10;
    for signValue=[-1,1]
        variation=[7,4,15,7.5;9,4,15,7.5;8,3,15,7.5;8,3.5,15,7.5; ...
            8,4.5,15,7.5;8,5,15,7.5;8,4,13,7.5;8,4,17,7.5;8,4,15,6.5;8,4,15,8.5];
        cases(cursor+(1:10),:)=[repmat(signValue*.01,10,1),variation];cursor=cursor+10;
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
        'firstFrameSeconds',report.runtime.frameSeconds(1),'maximumSeconds',report.runtime.maximumSeconds, ...
        'deadlineMisses',report.runtime.deadlineMisses,'failureMessage',report.failureMessage,'artifact',artifact);
end

function localSave(directory,summary)
    handle=fopen(fullfile(directory,summary.mode+'.json'),'w');assert(handle>=0);
    cleanup=onCleanup(@()fclose(handle));fprintf(handle,'%s\n',jsonencode(summary,PrettyPrint=true));
end
