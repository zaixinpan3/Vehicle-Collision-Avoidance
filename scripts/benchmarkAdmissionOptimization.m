function summary=benchmarkAdmissionOptimization(directory,baselineDirectory,fixtureDirectory,options)
%benchmarkAdmissionOptimization Alternate warmed baseline and production blocks.
% Run in a fresh MATLAB -singleCompThread process without concurrent workloads.
% BaselineDirectory contains the three original MATLAB source files; exact
% decisions and final safety programs are compared outside each timed call.
    arguments
        directory (1,1) string
        baselineDirectory (1,1) string
        fixtureDirectory (1,1) string
        options.Rounds (1,1) double {mustBeInteger,mustBePositive} = 4
        options.Repetitions (1,1) double {mustBeInteger,mustBePositive} = 15
        options.Warmups (1,1) double {mustBeInteger,mustBePositive} = 5
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'), ...
        fullfile(root,'scripts'),fullfile(root,'solver','clarabel','matlab'));
    assert(maxNumCompThreads==1,'Use matlab -singleCompThread for timing.');
    assert(isfolder(baselineDirectory));
    profile off;
    if ~isfolder(directory),mkdir(directory);end
    cleanup=onCleanup(@()localRestore(baselineDirectory));
    replay=jsondecode(fileread(fullfile(fixtureDirectory,'replay.json')));
    summary=struct('scope',"Alternating baseline/optimized first admissions; warmup excluded", ...
        'matlabVersion',string(version),'baselineDirectory',baselineDirectory, ...
        'fixtureDirectory',fixtureDirectory,'rounds',options.Rounds, ...
        'repetitions',options.Repetitions,'warmupsPerBlock',options.Warmups,'blocks',{{}});
    for index=1:numel(replay.fixtures)
        entry=replay.fixtures(index);loaded=load(entry.file,'fixture');fixture=loaded.fixture;
        localActivate(baselineDirectory,root,true);
        [~,expected]=localInvoke(fixture);
        assert(isequaln(expected.decision,fixture.expectedDecision),'Saved baseline changed.');
        for round=1:options.Rounds
            order=[true,false];
            if mod(round,2)==0,order=fliplr(order);end
            for baseline=order
                localActivate(baselineDirectory,root,baseline);
                for warmup=1:options.Warmups,localInvoke(fixture);end
                samples=cell(options.Repetitions,1);
                for repetition=1:options.Repetitions
                    [seconds,actual]=localInvoke(fixture);
                    localVerify(actual,expected);
                    samples{repetition}=struct('seconds',seconds,'phase',actual.metadata.runtime);
                end
                variant="optimized";
                if baseline,variant="baseline";end
                summary.blocks{end+1}=struct('fixture',entry.definition.name,'round',round, ...
                    'variant',variant,'samples',{samples},'completeDecisionEqual',true, ...
                    'finalSafetyProgramEqual',true,'certified',true,'nativeSolves',1);
                localWrite(fullfile(directory,'paired-admission.json'),summary);
                fprintf('%s round %d %s median/max %.3f/%.3f ms\n', ...
                    entry.definition.name,round,variant, ...
                    1000*median(cellfun(@(x)x.seconds,samples)),1000*max(cellfun(@(x)x.seconds,samples)));
            end
        end
    end
end

function localActivate(directory,root,baseline)
    localRestore(directory);
    if baseline,addpath(directory,'-begin');end
    clear avoidanceSafetyGeometry formulateAvoidanceProblem solveHardCbfClf
    expected=fullfile(root,'controller','avoidanceSafetyGeometry.m');
    if baseline,expected=fullfile(directory,'avoidanceSafetyGeometry.m');end
    assert(strcmp(which('avoidanceSafetyGeometry'),expected),'Unexpected controller source.');
end

function localRestore(directory)
    if any(string(strsplit(path,pathsep))==directory),rmpath(directory);end
    clear avoidanceSafetyGeometry formulateAvoidanceProblem solveHardCbfClf
end

function [seconds,problem]=localInvoke(fixture)
    timer=tic;
    [~,~,problem]=collisionAvoidanceController(fixture.ego,fixture.target,fixture.road,fixture.cfg,fixture.stored);
    seconds=toc(timer);
end

function localVerify(actual,expected)
    assert(isequaln(actual.decision,expected.decision),'Complete decision changed.');
    assert(isequaln(actual.inputPlan,expected.inputPlan),'Control plan changed.');
    assert(isequaln(actual.predictedState,expected.predictedState),'State prediction changed.');
    assert(actual.metadata.planCertified && actual.metadata.admissionSearch.nativeSolves==1);
    fields=["A","b","P","q","cones","physicalMatrix","physicalBound", ...
        "safetyBound","physicalLabels","jointCertificate","completion","geometry"];
    for field=fields
        assert(isequaln(actual.program.(field),expected.program.(field)), ...
            'Final safety program field %s changed.',field);
    end
end

function localWrite(file,value)
    handle=fopen(file,'w');assert(handle>=0);cleanup=onCleanup(@()fclose(handle));
    fprintf(handle,'%s\n',jsonencode(value,PrettyPrint=true));
end
