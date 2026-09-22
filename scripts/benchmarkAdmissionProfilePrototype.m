function summary=benchmarkAdmissionProfilePrototype(directory,variant,output,options)
%benchmarkAdmissionProfilePrototype Measure external admission-only experiments.
% Invoke in a fresh MATLAB -singleCompThread process per variant. The original
% controller is evaluated first; only selected class files are then shadowed.
% Every measured full decision and final safety problem must remain identical.
    arguments
        directory (1,1) string
        variant (1,1) string
        output (1,1) string
        options.Repetitions (1,1) double {mustBeInteger,mustBePositive} = 21
        options.Warmups (1,1) double {mustBeInteger,mustBePositive} = 5
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'), ...
        fullfile(root,'scripts'),fullfile(root,'solver','clarabel','matlab'));
    profile off;
    replay=jsondecode(fileread(fullfile(directory,'replay.json')));
    fixtures=cell(numel(replay.fixtures),1);baseline=fixtures;
    for index=1:numel(fixtures)
        loaded=load(replay.fixtures(index).file,'fixture');fixtures{index}=loaded.fixture;
        [~,baseline{index}]=localInvoke(fixtures{index});
        assert(isequaln(baseline{index}.decision,fixtures{index}.expectedDecision));
    end
    prototype=fullfile(directory,'prototypes',variant);
    assert(isfolder(prototype));
    addpath(prototype,'-begin');
    cleanup=onCleanup(@()rmpath(prototype));
    clear avoidanceSafetyGeometry solveHardCbfClf
    summary=struct('scope',"External fresh-admission experiment; no production changes", ...
        'variant',variant,'matlabVersion',string(version),'threads',maxNumCompThreads, ...
        'warmupsPerFixture',options.Warmups,'repetitionsPerFixture',options.Repetitions,'fixtures',{{}});
    for index=1:numel(fixtures)
        fixture=fixtures{index};expected=baseline{index};
        for warmup=1:options.Warmups,localInvoke(fixture);end
        samples=zeros(options.Repetitions,1);
        for repetition=1:options.Repetitions
            [samples(repetition),actual]=localInvoke(fixture);
            assert(isequaln(actual.decision,expected.decision),'Full decision changed.');
            assert(isequaln(actual.inputPlan,expected.inputPlan),'Control plan changed.');
            assert(isequaln(actual.predictedState,expected.predictedState),'Predicted state changed.');
            assert(actual.metadata.planCertified,'Independent certification failed.');
            fields=["A","b","P","q","cones","physicalMatrix","physicalBound", ...
                "safetyBound","physicalLabels","jointCertificate","completion","geometry"];
            for field=fields
                assert(isequaln(actual.program.(field),expected.program.(field)), ...
                    'Final program field %s changed.',field);
            end
        end
        entry=struct('name',replay.fixtures(index).definition.name,'seconds',samples, ...
            'completeDecisionEqual',true,'safetyProgramEqual',true, ...
            'certified',true,'nativeSolves',actual.metadata.admissionSearch.nativeSolves);
        summary.fixtures{end+1}=entry;
        fprintf('%s %s median/max %.3f/%.3f ms\n',variant,entry.name, ...
            1000*median(samples),1000*max(samples));
    end
    handle=fopen(output,'w');assert(handle>=0);close=onCleanup(@()fclose(handle));
    fprintf(handle,'%s\n',jsonencode(summary,PrettyPrint=true));
end

function [seconds,problem]=localInvoke(fixture)
    timer=tic;
    [~,~,problem]=collisionAvoidanceController(fixture.ego,fixture.target,fixture.road,fixture.cfg,fixture.stored);
    seconds=toc(timer);
end
