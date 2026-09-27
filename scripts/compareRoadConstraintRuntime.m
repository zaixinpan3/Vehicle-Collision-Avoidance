function summary = compareRoadConstraintRuntime(profileDirectory)
%compareRoadConstraintRuntime Isolate road-row cost on frozen shooting inputs.
% Only boundaries and lateralClearance are removed from saved solver models.
% Three clean repetitions follow one warm-up per variant. No plant is run.
    arguments
        profileDirectory (1,1) string
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'), ...
        fullfile(root,'solver','bicycle'));
    previousThreads = maxNumCompThreads(1);
    cleanup = onCleanup(@() maxNumCompThreads(previousThreads));
    names = ["straight-continuation","straight-fresh-visible", ...
        "circular-continuation","sCurve-continuation","sCurve-fresh-visible"];
    summary = struct('matlabVersion',version,'computationalThreads',1,'fixtures',{{}});
    for name = names
        loaded = load(fullfile(profileDirectory,name+".mat"),'model','options','cleanResult');
        variants = cell(1,2);
        for variant = 1:2
            model = loaded.model;
            if variant == 2
                model.road.boundaries = model.road.boundaries([]);
                model.road.lateralClearance = zeros(2,0);
            end
            solveNonlinearAvoidancePlan(model,loaded.options);
            seconds = zeros(1,3);
            for repetition = 1:3
                timer = tic;
                result = solveNonlinearAvoidancePlan(model,loaded.options);
                seconds(repetition) = toc(timer);
                assert(result.feasible,'The diagnostic plan was rejected.');
                if variant == 1
                    assert(max(abs(result.plan-loaded.cleanResult.plan),[],'all')<1e-8, ...
                        'Road-on replay differs from its frozen baseline.');
                else
                    assert(result.diagnostics.roadRowCount==0,'Road rows remain enabled.');
                end
            end
            variants{variant} = struct('roadEnabled',variant==1,'seconds',seconds, ...
                'medianSeconds',median(seconds),'diagnostics',result.diagnostics, ...
                'firstInput',result.plan(:,1),'plan',result.plan);
        end
        entry = struct('name',name,'roadOn',variants{1},'roadOff',variants{2}, ...
            'speedup',variants{1}.medianSeconds/variants{2}.medianSeconds, ...
            'maximumPlanDifference',max(abs(variants{1}.plan-variants{2}.plan),[],'all'));
        summary.fixtures{end+1} = entry;
        file = fopen(fullfile(profileDirectory,'road-ablation-timing.json'),'w');
        assert(file>=0,'Unable to save timing results.');
        fileCleanup = onCleanup(@() fclose(file));
        fprintf(file,'%s\n',jsonencode(summary,PrettyPrint=true));
        clear fileCleanup;
        fprintf('%s: %.4f -> %.4f s, %.2fx, plan difference %.3g\n', ...
            name,entry.roadOn.medianSeconds,entry.roadOff.medianSeconds, ...
            entry.speedup,entry.maximumPlanDifference);
    end
end
