function summary = profileNonlinearControllerRuntime(outputDirectory, sourceDirectory)
%profileNonlinearControllerRuntime Diagnose saved nonlinear controller frames.
% Replays fixed measured states without running the vehicle plant. Clean
% timing and profiler measurements are separate. The profiled solver alone
% receives a larger diagnostic budget, and its result must match clean replay.
    arguments
        outputDirectory (1,1) string
        sourceDirectory (1,1) string
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'), ...
        fullfile(root,'solver','bicycle'));
    oldThreads = maxNumCompThreads(1);
    cleanup = onCleanup(@() maxNumCompThreads(oldThreads));
    if ~isfolder(outputDirectory), mkdir(outputDirectory); end
    summary = struct('matlabVersion',version,'computationalThreads',1, ...
        'scope','Fixed measured-state diagnostic replay; no plant simulation or timing deadline guarantee', ...
        'fixtures',{{}});
    names = ["straight","circular","sCurve"];
    files = ["nonlinear/straight_oncoming.mat", ...
        "nonlinear/circular/circular_oncoming.mat", ...
        "nonlinear/scurve/varying_curvature_oncoming.mat"];
    for caseIndex = 1:numel(names)
        loaded = load(fullfile(sourceDirectory,files(caseIndex)),'result');
        recorded = loaded.result;
        cfg = recorded.controllerConfiguration;
        first = localFrame(recorded,1,cfg,[]);
        [firstCommand,~,~,previous] = localInvoke(first);
        initialInputDifference = max(abs(firstCommand.actuatorInput-recorded.command{1}.actuatorInput));
        fixture = localFrame(recorded,2,cfg,previous);
        fixture.ego.heldActuatorInput = previous.appliedInput;
        definitions = {struct('name',names(caseIndex)+"-continuation", ...
            'fixture',fixture,'initialInputDifference',initialInputDifference)};
        if caseIndex ~= 2
            visible = find(recorded.attempts.targetVisible,1);
            definitions{end+1} = struct('name',names(caseIndex)+"-fresh-visible", ...
                'fixture',localFrame(recorded,visible,cfg,[]), ...
                'initialInputDifference',NaN); %#ok<AGROW>
        end
        for definitionIndex = 1:numel(definitions)
            definition = definitions{definitionIndex};
            fixture = definition.fixture;
            fprintf('START %s at t=%.2f\n',definition.name,fixture.ego.stateTime);
            localInvoke(fixture);
            timer = tic;
            [command,~,problem] = localInvoke(fixture);
            publicSeconds = toc(timer);
            model = problem.model;
            options = rmfield(cfg.nonlinear,'horizonSeconds');
            samples = zeros(1,3);
            decisions = cell(1,3);
            for repetition = 1:3
                timer = tic;
                result = solveNonlinearAvoidancePlan(model,options);
                samples(repetition) = toc(timer);
                assert(result.feasible,'The fixed diagnostic fixture must admit a nonlinear plan.');
                decisions{repetition} = result.plan;
                assert(max(abs(result.plan(:,1)-command.actuatorInput))<1e-8, ...
                    'Public and direct solver decisions differ.');
                assert(max(abs(result.plan-decisions{1}),[],'all')<1e-8, ...
                    'Repeated clean solver decisions differ.');
            end
            cleanResult = result;
            profileOptions = options;
            profileOptions.timeLimitSeconds = 300;
            profile clear;
            profile on;
            profiledResult = solveNonlinearAvoidancePlan(model,profileOptions);
            profile off;
            information = profile('info');
            difference = max(abs(profiledResult.plan-cleanResult.plan),[],'all');
            assert(profiledResult.feasible && difference<1e-8, ...
                'Profiling changed the accepted plan; do not compare different work.');
            entry = struct('name',definition.name,'time',fixture.ego.stateTime, ...
                'sourceFile',fullfile(sourceDirectory,files(caseIndex)), ...
                'sourceFrame',fixture.sourceFrame,'hasPrevious',~isempty(fixture.previous), ...
                'initialInputDifference',definition.initialInputDifference, ...
                'publicSeconds',publicSeconds,'cleanSolverSeconds',samples, ...
                'profiledPlanDifference',difference,'profileBudgetSeconds',300, ...
                'diagnostics',cleanResult.diagnostics, ...
                'horizonHolds',model.horizonSteps, ...
                'auditNodes',model.horizonSteps*options.auditSubsteps+1, ...
                'decisionVariables',2*ceil(model.horizonSteps/options.blockSteps), ...
                'functions',{localFunctions(information)});
            summary.fixtures{end+1} = entry;
            save(fullfile(outputDirectory,definition.name+".mat"), ...
                'fixture','model','options','cleanResult','information','profiledResult');
            localWrite(fullfile(outputDirectory,'profile-summary.json'),summary);
            fprintf('%s clean median %.6f s, public %.6f s, profile difference %.3g\n', ...
                definition.name,median(samples),publicSeconds,difference);
        end
    end
end

function fixture = localFrame(recorded,index,cfg,previous)
    ego = recorded.attempts.controllerEgoEstimate{index};
    if index>1, ego.heldActuatorInput = recorded.command{index-1}.actuatorInput; end
    road = recorded.attempts.roadPerception{index}.roadGeometry;
    if isfield(recorded.scenario.geometry,'referenceCurve')
        road.referenceCurve = recorded.scenario.geometry.referenceCurve;
    end
    fixture = struct('ego',ego,'target',recorded.attempts.targetEstimate{index}, ...
        'road',road,'configuration',cfg,'previous',previous,'sourceFrame',index);
end

function [command,inputs,problem,stored] = localInvoke(fixture)
    [command,inputs,problem,stored] = collisionAvoidanceController( ...
        fixture.ego,fixture.target,fixture.road,fixture.configuration,fixture.previous);
end

function functions = localFunctions(information)
    table = information.FunctionTable;
    functions = cell(numel(table),1);
    for index = 1:numel(table)
        children = 0;
        if ~isempty(table(index).Children), children = sum([table(index).Children.TotalTime]); end
        functions{index} = struct('name',table(index).FunctionName, ...
            'file',table(index).FileName,'calls',table(index).NumCalls, ...
            'totalSeconds',table(index).TotalTime, ...
            'selfSeconds',table(index).TotalTime-children, ...
            'executedLines',table(index).ExecutedLines);
    end
end

function localWrite(path,value)
    file = fopen(path,'w');
    assert(file>=0,'Unable to open diagnostic summary.');
    cleanup = onCleanup(@() fclose(file));
    fprintf(file,'%s\n',jsonencode(value,PrettyPrint=true));
end
