function summary = profileFiniteBranchRuntime(outputDirectory, options)
%profileFiniteBranchRuntime Replay current controller frames and attribute cost.
% Production algorithms and tolerances are unchanged. Profiled timings are
% attribution data only; unprofiled replays are the timing measurements.
    arguments
        outputDirectory (1,1) string
        options.AdmissionRepetitions (1,1) double {mustBeInteger,mustBePositive} = 3
        options.RegularRepetitions (1,1) double {mustBeInteger,mustBePositive} = 15
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    if ~isfolder(outputDirectory), mkdir(outputDirectory); end
    assert(strcmp(profile('status').ProfilerStatus,'off'), ...
        'Stop an existing profiler before starting this isolated study.');
    cleanup = onCleanup(@() profile('off'));
    cfg = collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'controller',struct('sampleTime',.1),'model',struct('lateralDomainRadius',4), ...
        'solver',struct('frameDeadlineSeconds',120)));
    ego = struct('position',[0;0],'yaw',0,'speed',8,'stateTime',0, ...
        'perception',struct('time',0,'range',16,'completeWithinRange',true));
    fixture = struct('ego',ego,'target',[],'road',[-100,0;2000,0], ...
        'cfg',cfg,'previous',[]);
    fixtures = {fixture}; names = "cruiseFresh";
    cruise = fixture;
    for index = 1:26
        [~,~,problem,state] = localCall(cruise);
        cruise = localNext(cruise,problem,state);
    end
    fixtures{end+1} = cruise; names(end+1) = "cruiseSuccessor";
    oncoming = cruise; oncoming.target = localTarget([39.2;0],[-8;0],pi);
    fixtures{end+1} = oncoming; names(end+1) = "oncomingAdmission";
    stationary = fixture; stationary.target = localTarget([15;0],[0;0],0);
    fixtures{end+1} = stationary; names(end+1) = "stationaryAdmission";
    summary = struct('matlabVersion',string(version),'threads',maxNumCompThreads, ...
        'sampleTime',.1,'scope',"Exact-state frozen frame replay; no production change or deadline guarantee", ...
        'cases',struct());
    index = 1;
    while index <= numel(fixtures)
        current = fixtures{index}; name = names(index);
        count = options.RegularRepetitions;
        if contains(name,"Admission"), count = options.AdmissionRepetitions; end
        seconds = zeros(count,1); phases = zeros(count,4); native = zeros(count,1);
        for repeat = 1:count
            timer = tic; [~,~,problem,state] = localCall(current); seconds(repeat) = toc(timer);
            runtime = problem.metadata.runtime;
            phases(repeat,:) = [runtime.inputPreparationSeconds,runtime.formulationSeconds, ...
                runtime.solveSeconds,seconds(repeat)-sum(struct2array(runtime))];
            native(repeat) = problem.metadata.branchSearch.integerSeconds;
            assert(problem.metadata.postSolveCertificationPerformed && ~problem.metadata.fallbackUsed);
        end
        if contains(name,"Admission")
            fixtures{end+1} = localNext(current,problem,state); %#ok<AGROW>
            names(end+1) = replace(name,"Admission","Successor"); %#ok<AGROW>
        end
        result = struct('seconds',seconds,'phaseSeconds',phases, ...
            'phaseNames',["preparation","formulation","solve","verificationAndOutput"], ...
            'integerSeconds',native,'medianSeconds',median(seconds),'maximumSeconds',max(seconds), ...
            'variables',numel(problem.program.q),'rows',size(problem.program.A,1), ...
            'nonzeros',nnz(problem.program.A),'cells',numel(problem.prediction.cells), ...
            'branchSearch',problem.metadata.branchSearch);
        baseline = problem.decision;
        profile clear; profile on;
        [~,~,profiled] = localCall(current);
        profile off; profileData = profile('info');
        rows = profileData.FunctionTable;
        total = [rows.TotalTime].'; self = total;
        for item = 1:numel(rows), self(item) = total(item)-sum([rows(item).Children.TotalTime]); end
        profileTable = table(string({rows.FunctionName}).', [rows.NumCalls].', total, self, ...
            VariableNames={'functionName','calls','totalSeconds','selfSeconds'});
        profileTable = sortrows(profileTable,'selfSeconds','descend');
        writetable(profileTable,fullfile(outputDirectory,name+'-profile.csv'));
        assert(norm(profiled.decision-baseline,inf)<1e-7);
        programs = {}; solverCalls = struct([]);
        captured = current; captured.cfg.solver.jointFunction = @capture;
        [~,~,inspected] = localCall(captured);
        assert(norm(inspected.decision-baseline,inf)<1e-7);
        result.solverCalls = solverCalls;
        summary.cases.(name) = result;
        save(fullfile(outputDirectory,name+'.mat'),'current','result','profileData','programs','baseline');
        localSaveSummary(outputDirectory,summary);
        fprintf('%s median %.3f ms; phases [%.3f %.3f %.3f %.3f] ms; %d vars/%d rows\n', ...
            name,1e3*median(seconds),1e3*median(phases,1),result.variables,result.rows);
        index = index+1;
    end
    function solve = capture(~,program)
        timer = tic; solve = program.defaultSolver(); elapsed = toc(timer);
        record = struct('seconds',elapsed,'variables',numel(program.q),'rows',size(program.A,1), ...
            'nonzeros',nnz(program.A),'iterations',solve.output.iterations, ...
            'nativeSeconds',solve.output.solveTime,'status',solve.output.status);
        programs{end+1} = rmfield(program,'defaultSolver');
        solverCalls = [solverCalls;record];
    end
end

function [command,inputs,problem,state] = localCall(fixture)
    [command,inputs,problem,state] = collisionAvoidanceController( ...
        fixture.ego,fixture.target,fixture.road,fixture.cfg,fixture.previous);
end

function next = localNext(fixture,problem,state)
    next = fixture; x = problem.predictedState(:,2);
    [next.ego.position,next.ego.yaw] = laneGeometry.fromFrenet(x,problem.model.lane);
    next.ego.speed = x(4); next.ego.lateralVelocity = x(5); next.ego.yawRate = x(6);
    next.ego.stateTime = fixture.ego.stateTime+.1;
    next.ego.perception.time = next.ego.stateTime;
    next.ego.heldActuatorInput = state.appliedInput; next.previous = state;
    if ~isempty(next.target)
        next.target.targetPositionInertial = next.target.targetPositionInertial ...
            +.1*next.target.targetVelocityInertial;
    end
end

function target = localTarget(position,velocity,heading)
    target = struct('trackId',1,'targetPositionInertial',position, ...
        'targetVelocityInertial',velocity,'targetAccelerationInertial',[0;0], ...
        'targetHeadingInertial',heading,'targetYawRate',0, ...
        'predictionMotion',struct('kind',"finite-sensing-motion-v1", ...
        'jerkBound',[0;0],'yawAccelerationBound',0));
end

function localSaveSummary(folder,summary)
    save(fullfile(folder,'summary.mat'),'summary');
    file = fopen(fullfile(folder,'summary.json'),'w'); assert(file>=0);
    cleanup = onCleanup(@() fclose(file));
    fprintf(file,'%s\n',jsonencode(summary,PrettyPrint=true));
end
