function summary = benchmarkFiniteBranchRowReduction(inputDirectory, options)
%benchmarkFiniteBranchRowReduction Test exact duplicate-row removal offline.
% Equal left-hand sides are replaced by their tightest bound; every cone,
% objective and original physical certificate remains unchanged. A selected
% branch is already known in these replays, so they exclude admission search.
    arguments
        inputDirectory (1,1) string
        options.Repetitions (1,1) double {mustBeInteger,mustBePositive} = 5
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    names = ["cruiseSuccessor","oncomingAdmission","stationaryAdmission", ...
        "oncomingSuccessor","stationarySuccessor"];
    summary = struct();
    for name = names
        saved = load(fullfile(inputDirectory,name+'.mat'),'programs','current');
        original = saved.programs{end}; cfg = saved.current.cfg;
        cfg.solver.frameDeadlineSeconds = Inf;
        count = original.cones(2); assert(original.cones(1)==0);
        timer = tic;
        [rows,~,group] = unique(full(original.A(1:count,:)),'rows');
        limits = accumarray(group,original.b(1:count),[],@min);
        reduced = original;
        reduced.A = [sparse(rows);original.A(count+1:end,:)];
        reduced.b = [limits;original.b(count+1:end)];
        reduced.cones(2) = numel(limits);
        reductionSeconds = toc(timer);
        assert(isequal(rows(group,:),full(original.A(1:count,:))));
        assert(all(limits(group)<=original.b(1:count)));
        seconds = zeros(options.Repetitions,2); native = seconds;
        differences = zeros(options.Repetitions,1); margins = differences;
        objectives = zeros(options.Repetitions,2);
        for repeat = 1:options.Repetitions
            timer = tic; baseline = solveHardCbfClf.constrained(original,cfg);
            seconds(repeat,1) = toc(timer); native(repeat,1) = baseline.output.solveTime;
            timer = tic; trial = solveHardCbfClf.constrained(reduced,cfg);
            seconds(repeat,2) = toc(timer); native(repeat,2) = trial.output.solveTime;
            assert(baseline.feasible && trial.feasible);
            solveHardCbfClf.certify(original,baseline.decision);
            solveHardCbfClf.certify(original,trial.decision);
            differences(repeat) = norm(trial.decision-baseline.decision,inf);
            margins(repeat) = min(original.physicalBound-original.physicalMatrix*trial.decision);
            decisions = [baseline.decision,trial.decision];
            objectives(repeat,:) = .5*sum(decisions.*(original.P*decisions),1)+original.q.'*decisions;
        end
        result = struct('originalRows',size(original.A,1),'reducedRows',size(reduced.A,1), ...
            'reductionSeconds',reductionSeconds,'seconds',seconds,'nativeSeconds',native, ...
            'decisionDifferences',differences,'physicalMargins',margins,'objectives',objectives);
        summary.(name) = result;
        fprintf('%s rows %d -> %d; solve %.3f -> %.3f ms; preprocessing %.3f ms\n', ...
            name,result.originalRows,result.reducedRows,1e3*median(seconds,1),1e3*reductionSeconds);
        save(fullfile(inputDirectory,'row-reduction.mat'),'summary');
        file = fopen(fullfile(inputDirectory,'row-reduction.json'),'w'); assert(file>=0);
        cleanup = onCleanup(@() fclose(file));
        fprintf(file,'%s\n',jsonencode(summary,PrettyPrint=true)); clear cleanup;
    end
end
