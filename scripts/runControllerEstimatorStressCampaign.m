function summary = runControllerEstimatorStressCampaign(options)
%runControllerEstimatorStressCampaign Reproducible algorithm stress experiments.
% Each controller case uses the declared affine plant and audits 21 points
% per hold. Failures are retained, never converted into passing cases. The
% estimator trials use identical seeds for comparisons. Nonlinear-plant
% validation and regression fault injection are separate from these tables.
    arguments
        options.OutputDirectory (1,1) string
        options.Groups (1,:) string = ["operating","estimator","integration"]
        options.Seed (1,1) double {mustBeInteger,mustBeNonnegative} = 20260925
        options.MonteCarloRuns (1,1) double {mustBeInteger,mustBePositive} = 8
        options.Duration (1,1) double {mustBeFinite,mustBePositive} = 12
        options.CaseIndices (1,:) double {mustBeInteger,mustBePositive} = []
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'scripts'),fullfile(root,'config'),fullfile(root,'controller'),fullfile(root,'estimator'));
    mustBeMember(options.Groups,["declared","operating","estimator","integration"]);
    if ~isfolder(options.OutputDirectory),mkdir(options.OutputDirectory);end
    summary = struct('options',options,'matlabVersion',string(version));
    if ismember("declared",options.Groups)
        summary.declared = runDeclaredPlantFailureSweep( ...
            OutputDirectory=fullfile(options.OutputDirectory,'declared'));
    end
    if ismember("operating",options.Groups)
        summary.operating = localOperating(options);
    end
    if ismember("estimator",options.Groups)
        benchmark = runOnlineNrmmTrackingErrorBenchmark(Seed=options.Seed, ...
            MonteCarloRuns=options.MonteCarloRuns,Duration=options.Duration);
        summary.estimator = benchmark.table;
        writetable(benchmark.table,fullfile(options.OutputDirectory,'estimator.csv'));
        save(fullfile(options.OutputDirectory,'estimator.mat'),'benchmark');
    end
    if ismember("integration",options.Groups)
        summary.integration = localIntegration(options);
    end
    save(fullfile(options.OutputDirectory,'stressCampaign.mat'),'summary');
end

function results = localOperating(options)
    cases = {};
    for scenario = ["stationary","oncoming","crossing","cruise"]
        for curvature = [0,.01]
            for speed = [2,5,12,18]
                cases{end+1} = localCase("speed",scenario,curvature,{"ReferenceSpeed",speed}); %#ok<AGROW>
            end
            for friction = [.2,.4,1.0]
                cases{end+1} = localCase("friction",scenario,curvature, ...
                    {"FrictionCoefficient",[friction;friction]}); %#ok<AGROW>
            end
            for h = [.02,.1,.2]
                cases{end+1} = localCase("samplePeriod",scenario,curvature,{"SampleTime",h}); %#ok<AGROW>
            end
            for rate = [.3,.7]
                cases{end+1} = localCase("slew",scenario,curvature, ...
                    {"SteeringRateMaximum",rate,"BrakingRatioRateMaximum",2}); %#ok<AGROW>
            end
        end
    end
    egoBound = [.01;.01;.001;.01;.01;.001];
    targetBound = [.1;.1;.05;.05;.01;.01;.01;.01];
    for scenario = ["stationary","oncoming","crossing"]
        for seed = options.Seed+(0:options.MonteCarloRuns-1)
            cases{end+1} = localCase("noiseSeed",scenario,.01, ...
                {"Seed",seed,"EgoErrorBound",3*egoBound,"TargetErrorBound",3*targetBound}); %#ok<AGROW>
        end
    end
    columns = {'caseIndex','group','scenario','curvature','parameters','completed','passed', ...
        'executedHolds','failureIdentifier','failureMessage','minimumNodeGapM','minimumSampledGapM', ...
        'minimumRoadMarginM','minimumDomainMargin','maximumClfResidual','maximumSlewViolation', ...
        'maximumFrameMs','deadlineMisses','minimumSpeedMps','maximumAbsoluteHeadingRad'};
    rows = cell(0,numel(columns));
    indices = options.CaseIndices;
    if isempty(indices),indices = 1:numel(cases);end
    assert(all(indices<=numel(cases)),'runControllerEstimatorStressCampaign:invalidCase','Case index exceeds the campaign.');
    for index = indices
        c = cases{index};
        directory = fullfile(options.OutputDirectory,sprintf('operating%03d',index));
        h = .05;
        location = find(strcmp(c.arguments,'SampleTime'),1);
        if ~isempty(location),h = c.arguments{location+1};end
        report = struct();
        try
            report = runExactStateRecursiveFeasibilityScenario('Scenario',c.scenario, ...
                'RoadCurvature',c.curvature,'SampleCount',round(options.Duration/h), ...
                'DeadlineSeconds',Inf,'SearchTimeLimitSeconds',30, ...
                'AuditSubsteps',20,'OutputDirectory',directory,'RethrowFailure',false,c.arguments{:});
            identifier = report.failureIdentifier;message = report.failureMessage;
        catch exception
            identifier = string(exception.identifier);message = string(exception.message);
        end
        runtime = localField(report,'runtime',struct());
        states = localField(report,'state',nan(6,1));
        rows(end+1,:) = {index,c.group,c.scenario,c.curvature,string(jsonencode(c.arguments)), ...
            localField(report,'completed',false),localField(report,'passed',false), ...
            localField(report,'executedHolds',0),identifier,message, ...
            localField(report,'minimumNodeBodyGap',NaN),localField(report,'minimumSampledBodyGap',NaN), ...
            localField(report,'minimumSampledRoadMargin',NaN),localField(report,'minimumSampledModelDomainMargin',NaN), ...
            max([localField(report,'clfDissipationResidual',NaN),-Inf]), ...
            localField(report,'maximumSlewViolation',NaN), ...
            1000*localField(runtime,'maximumSeconds',NaN),localField(runtime,'deadlineMisses',NaN), ...
            min(states(4,:)),max(abs(states(3,:)))}; %#ok<AGROW>
        results = cell2table(rows,VariableNames=columns);
        writetable(results,fullfile(options.OutputDirectory,'operating.csv'));
        fprintf('[%d/%d] %s %s k=%g completed=%d passed=%d %s\n', ...
            index,numel(cases),c.group,c.scenario,c.curvature,rows{end,6},rows{end,7},identifier);
    end
end

function item = localCase(group,scenario,curvature,caseArguments)
    item = struct('group',group,'scenario',scenario,'curvature',curvature,'arguments',{caseArguments});
end

function value = localField(record,name,default)
    value = default;
    if isfield(record,name),value = record.(name);end
end

function results = localIntegration(options)
    cfg = nrmmTrackingConfig();
    design = synthesizeNrmmObserverGains(cfg);
    rows = cell(0,8);
    for h = [.02,.04,.05,.1,.2,.25,.3,.5,1]
        for maximumStep = [.005,h]
            trial = cfg;
            trial.runtime.samplePeriod = h;trial.runtime.integrationStepMaximum = maximumStep;
            identifier = "";finite = false;errors = nan(1,3);
            try
                result = runOnlineNrmmComplexManeuverScenario(Plot=false,Report=false, ...
                    Config=trial,Duration=6,Seed=options.Seed,DesignFunction=@(~) design);
                finite = result.metrics.allSamplesFinite;
                errors = [result.metrics.relativePositionRmse,result.metrics.targetVelocityRmse, ...
                    result.metrics.targetAccelerationRmse];
            catch exception
                identifier = string(exception.identifier);
            end
            expectedRejection = h==1 && identifier=="onlineNrmmTrackingRuntime:unstableSamplePeriod";
            rows(end+1,:) = {h,maximumStep,finite,errors(1),errors(2),errors(3),identifier,expectedRejection}; %#ok<AGROW>
        end
    end
    results = cell2table(rows,VariableNames={'samplePeriod','requestedStep','finite','positionRmseM', ...
        'velocityRmseMps','accelerationRmseMps2','failureIdentifier','expectedRejection'});
    writetable(results,fullfile(options.OutputDirectory,'integration.csv'));
end
