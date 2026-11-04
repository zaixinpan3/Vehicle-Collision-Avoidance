function summaries = runNonlinearControllerExperiment(outputDirectory,options)
%runNonlinearControllerExperiment Joint raw-sensor output-feedback campaign.
% The controller optimizes both memory and actuator maps. Each online call
% takes at most one SQP step after initialization and checks its finite-scenario
% nonlinear constraints. Timings are measured, not a real-time guarantee.
    arguments
        outputDirectory (1,1) string
        options.Scenarios (1,:) string = ["stoppedLead","hardBrakingLead","crossing","curvedCrossing"]
        options.Speeds (1,:) double = [8,8,8,8]
        options.Frames (1,1) double {mustBePositive,mustBeInteger,mustBeFinite} = 100
        options.MaximumIterations (1,1) double {mustBePositive,mustBeInteger} = 20
        options.FrameBudgetSeconds (1,1) double {mustBePositive} = 30
        options.ControllerConfiguration (1,1) struct = struct()
    end
    assert(numel(options.Scenarios)==numel(options.Speeds));
    root=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(root,'config'));
    cfg=collisionAvoidanceControllerConfig(options.ControllerConfiguration);
    cfg.outputFeedback.initializationIterations=options.MaximumIterations;
    cfg.outputFeedback.maximumSolveSeconds=options.FrameBudgetSeconds;
    summaries=struct([]);
    for k=1:numel(options.Scenarios)
        folder=fullfile(outputDirectory,options.Scenarios(k));
        campaign=runJointOutputFeedbackScenario(Scenarios=options.Scenarios(k), ...
            Speeds=options.Speeds(k),Frames=options.Frames, ...
            ControllerConfiguration=cfg,OutputDirectory=folder);
        if isempty(summaries),summaries=campaign.results;else,summaries(end+1)=campaign.results;end %#ok<AGROW>
    end
    file=fopen(fullfile(outputDirectory,'summary.json'),'w');assert(file>=0);
    cleanup=onCleanup(@()fclose(file));fprintf(file,'%s\n',jsonencode(summaries,PrettyPrint=true));
end
