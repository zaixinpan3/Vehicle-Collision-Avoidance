function campaign = runCollisionThreatValidation(options)
%runCollisionThreatValidation Test encounters in which immediate full straight braking collides.
% Use the actual NRMM observer with synthetic noisy sensors by default.
% Historical benign and exact-state fixtures remain separate diagnostics.
    arguments
        options.Frames (1,1) double {mustBePositive} = Inf
        options.Speeds (1,:) double {mustBePositive,mustBeFinite} = 25
        options.Scenarios (1,:) string = ["stoppedLead","hardBrakingLead"]
        options.OutputDirectory (1,1) string = string(tempname)
        options.EstimatorConfiguration (1,1) struct = struct()
        options.RecoveryDwellSeconds (1,1) double {mustBeNonnegative,mustBeFinite} = 1
        options.RecoveryMinimumSeconds (1,1) double {mustBeNonnegative,mustBeFinite} = 0
    end
    names = options.Scenarios;
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    estimatorConfiguration = options.EstimatorConfiguration;
    if isempty(fieldnames(estimatorConfiguration))
        estimatorConfiguration = estimatorControllerIntegrationConfig();
    end
    summaries = struct([]);
    for speed = options.Speeds
        folder = fullfile(options.OutputDirectory,"speed"+speed);
        if ~isfolder(folder),mkdir(folder);end
        cfg = struct('referenceSpeed',speed,'model',struct('speedMaximum',max(18,1.2*speed)));
        for name = names
            file = fullfile(folder,name+".json");
            entry = struct('referenceSpeedMetersPerSecond',speed,'scenario',name,'source',"", ...
                'observationModel',"nrmmSyntheticSensors",'randomSeed',estimatorConfiguration.randomSeed, ...
                'baselineCollision',NaN,'maximumBrakingCollision',NaN,'baselineFirstCollisionSeconds',NaN, ...
                'completed',false,'recovery',struct('recovered',false),'outcome',"setupError", ...
                'executedFrames',0,'failure',"",'minimumReplayClearanceMeters',NaN, ...
                'initializationSeconds',NaN,'maximumRunningSeconds',NaN);
            try
                result = runNonlinearPredictiveSafetyValidation(Scenarios=name,Frames=options.Frames, ...
                    ControllerConfiguration=cfg,EstimatorConfiguration=estimatorConfiguration, ...
                    RequireCollisionThreat=true,RequireBrakingInsufficient=true,OutputFile=file, ...
                    RecoveryDwellSeconds=options.RecoveryDwellSeconds,RecoveryMinimumSeconds=options.RecoveryMinimumSeconds);
                r = result.results;
                entry.source = file;
                entry.baselineCollision = r.baselineCruise.collisionDetected;
                entry.maximumBrakingCollision = r.baselineMaximumBraking.collisionDetected;
                entry.baselineFirstCollisionSeconds = r.baselineCruise.firstCollisionSeconds;
                entry.completed = r.completed;
                entry.recovery = r.recovery;
                entry.outcome = r.outcome;
                entry.executedFrames = r.executedFrames;
                entry.failure = r.failure;
                entry.minimumReplayClearanceMeters = r.minimumReplayClearanceMeters;
                entry.initializationSeconds = r.firstFrameSeconds;
                if numel(r.trace)>1,entry.maximumRunningSeconds=max([r.trace(2:end).controllerSeconds]);end
            catch exception
                if ~strcmp(exception.identifier,'nrmmEstimatorControllerAdapter:invalidTargetInitialization')
                    rethrow(exception);
                end
                % Retain unsupported target initialization as a failed case;
                % never replace observer measurements with exact states.
                entry.failure = string(exception.identifier)+": "+string(exception.message);
            end
            if isempty(summaries),summaries=entry;else,summaries(end+1)=entry;end %#ok<AGROW>
        end
    end
    campaign = struct('scope',"joint NRMM observer/controller; cruise and maximum-straight-braking collision counterfactuals", ...
        'scenarioOrder',names,'speedsMetersPerSecond',options.Speeds,'frames',options.Frames, ...
        'results',summaries);
    file = fopen(fullfile(options.OutputDirectory,'campaign.json'),'w');assert(file>=0);
    cleanup = onCleanup(@()fclose(file));fprintf(file,'%s\n',jsonencode(campaign));
end
