function campaign = runCollisionThreatValidation(options)
%runCollisionThreatValidation Test encounters in which immediate full straight braking collides.
% Historical benign fixtures remain available as separate regression controls.
    arguments
        options.Frames (1,1) double {mustBePositive} = Inf
        options.Speeds (1,:) double {mustBePositive,mustBeFinite} = 25
        options.Scenarios (1,:) string = ["stoppedLead","hardBrakingLead"]
        options.OutputDirectory (1,1) string = string(tempname)
        options.RecoveryDwellSeconds (1,1) double {mustBeNonnegative,mustBeFinite} = 1
        options.RecoveryMinimumSeconds (1,1) double {mustBeNonnegative,mustBeFinite} = 0
    end
    names = options.Scenarios;
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    summaries = struct([]);
    for speed = options.Speeds
        folder = fullfile(options.OutputDirectory,"speed"+speed);
        if ~isfolder(folder),mkdir(folder);end
        cfg = struct('referenceSpeed',speed,'model',struct('speedMaximum',max(18,1.2*speed)));
        for name = names
            file = fullfile(folder,name+".json");
            result = runNonlinearPredictiveSafetyValidation(Scenarios=name,Frames=options.Frames, ...
                ControllerConfiguration=cfg,RequireCollisionThreat=true,RequireBrakingInsufficient=true,OutputFile=file, ...
                RecoveryDwellSeconds=options.RecoveryDwellSeconds,RecoveryMinimumSeconds=options.RecoveryMinimumSeconds);
            r = result.results;
            entry = struct('referenceSpeedMetersPerSecond',speed,'scenario',name,'source',file, ...
                'baselineCollision',r.baselineCruise.collisionDetected, ...
                'maximumBrakingCollision',r.baselineMaximumBraking.collisionDetected, ...
                'baselineFirstCollisionSeconds',r.baselineCruise.firstCollisionSeconds, ...
                'completed',r.completed,'recovery',r.recovery,'executedFrames',r.executedFrames,'failure',r.failure, ...
                'minimumReplayClearanceMeters',r.minimumReplayClearanceMeters, ...
                'initializationSeconds',r.firstFrameSeconds,'maximumRunningSeconds',NaN);
            if numel(r.trace)>1,entry.maximumRunningSeconds=max([r.trace(2:end).controllerSeconds]);end
            if isempty(summaries),summaries=entry;else,summaries(end+1)=entry;end %#ok<AGROW>
        end
    end
    campaign = struct('scope',"cruise and maximum-straight-braking collision counterfactuals; exact observations", ...
        'scenarioOrder',names,'speedsMetersPerSecond',options.Speeds,'frames',options.Frames, ...
        'results',summaries);
    file = fopen(fullfile(options.OutputDirectory,'campaign.json'),'w');assert(file>=0);
    cleanup = onCleanup(@()fclose(file));fprintf(file,'%s\n',jsonencode(campaign));
end
