function runPotentialFieldCampaign(sourceRoot,outputDirectory,frameCount,options)
%runPotentialFieldCampaign Follow steering-required encounters through recovery.
% By default run to nominal recovery or failure. A finite frameCount is an
% explicitly requested diagnostic observation cap, never a success deadline.
% Empty EstimatorConfiguration selects the actual noisy-sensor NRMM defaults.
% Crossing and curved diagnostic fixtures require a separate longitudinal
% impossibility argument before admission to this steering challenge.
    arguments
        sourceRoot
        outputDirectory
        frameCount (1,1) double {mustBePositive} = Inf
        options.EstimatorConfiguration (1,1) struct = struct()
    end
    cd(sourceRoot);addpath('controller','config','scripts');
    estimatorConfiguration=options.EstimatorConfiguration;
    if isempty(fieldnames(estimatorConfiguration))
        estimatorConfiguration=estimatorControllerIntegrationConfig();
    end
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    summaries=struct([]);
    for speed=[8,15,25]
        names=["headOn","acceleratingHeadOn"];
        if speed==25,names=["stoppedLead","hardBrakingLead"];end
        configuration=struct('referenceSpeed',speed,'model',struct('speedMaximum',max(18,1.2*speed)), ...
            'controller',struct('horizonSteps',8+8*(speed>=15)));
        for name=names
            stem=fullfile(outputDirectory,sprintf('speed%g-%s',speed,name));
            result=runNonlinearPredictiveSafetyValidation(Scenarios=name,Frames=frameCount, ...
                ControllerConfiguration=configuration,RequireCollisionThreat=true,RequireBrakingInsufficient=true,RecoveryDwellSeconds=1, ...
                EstimatorConfiguration=estimatorConfiguration,FailureFile=stem+"-failure.mat", ...
                OutputFile=stem+".json",ContinuationFile=stem+".mat");
            r=result.results;r=rmfield(r,{'trace','configuration'});r.speed=speed;
            summaries=[summaries,r]; %#ok<AGROW>
            fid=fopen(fullfile(outputDirectory,'summary.json'),'w');fprintf(fid,'%s\n',jsonencode(summaries));fclose(fid);
            fprintf('RESULT %g %s holds=%d gap=%.6g recovery=%d max=%.6g failure=%s\n', ...
                speed,name,r.executedFrames,r.minimumReplayClearanceMeters,r.recovery.recovered,r.maximumFrameSeconds,r.failure);
        end
    end
end
