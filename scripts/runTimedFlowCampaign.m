function runTimedFlowCampaign(sourceRoot,outputDirectory,frameCount)
    cd(sourceRoot);addpath('controller','config','scripts');
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    names=["headOn","acceleratingHeadOn","brakingLead","crossing","turningCrossing","curvedHeadOn","curvedCrossing"];
    summaries=struct([]);
    for speed=[8,15]
        configuration=struct('referenceSpeed',speed,'controller',struct('horizonSteps',8+8*(speed==15)));
        for name=names
            stem=fullfile(outputDirectory,sprintf('speed%g-%s',speed,name));
            result=runNonlinearPredictiveSafetyValidation(Scenarios=name,Frames=frameCount, ...
                ControllerConfiguration=configuration,RequireCollisionThreat=true,RecoveryDwellSeconds=1, ...
                OutputFile=stem+".json",ContinuationFile=stem+".mat");
            r=result.results;r=rmfield(r,{'trace','configuration'});r.speed=speed;
            summaries=[summaries,r]; %#ok<AGROW>
            fid=fopen(fullfile(outputDirectory,'summary.json'),'w');fprintf(fid,'%s\n',jsonencode(summaries));fclose(fid);
            fprintf('RESULT %g %s holds=%d complete=%d gap=%.6g recovery=%d max=%.6g failure=%s\n', ...
                speed,name,r.executedFrames,r.completed,r.minimumReplayClearanceMeters,r.recovery.recovered,r.maximumFrameSeconds,r.failure);
        end
    end
end
