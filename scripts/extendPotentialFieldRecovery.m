function rows=extendPotentialFieldRecovery(root,directory,frameCount)
%extendPotentialFieldRecovery Extend unfinished runs until nominal dwell or observation cap.
    cd(root);addpath('controller','config','scripts');
    rows=jsondecode(fileread(fullfile(directory,'summary.json')));
    for index=1:numel(rows)
        r=rows(index);
        if ~r.completed || r.recovery.recovered,continue;end
        stem=fullfile(directory,sprintf('speed%g-%s',r.speed,r.scenario));
        configuration=struct('referenceSpeed',r.speed,'controller',struct('horizonSteps',8+8*(r.speed==15)));
        result=runNonlinearPredictiveSafetyValidation(Scenarios=string(r.scenario),Frames=frameCount, ...
            ControllerConfiguration=configuration,RequireCollisionThreat=true,RecoveryDwellSeconds=1, ...
            OutputFile=stem+"-recovery.json",ContinuationFile=stem+"-recovery.mat",ResumeFrom=stem+".mat");
        next=result.results;next=rmfield(next,{'trace','configuration'});next.speed=r.speed;
        rows(index)=orderfields(next,r);
        fid=fopen(fullfile(directory,'recovery-summary.json'),'w');fprintf(fid,'%s\n',jsonencode(rows));fclose(fid);
        fprintf('RECOVERY %g %s holds=%d recovered=%d entry=%.6g gap=%.6g failure=%s\n', ...
            r.speed,r.scenario,next.executedFrames,next.recovery.recovered,next.recovery.entryTimeSeconds, ...
            next.minimumReplayClearanceMeters,next.failure);
    end
end
