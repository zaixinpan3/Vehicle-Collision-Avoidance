% Partitioned closed-loop campaign on the frozen snapshot of the RTI controller.
% Set part and parts before running.
source='/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/source';
output='/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/campaign';
cd(source);addpath('scripts','controller','config');caseIndex=0;
fprintf('MATLAB %s, partition %d of %d\n',version,part,parts);
for speed=[8,15]
    prefix=8;if speed==15,prefix=16;end
    config=struct('referenceSpeed',speed,'controller',struct('horizonSteps',prefix));
    folder=fullfile(output,"speed"+speed);
    for name=["headOn","acceleratingHeadOn","brakingLead","crossing","turningCrossing","curvedHeadOn","curvedCrossing"]
        caseIndex=caseIndex+1;
        if mod(caseIndex-1,parts)~=part,continue;end
        fprintf('CASE-START speed=%g scenario=%s\n',speed,name);wall=tic;
        try
            report=runNonlinearPredictiveSafetyValidation(Scenarios=name,Frames=1600, ...
                ControllerConfiguration=config,RequireCollisionThreat=true,RecoveryDwellSeconds=5, ...
                OutputFile=fullfile(folder,name+".json"),ContinuationFile=fullfile(folder,name+".mat"));
            r=report.results;
            fprintf('CASE-END speed=%g scenario=%s frames=%d wall=%.3f recovered=%d max=%.6f gap=%.9f failure=%s\n', ...
                speed,name,r.executedFrames,toc(wall),r.recovery.recovered,r.maximumFrameSeconds, ...
                r.minimumReplayClearanceMeters,r.failure);
        catch exception
            fprintf('DRIVER-FAILURE speed=%g scenario=%s %s\n',speed,name,getReport(exception,'extended','hyperlinks','off'));
        end
    end
end
fprintf('PARTITION-COMPLETE %d\n',part);
