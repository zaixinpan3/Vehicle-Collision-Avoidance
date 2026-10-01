source='/home/zai/.cache/collisionAvoidance/rti-controller-20261001/source';
output='/home/zai/.cache/collisionAvoidance/rti-controller-20261001';
cd(source);addpath('scripts','controller','config');caseIndex=0;
if ~exist('rtiPartition','var'),rtiPartition=0;end
if ~exist('rtiPartitions','var'),rtiPartitions=1;end
for speed=[8,15]
    prefix=8;if speed==15,prefix=16;end
    config=struct('referenceSpeed',speed,'controller',struct('horizonSteps',prefix));
    folder=fullfile(output,'campaign',"speed"+speed);
    for name=["headOn","acceleratingHeadOn","brakingLead","crossing","turningCrossing","curvedHeadOn","curvedCrossing"]
        caseIndex=caseIndex+1;
        if mod(caseIndex-1,rtiPartitions)~=rtiPartition,continue;end
        saved=load(fullfile(folder,name+".mat"),'continuation');r=saved.continuation.result;
        if strlength(r.failure)>0 || r.recovery.recovered || r.minimumReplayClearanceMeters<=0,continue;end
        if ~isfile(fullfile(folder,name+"-20s.json"))
            copyfile(fullfile(folder,name+".json"),fullfile(folder,name+"-20s.json"));
        end
        fprintf('EXTEND-START speed=%g scenario=%s\n',speed,name);wall=tic;
        report=runNonlinearPredictiveSafetyValidation(Scenarios=name,Frames=1600, ...
            ControllerConfiguration=config,RequireCollisionThreat=true,RecoveryDwellSeconds=5, ...
            ResumeFrom=fullfile(folder,name+".mat"),OutputFile=fullfile(folder,name+".json"), ...
            ContinuationFile=fullfile(folder,name+".mat"));
        r=report.results;
        fprintf('EXTEND-END speed=%g scenario=%s frames=%d wall=%.3f recovered=%d max=%.6f failedCall=%.6f gap=%.9f failure=%s\n', ...
            speed,name,r.executedFrames,toc(wall),r.recovery.recovered,r.maximumFrameSeconds, ...
            r.failedFrameSeconds,r.minimumReplayClearanceMeters,r.failure);
    end
end
