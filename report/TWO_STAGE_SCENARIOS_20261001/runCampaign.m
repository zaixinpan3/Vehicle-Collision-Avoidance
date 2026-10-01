source=fileparts(fileparts(fileparts(mfilename('fullpath'))));
output=tempname;mkdir(output);
fprintf('Output directory: %s\n',output);
cd(source);addpath('scripts','controller','config');
fprintf('MATLAB %s\n',version);
warmup=struct([]);
for speed=[8,15]
    prefix=8;if speed==15,prefix=16;end
    config=struct('referenceSpeed',speed,'controller',struct('horizonSteps',prefix));
    ego=struct('position',[0;0],'yaw',0,'speed',speed,'lateralVelocity',0,'yawRate',0, ...
        'heldActuatorInput',[0;0],'stateTime',0);
    road=struct('centerline',[-100,0;1000,0]);
    for repeat=1:2
        information=prepareCollisionAvoidanceController(ego,road,config);
        entry=struct('speed',speed,'repeat',repeat,'information',information);
        if isempty(warmup),warmup=entry;else,warmup(end+1)=entry;end
        fprintf('WARMUP speed=%g repeat=%d seconds=%.6f\n',speed,repeat,information.elapsedSeconds);
    end
    save(fullfile(output,'warmup.mat'),'warmup');
    folder=fullfile(output,'campaign',"speed"+speed);if ~isfolder(folder),mkdir(folder);end
    for name=["headOn","acceleratingHeadOn","brakingLead","crossing", ...
            "turningCrossing","curvedHeadOn","curvedCrossing"]
        folder=fullfile(output,'campaign',"speed"+speed);wall=tic;
        fprintf('CASE-START speed=%g scenario=%s\n',speed,name);
        try
            report=runNonlinearPredictiveSafetyValidation(Scenarios=name,Frames=1600, ...
                ControllerConfiguration=config,RequireCollisionThreat=true,RecoveryDwellSeconds=5, ...
                OutputFile=fullfile(folder,name+".json"),ContinuationFile=fullfile(folder,name+".mat"));
            r=report.results;
            fprintf('CASE-END speed=%g scenario=%s frames=%d wall=%.3f recovered=%d max=%.6f failedCall=%.6f gap=%.9f failure=%s\n', ...
                speed,name,r.executedFrames,toc(wall),r.recovery.recovered,r.maximumFrameSeconds, ...
                r.failedFrameSeconds,r.minimumReplayClearanceMeters,r.failure);
        catch exception
            failure=struct('speed',speed,'scenario',name,'identifier',exception.identifier, ...
                'message',exception.message,'stack',exception.stack);
            save(fullfile(folder,name+"-driver-failure.mat"),'failure');
            fprintf('DRIVER-FAILURE speed=%g scenario=%s %s\n',speed,name,getReport(exception,'extended','hyperlinks','off'));
        end
    end
end
file=fopen(fullfile(output,'warmup.json'),'w');fprintf(file,'%s\n',jsonencode(warmup));fclose(file);
fprintf('CAMPAIGN-COMPLETE\n');
