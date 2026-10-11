function runOneEncounter(sourceRoot,outputDirectory,speed,name,seed,frameCount,variant)
%runOneEncounter One campaign encounter (study driver; seed 0 = exact states).
% Mirrors scripts/runPotentialFieldCampaign.m for one (speed, scenario, seed),
% as report/TERMINAL_BACKUP_SET_20261008/scripts/runOneEncounter.m, and also
% records the terminal fields of the braking-continuation set where the
% source under test provides them (absent fields are recorded as empty).
% variant: struct with optional fields 'controller' (override struct) and
% 'estimator' (override struct merged into the estimator configuration).
    if nargin<7,variant=struct();end
    cd(sourceRoot);addpath('controller','config','scripts','estimator');
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    configuration=struct('referenceSpeed',speed,'controller',struct('horizonSteps',8+8*(speed==15)));
    if isfield(variant,'controller'),configuration=localMerge(configuration,variant.controller);end
    c=struct();
    if seed~=0
        c=estimatorControllerIntegrationConfig();c.randomSeed=seed;
        if isfield(variant,'estimator'),c=localMerge(c,variant.estimator);end
    end
    stem=fullfile(outputDirectory,sprintf('seed%d-speed%g-%s',seed,speed,name));
    timer=tic;
    try
        result=runNonlinearPredictiveSafetyValidation(Scenarios=string(name),Frames=frameCount, ...
            ControllerConfiguration=configuration,RequireCollisionThreat=true,RecoveryDwellSeconds=1, ...
            EstimatorConfiguration=c,FailureFile=stem+"-failure.mat",OutputFile=stem+".json");
        r=result.results;trace=r.trace;r=rmfield(r,{'trace','configuration'});
        r.seed=seed;r.speed=speed;r.wallSeconds=toc(timer);
        names=["time","primaryOptimum","horizonSteps","controllerSeconds","terminalEnd","terminalExitSeconds", ...
            "terminalStopSeconds","terminalMarginMeters","terminalLevel","anchorCertified","candidateFeasible", ...
            "acceptedStep","appendedTerminalSteps","appendedBrakingSteps","appendedGuidedSteps","extensionStopped","initialization", ...
            "potentialFieldRestarted","terminationReason"];
        frames=struct();
        for k=1:numel(trace)
            for n=names
                value=[];if isfield(trace,n),value=trace(k).(n);end
                frames(k).(n)=value;
            end
        end
        if isempty(trace),frames=struct([]);end
        r.frames=frames;
    catch exception
        r=struct('seed',seed,'speed',speed,'scenario',string(name),'failure',string(exception.identifier)+": "+string(exception.message), ...
            'wallSeconds',toc(timer),'harnessError',true);
    end
    fid=fopen(stem+"-summary.json",'w');fprintf(fid,'%s\n',jsonencode(r));fclose(fid);
end

function a=localMerge(a,b)
    for f=fieldnames(b).'
        if isfield(a,f{1}) && isstruct(a.(f{1})) && isstruct(b.(f{1})),a.(f{1})=localMerge(a.(f{1}),b.(f{1}));
        else,a.(f{1})=b.(f{1});end
    end
end
