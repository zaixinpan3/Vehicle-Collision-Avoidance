function study = runControllerEntryTimingStudy(campaignFile,outputDirectory,repetitions)
%runControllerEntryTimingStudy Replay the recorded first head-on controller call.
% Timers are injected into a temporary, renamed copy of the entry function.
% Production controller code and the optimization algorithm are unchanged.
% Cache-miss and cache-hit replays are new measurements, not a decomposition
% retroactively measured inside the historical controller call.
    arguments
        campaignFile (1,1) string
        outputDirectory (1,1) string
        repetitions (1,1) double {mustBeInteger,mustBePositive} = 5
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'),fullfile(root,'solver','controller'));
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    campaign=jsondecode(fileread(campaignFile));record=campaign.results;
    assert(isscalar(record) && string(record.scenario)=="headOn");
    frame=record.trace(1);assert(frame.time==0 && frame.solverCalls==3);
    configuration=struct('referenceSpeed',8,'controller',struct('horizonSteps',8));
    [x,q,road,cfg]=collisionThreatScenario("headOn",configuration);
    recordedConfiguration=record.configuration;
    % JSON represents the original infinite slew bound as null.
    assert(isempty(recordedConfiguration.model.brakingRatioRateMaximum) ...
        || isnan(recordedConfiguration.model.brakingRatioRateMaximum));
    recordedConfiguration.model.brakingRatioRateMaximum=Inf;
    assert(isequaln(cfg,recordedConfiguration), ...
        'The reconstructed configuration differs from the recorded experiment.');
    assert(isequal(x,frame.state));
    ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5), ...
        'yawRate',x(6),'stateTime',0,'heldActuatorInput',[0;0]);
    direction=[cos(q(3)+q(6));sin(q(3)+q(6))];velocity=q(4)*direction;
    yawRate=q(4)*sin(q(6))/q(7);
    target=struct('targetPositionInertial',q(1:2),'targetVelocityInertial',velocity, ...
        'targetYawInertial',q(3),'targetYawRate',yawRate,'targetSideslip',q(6), ...
        'targetTangentialAcceleration',q(5),'targetRearAxleDistance',q(7), ...
        'targetAccelerationInertial',q(5)*direction+yawRate*[-velocity(2);velocity(1)], ...
        'targetLength',2*q(8),'targetWidth',2*q(9),'targetRectangleOffset',q(10:11));
    source=fileread(fullfile(root,'controller','collisionAvoidanceController.m'));
    original=source;
    replacements={ ...
        'collisionAvoidanceController(egoState', 'timedCollisionAvoidanceController(egoState'; ...
        'timer=tic;cfg=', 'timer=tic;entryWall=tic;cfg='; ...
        '    if isstruct(previousState),terminal=', sprintf('    entryTiming.inputPreparationSeconds=toc(entryWall);entryWall=tic;\n    if isstruct(previousState),terminal='); ...
        '    nominalReference=nonlinearBicycleModel.cruise', sprintf('    entryTiming.terminalBuildSeconds=toc(entryWall);entryWall=tic;\n    nominalReference=nonlinearBicycleModel.cruise'); ...
        '    [solution,search,model]=solvePredictiveControl', sprintf('    entryTiming.modelAssemblySeconds=toc(entryWall);entryWall=tic;\n    [solution,search,model]=solvePredictiveControl'); ...
        '    if isempty(solution)', sprintf('    entryTiming.predictiveSolveSeconds=toc(entryWall);entryWall=tic;\n    if isempty(solution)'); ...
        '    metadata=struct', sprintf('    entryTiming.commandAndTargetSeconds=toc(entryWall);entryWall=tic;\n    metadata=struct'); ...
        '    if ~explicitState,lastState=controllerState;end', sprintf(['    entryTiming.outputAssemblySeconds=toc(entryWall);\n', ...
            '    entryTiming.totalSeconds=toc(timer);\n', ...
            '    prediction.metadata.entryTiming=entryTiming;\n', ...
            '    if ~explicitState,lastState=controllerState;end'])};
    for index=1:size(replacements,1)
        assert(isscalar(strfind(source,replacements{index,1})),'Timer injection location is not unique.');
        source=strrep(source,replacements{index,1},replacements{index,2});
    end
    timedFile=fullfile(outputDirectory,'timedCollisionAvoidanceController.m');
    localWrite(timedFile,source);addpath(outputDirectory,'-begin');
    cleanup=onCleanup(@()rmpath(outputDirectory));
    assert(strcmp(which('timedCollisionAvoidanceController'),timedFile));
    % Warm execution and JIT separately from the measured cache comparisons.
    warmup=localRun("processWarmup",true);
    terminalContinuation.build(cfg,0);
    baseline=localRun("unmodifiedWarm",false);
    alternate=cfg;alternate.nonlinear.terminalRadius=2*cfg.nonlinear.terminalRadius;
    rows=cell(1,2*repetitions);
    for index=1:repetitions
        % The terminal constructor retains one configuration. Changing only
        % its initial radius evicts that entry without clearing MATLAB/JIT.
        terminalContinuation.build(alternate,0);
        rows{2*index-1}=localRun("terminalCacheMiss",true);
        rows{2*index}=localRun("terminalCacheHit",true);
    end
    terminalContinuation.build(alternate,0);
    profile clear;profile on;
    profiled=localRun("profiledTerminalCacheMiss",true);
    profile off;profileData=profile('info');
    save(fullfile(outputDirectory,'profile.mat'),'profileData');
    functions=profileData.FunctionTable;
    functions=rmfield(functions,intersect(fieldnames(functions),{'Children','Parents','ExecutedLines','PartialData'}));
    localWrite(fullfile(outputDirectory,'profile-functions.json'),jsonencode(functions));
    core=terminalContinuation.build(cfg,0);
    coreSummary=struct('initialRadius',cfg.nonlinear.terminalRadius,'radius',core.radius, ...
        'radiusHalvings',round(log2(cfg.nonlinear.terminalRadius/core.radius)), ...
        'contractionBound',core.contractionBound,'construction',core.construction);
    study=struct('scope',"New single-frame replays; original timing is retained separately", ...
        'version',version,'campaignFile',campaignFile,'configuration',cfg, ...
        'historicalFrame',frame,'warmup',warmup,'unmodifiedWarm',baseline, ...
        'replays',[rows{:}],'profiled',profiled,'terminalCore',coreSummary);
    assert(strcmp(original,fileread(fullfile(root,'controller','collisionAvoidanceController.m'))));
    localWrite(fullfile(outputDirectory,'replays.json'),jsonencode(study));
    function row=localRun(label,instrumented)
        wall=tic;
        if instrumented
            [command,~,prediction]=timedCollisionAvoidanceController(ego,target,road,cfg,[]);
            timing=prediction.metadata.entryTiming;
        else
            [command,~,prediction]=collisionAvoidanceController(ego,target,road,cfg,[]);
            timing=struct();
        end
        seconds=toc(wall);search=prediction.metadata.search;
        error=norm(command.actuatorInput-frame.input,inf);
        assert(error<1e-10 && search.solverCalls==3, ...
            'Replay differs from the recorded control or optimization sequence.');
        row=struct('label',label,'controllerSeconds',seconds,'entryTiming',timing, ...
            'inputError',error,'search',search);
        fprintf('%s: %.3f ms, input error %.3g, %d solves\n', ...
            label,1000*seconds,error,search.solverCalls);
    end
end

function localWrite(file,contents)
    handle=fopen(file,'w');assert(handle>=0);cleanup=onCleanup(@()fclose(handle));
    fprintf(handle,'%s\n',contents);
end
