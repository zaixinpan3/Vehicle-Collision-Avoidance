function exportArm64Sources(directory,configuration,device)
%exportArm64Sources Generate standalone NRMM and PCBF numerical components.
% The workstation needs MATLAB Coder and the project's CasADi/IPOPT package.
% The target domain, observer gains, model and mesh are specialized together.
% Use a dedicated build directory, then run scripts/native/buildArm64Kernels.sh.
    arguments
        directory (1,1) string
        configuration (1,1) struct = struct()
        device (1,1) string = "ARM Compatible->ARM Cortex-A (64-bit)"
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    native=fullfile(root,'scripts','native');
    addpath(native,fullfile(root,'config'),fullfile(root,'controller'),fullfile(root,'estimator'));
    setupJointNlpSolver;
    cfg=collisionAvoidanceControllerConfig(configuration);
    build=coder.config('lib');
    build.GenCodeOnly=true;
    build.GenerateReport=false;
    build.EnableOpenMP=false;
    build.HardwareImplementation.ProdHWDeviceType=char(device);

    localFlows(fullfile(directory,'generated'),cfg);
    codegen('-config',build,fullfile(native,'nrmmObserverInterval.m'), ...
        '-args',nrmmObserverIntervalTypes(),'-d',fullfile(directory,'nrmm'));
    localTarget(native,fullfile(directory,'target'),cfg,build);
    localGeometry(native,fullfile(directory,'geometry'),cfg,build);
end

function localFlows(outputDirectory,cfg)
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    [ok,folder]=fileattrib(outputDirectory);
    assert(ok,'exportPredictiveKernels:invalidDirectory','Cannot resolve the output directory.');
    kernels=solvePredictiveControl("outputFeedback",struct('operation',"predictionKernels",'cfg',cfg));
    generator=casadi.CodeGenerator('pcbf_kernels.c',struct('with_header',true));
    names=fieldnames(kernels);
    for i=1:numel(names)
        generator.add(kernels.(names{i}));
    end
    generator.generate([folder.Name,filesep]);
    save(fullfile(folder.Name,'configuration.mat'),'cfg');
    tire=nonlinearBicycleModel.tireParameters(cfg);
    runtime=struct('cfg',cfg,'trim',terminalSafeSet.movingControl(cfg), ...
        'rearSlipScale',tire.longitudinalForceScale(2)/tire.corneringStiffness(2));
    f=fopen(fullfile(folder.Name,'controller.json'),'w');assert(f>=0);
    close=onCleanup(@()fclose(f));fprintf(f,'%s',jsonencode(runtime));

end

function localTarget(native,outputDirectory,cfg,build)
    [design,kernel,settings,bound]=onlineNrmmTrackingRuntime('targetDesign',cfg);
    parameters=struct('cfg',struct('outputFeedback',struct('noiseBounds',cfg.outputFeedback.noiseBounds), ...
        'target',struct('rearAxleDistance',cfg.target.rearAxleDistance)), ...
        'design',struct('domain',design.domain),'kernel',kernel,'settings',struct( ...
        'initialization',settings.initialization,'sensor',settings.sensor,'observer',struct('runtime', ...
        struct('integrationStepMaximum',settings.observer.runtime.integrationStepMaximum))),'bound',bound);
    anchor=struct('time',0,'lower',0,'upper',0,'fullWindow',false);
    boundTemplate=struct('targetComponents',zeros(3,1),'targetLyapunov',0,'trueRangeMaximum',0,'radarPredictor',0);
    track=struct('state',zeros(6,1),'time',0,'lastReturn',0,'previousPosition',zeros(2,1), ...
        'previousRadius',0,'positionRadii',0,'errorBound',boundTemplate,'times',0,'positions',zeros(2,1), ...
        'initialized',false,'heading',0,'detections',0,'restCompatible',false,'motionBounds',zeros(2), ...
        'speedAnchor',anchor,'positionBounds',zeros(2),'speedUpper',design.domain.speedMaximum, ...
        'restBounds',zeros(2),'lastReturnHeading',NaN);
    t=coder.typeof(track);
    t.Fields.previousPosition=coder.typeof(0,[2,1],[true,true]);
    t.Fields.positionRadii=coder.typeof(0,[1,Inf],[false,true]);t.Fields.times=t.Fields.positionRadii;
    t.Fields.positions=coder.typeof(0,[2,Inf],[false,true]);
    t.Fields.motionBounds=coder.typeof(0,[2,2],[true,true]);
    t.Fields.positionBounds=t.Fields.motionBounds;t.Fields.restBounds=t.Fields.motionBounds;
    t.Fields.errorBound=coder.typeof(boundTemplate,[1,1],[true,true]);
    t.Fields.speedAnchor=coder.typeof(anchor,[1,1],[true,true]);
    codegen('-config',build,fullfile(native,'nrmmTargetSample.m'),'-args', ...
        {t,0,t.Fields.previousPosition,0,0,0,coder.Constant(parameters)},'-d',outputDirectory);
end

function localGeometry(native,outputDirectory,cfg,build)
    fullConfiguration=cfg;
    curve=struct('origin',zeros(2,1),'heading',0,'curvature',0,'length',1);
    cfg=struct('collision',cfg.collision,'outputFeedback',struct('clearanceMeters',cfg.outputFeedback.clearanceMeters, ...
        'nlpTolerance',cfg.outputFeedback.nlpTolerance),'vehicle',cfg.vehicle,'controller',struct('sampleTime',cfg.controller.sampleTime), ...
        'target',cfg.target);
    codegen('-config',build,fullfile(native,'pcbfNodeClearance.m'), ...
        '-args',{zeros(3,1),zeros(11,1),zeros(4,1)},fullfile(native,'pcbfTargetEnvelope.m'), ...
        '-args',{zeros(11,1),zeros(2),coder.typeof(0,[1,Inf],[false,true]),coder.Constant(cfg.target.observerDomain)}, ...
        fullfile(native,'pcbfTerminalSector.m'),'-args',{zeros(6,1),zeros(11,1),zeros(2,1),0,0,0,coder.Constant(cfg)}, ...
        fullfile(native,'pcbfTerminalKnown.m'),'-args',{zeros(6,1),zeros(11,1),0,coder.Constant(cfg)}, ...
        fullfile(native,'pcbfFieldInputs.m'),'-args',{zeros(6,1),zeros(2,1),zeros(11,1),curve, ...
        coder.typeof(0,[11,Inf],[false,true]),coder.typeof(0,[4,Inf],[false,true]),0,coder.Constant(fullConfiguration)}, ...
        fullfile(native,'pcbfMemberPreview.m'),'-args',{zeros(6,1),coder.typeof(0,[11,1],[true,false]),coder.Constant(fullConfiguration)}, ...
        fullfile(native,'pcbfHoldDomain.m'),'-args',{zeros(6,1+fullConfiguration.outputFeedback.validationSubsteps),zeros(2,1),coder.Constant(fullConfiguration)}, ...
        fullfile(native,'pcbfCruiseInputs.m'),'-args',{zeros(6,1),zeros(2,1),curve,zeros(2,1),0,coder.Constant(fullConfiguration)}, ...
        fullfile(native,'pcbfTrimResidual.m'),'-args',{zeros(3,1),0,coder.Constant(fullConfiguration)}, ...
        fullfile(native,'pcbfTargetAt.m'),'-args',{zeros(11,1),0}, ...
        '-d',outputDirectory);
end
