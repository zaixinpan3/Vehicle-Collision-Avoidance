function information = buildNrmmObserverKernel()
%buildNrmmObserverKernel Compile shared observer and history-enclosure kernels.
% Template numbers specify types only; all gains and domains remain runtime
% inputs. Generated code and binaries stay outside the estimator directory.
% Both kernels are thin generated wrappers around one action of an
% action-dispatch estimator function: nrmmObserverVectorField("integrate")
% and nrmmTargetHistory("encloseKernel").
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root,"estimator"));
    output = fullfile(root,"solver","nrmm");
    if ~isfolder(output),mkdir(output);end
    addpath(output);
    % The integration kernel was compiled directly from the former
    % nrmmObserverRk4Interval.m; remove that binary so it cannot shadow the
    % current one.
    legacy = dir(fullfile(output,"nrmmObserverRk4IntervalMex.*"));
    for item = legacy.'
        delete(fullfile(item.folder,item.name));
    end
    domain = struct("speedMinimum",5,"speedMaximum",20,"scalarAccelerationMaximum",2, ...
        "yawRateMaximum",0.1,"accelerationNormBound",3,"sideslipMaximum",0.1, ...
        "rearAxleDistance",1.6,"relativePositionMaximum",50);
    design = struct("velocity",struct("gain",1),"position",struct("gain",1), ...
        "target",struct("innovationGains",ones(3,1),"domain",domain), ...
        "yaw",struct("correctionBandwidth",1,"courseModel", ...
        struct("rearAxleDistance",1.5,"sideslipDomainMaximum",0.12)));
    measurement = struct("yawRate",0,"gnssVelocity",zeros(2,1),"bodyAcceleration",zeros(2,1), ...
        "radarDetectionAvailable",false, ...
        "correspondence",struct("informative",false,"heading",0),"lateralVelocity",0);
    types = {zeros(15,1),measurement,design,0,0};
    settings = coder.config("mex");settings.GenerateReport = false;settings.EnableOpenMP = false;
    integrationWrapper = fullfile(output,"nrmmObserverIntegrationKernel.m");
    localWriteWrapper(integrationWrapper, ...
        "function [states,firstDerivatives] = nrmmObserverIntegrationKernel(state,measurement,design,step,count)\n" ...
        + "[states,firstDerivatives] = nrmmObserverVectorField('integrate',state,measurement,design,step,count);\nend\n");
    clear nrmmObserverIntegrationKernelMex;
    codegen("-config",settings,integrationWrapper, ...
        "-args",types,"-o",fullfile(output,"nrmmObserverIntegrationKernelMex"), ...
        "-d",fullfile(output,"generated"));
    history = nrmmTargetHistory("initialize",domain,0,2,0.6);
    record = struct("time",0,"relativePosition",zeros(2,1),"egoPosition",zeros(2,1), ...
        "yawRate",0,"heading",0,"headingRadius",0);
    history.time = zeros(1,2);history.position = zeros(2,2);history.radius = zeros(2,2);
    history.records = repmat(record,1,2);history.gyroscopeNoise = 0;history.positionNoise = 0;
    historyType = coder.typeof(history);
    historyType.Fields.time = coder.typeof(0,[1,Inf],[false,true]);
    historyType.Fields.position = coder.typeof(0,[2,Inf],[false,true]);
    historyType.Fields.radius = coder.typeof(0,[2,Inf],[false,true]);
    historyType.Fields.records = coder.typeof(record,[1,Inf],[false,true]);
    historyWrapper = fullfile(output,"nrmmTargetHistoryKernel.m");
    localWriteWrapper(historyWrapper, ...
        "function enclosure = nrmmTargetHistoryKernel(history,time)\n" ...
        + "enclosure = nrmmTargetHistory('encloseKernel',history,time);\nend\n");
    clear nrmmTargetHistoryKernelMex;
    codegen("-config",settings,historyWrapper,"-args",{historyType,0}, ...
        "-o",fullfile(output,"nrmmTargetHistoryKernelMex"), ...
        "-d",fullfile(output,"generatedTargetHistory"));
    rehash;
    information = struct("binary",fullfile(output,"nrmmObserverIntegrationKernelMex."+mexext), ...
        "historyBinary",fullfile(output,"nrmmTargetHistoryKernelMex."+mexext), ...
        "matlabVersion",string(version),"algorithmSource","estimator/nrmmObserverVectorField.m", ...
        "historySource","estimator/nrmmTargetHistory.m");
end

function localWriteWrapper(path, text)
    file = fopen(path,"w");
    assert(file>=0,"buildNrmmObserverKernel:writeFailed","Cannot create generated adapter %s.",path);
    cleanup = onCleanup(@() fclose(file));
    fprintf(file,"%s",compose(text));
    clear cleanup;
end
