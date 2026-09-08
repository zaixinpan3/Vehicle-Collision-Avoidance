function information = buildNrmmObserverKernel()
%buildNrmmObserverKernel Compile shared observer and history-enclosure kernels.
% Template numbers specify types only; all gains and domains remain runtime
% inputs. Generated code and binaries stay outside the estimator directory.
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root,"estimator"));
    output = fullfile(root,"solver","nrmm");
    if ~isfolder(output),mkdir(output);end
    addpath(output);
    domain = struct("speedMinimum",5,"speedMaximum",20,"scalarAccelerationMaximum",2, ...
        "yawRateMaximum",0.1,"accelerationNormBound",3,"sideslipMaximum",0.1, ...
        "rearAxleDistance",1.6,"relativePositionMaximum",50);
    design = struct("velocity",struct("gain",1),"position",struct("gain",1), ...
        "target",struct("innovationGains",ones(3,1),"domain",domain), ...
        "yaw",struct("correctionBandwidth",1,"courseModel", ...
        struct("rearAxleDistance",1.5,"sideslipDomainMaximum",0.12)));
    measurement = struct("yawRate",0,"gnssVelocity",zeros(2,1),"bodyAcceleration",zeros(2,1), ...
        "radarDetectionAvailable",coder.typeof(false,[Inf,1],[true,false]), ...
        "correspondence",struct("informative",false,"heading",0));
    types = {coder.typeof(0,[Inf,1],[true,false]),measurement,design,0,0};
    settings = coder.config("mex");settings.GenerateReport = false;settings.EnableOpenMP = false;
    clear nrmmObserverRk4IntervalMex;
    codegen("-config",settings,fullfile(root,"estimator","nrmmObserverRk4Interval.m"), ...
        "-args",types,"-o",fullfile(output,"nrmmObserverRk4IntervalMex"), ...
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
    wrapper = fullfile(output,"nrmmTargetHistoryKernel.m");
    file = fopen(wrapper,"w");
    assert(file>=0,"buildNrmmObserverKernel:writeFailed","Cannot create generated adapter.");
    cleanup = onCleanup(@() fclose(file));
    fprintf(file,"function enclosure = nrmmTargetHistoryKernel(history,time)\n");
    fprintf(file,"enclosure = nrmmTargetHistory('encloseKernel',history,time);\nend\n");
    clear cleanup nrmmTargetHistoryKernelMex;
    codegen("-config",settings,wrapper,"-args",{historyType,0}, ...
        "-o",fullfile(output,"nrmmTargetHistoryKernelMex"), ...
        "-d",fullfile(output,"generatedTargetHistory"));
    rehash;
    information = struct("binary",fullfile(output,"nrmmObserverRk4IntervalMex."+mexext), ...
        "historyBinary",fullfile(output,"nrmmTargetHistoryKernelMex."+mexext), ...
        "matlabVersion",string(version),"algorithmSource","estimator/nrmmObserverRk4Interval.m", ...
        "historySource","estimator/nrmmTargetHistory.m");
end
