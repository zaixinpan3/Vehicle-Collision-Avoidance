function information = buildBicycleNominalKernel()
%buildBicycleNominalKernel Compile shared prediction and enclosure kernels.
% Generated entry adapters and binaries live under solver/. The algorithm
% remains in ltvBicycleModel and stateUncertainty; all parameters are runtime inputs.
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root,"controller"),fullfile(root,"config"));
    output = fullfile(root,"solver","bicycle");
    if ~isfolder(output),mkdir(output);end
    wrapper = fullfile(output,"bicycleNominalKernel.m");
    file = fopen(wrapper,"w");
    assert(file>=0,"buildBicycleNominalKernel:writeFailed","Cannot create generated adapter.");
    cleanup = onCleanup(@() fclose(file));
    fprintf(file,"function [x,a,b] = bicycleNominalKernel(x0,u,h,s,k,cfg,tire,bias,linearize)\n");
    fprintf(file,"[x,a,b] = ltvBicycleModel.nominalKernel(x0,u,h,s,k,cfg,tire,bias,linearize);\nend\n");
    clear cleanup;
    addpath(output);
    cfg = collisionAvoidanceControllerConfig();
    kernelCfg = struct("vehicle",struct("m",cfg.vehicle.m,"Iz",cfg.vehicle.Iz, ...
        "lf",cfg.vehicle.lf,"lr",cfg.vehicle.lr,"gravity",cfg.vehicle.gravity), ...
        "model",struct("scheduleSpeedFloor",cfg.model.scheduleSpeedFloor),"roadLoad",cfg.roadLoad);
    parameters = modifiedFialaTire.parameters(cfg);
    tire = struct("corneringStiffness",parameters.corneringStiffness, ...
        "longitudinalForceScale",parameters.longitudinalForceScale);
    types = {zeros(6,1),coder.typeof(0,[2,Inf],[false,true]),0, ...
        coder.typeof(0,[Inf,1],[true,false]),coder.typeof(0,[Inf,1],[true,false]), ...
        kernelCfg,tire,0,false};
    settings = coder.config("mex");
    settings.GenerateReport = false;
    settings.EnableOpenMP = false;
    clear bicycleNominalKernelMex;
    codegen("-config",settings,wrapper,"-args",types,"-o", ...
        fullfile(output,"bicycleNominalKernelMex"),"-d",fullfile(output,"generated"));
    wrapper = fullfile(output,"bicycleLinearizationKernel.m");
    file = fopen(wrapper,"w");
    assert(file>=0,"buildBicycleNominalKernel:writeFailed","Cannot create generated adapter.");
    cleanup = onCleanup(@() fclose(file));
    fprintf(file,"function [a,b,c,tire,q] = bicycleLinearizationKernel(x,u,k,cfg,bias,rate,h)\n");
    fprintf(file,"[a,b,c,tire,q] = ltvBicycleModel.linearizationKernel(x,u,k,cfg,bias,rate,h);\nend\n");
    clear cleanup;
    kernelCfg.tire = cfg.tire;
    types = {coder.typeof(0,[6,Inf],[false,true]),coder.typeof(0,[2,Inf],[false,true]), ...
        coder.typeof(0,[1,Inf],[false,true]),kernelCfg,0,zeros(6,1),0};
    clear bicycleLinearizationKernelMex;
    codegen("-config",settings,wrapper,"-args",types,"-o", ...
        fullfile(output,"bicycleLinearizationKernelMex"),"-d",fullfile(output,"generatedLinearization"));
    wrapper = fullfile(output,"bicycleHeldIntervalKernel.m");
    file = fopen(wrapper,"w");
    assert(file>=0,"buildBicycleNominalKernel:writeFailed","Cannot create generated adapter.");
    cleanup = onCleanup(@() fclose(file));
    fprintf(file,"function tubes = bicycleHeldIntervalKernel(a,b,c,map,x,r,q,h,p,xMax,uMax,n,cells)\n");
    fprintf(file,"tubes = stateUncertainty.heldInterval(a,b,c,map,x,r,q,h,p,xMax,uMax,n,cells);\nend\n");
    clear cleanup;
    types = {zeros(6),coder.typeof(0,[6,Inf],[false,true]),zeros(6,1), ...
        coder.typeof(0,[6,Inf],[false,true]),zeros(6,1),zeros(6,1),zeros(6,1),0,0, ...
        zeros(6,1),coder.typeof(0,[Inf,1],[true,false]),zeros(6,1),0};
    clear bicycleHeldIntervalKernelMex;
    codegen("-config",settings,wrapper,"-args",types,"-o", ...
        fullfile(output,"bicycleHeldIntervalKernelMex"),"-d",fullfile(output,"generatedHeldInterval"));
    rehash;
    information = struct("binary",fullfile(output,"bicycleNominalKernelMex."+mexext), ...
        "linearizationBinary",fullfile(output,"bicycleLinearizationKernelMex."+mexext), ...
        "heldIntervalBinary",fullfile(output,"bicycleHeldIntervalKernelMex."+mexext), ...
        "matlabVersion",string(version),"algorithmSource","controller/ltvBicycleModel.m", ...
        "enclosureSource","controller/stateUncertainty.m");
end
