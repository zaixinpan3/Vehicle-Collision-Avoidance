function summary=runStandaloneKernelClosedLoop(mexDirectory,outputDirectory,options)
%runStandaloneKernelClosedLoop Experimental native numerical kernel integration.
% Complete frame timings INCLUDE MATLAB preparation, MEX marshalling and the
% unchanged final verifier. This is a hybrid run, not a standalone pipeline.
    arguments
        mexDirectory (1,1) string
        outputDirectory (1,1) string
        options.SampleCount (1,1) double = 600
        options.Curvatures (1,:) double = [0,.01]
    end
    root=fileparts(fileparts(mfilename('fullpath')));originalPath=path;
    warningState=warning('off','Coder:toolbox:FindVectorOrientationMismatch');
    restoreWarning=onCleanup(@()warning(warningState));
    restore=onCleanup(@()path(originalPath));
    source=fullfile(outputDirectory,'source');
    for folder=["controller","config","scripts"]
        destination=fullfile(source,folder);if ~isfolder(destination),mkdir(destination);end
        files=dir(fullfile(root,folder,'*.m'));
        for index=1:numel(files),copyfile(fullfile(files(index).folder,files(index).name),destination);end
    end
    file=fullfile(source,'controller','solveHardCbfClf.m');code=fileread(file);
    needle='            [program,result,search]=localJointSearch(program,model,cfg);';
    assert(contains(code,needle),'The native integration point changed.');
    code=strrep(code,needle,'            [program,result,search]=standaloneControllerBenchmark.nativeJoint(program,model,cfg);');
    fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s',code);fclose(fid);
    addpath(fullfile(source,'controller'),fullfile(source,'config'),fullfile(source,'scripts'));
    addpath(mexDirectory,fullfile(root,'solver','clarabel','matlab'));
    clear solveHardCbfClf collisionAvoidanceController
    runJointSupportCertificateValidation(OutputDirectory=fullfile(outputDirectory,'warmup'), ...
        SampleCount=120,Curvatures=options.Curvatures);
    summary=runJointSupportCertificateValidation(OutputDirectory=fullfile(outputDirectory,'measured'), ...
        SampleCount=options.SampleCount,Curvatures=options.Curvatures);
    clear solveHardCbfClf collisionAvoidanceController
end
