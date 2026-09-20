function summary=captureStandaloneControllerFrames(directory,options)
%captureStandaloneControllerFrames Record prepared frames from scenario runs.
% A disposable source copy instruments the numerical entry. Its I/O-inflated
% frame timings are not timing results; standalone replay excludes fixture I/O.
    arguments
        directory (1,1) string
        options.SampleCount (1,1) double = 600
        options.Curvatures (1,:) double = [0,.01,-.01]
        options.Scenarios (1,:) string = ["stationary","oncoming","crossing"]
    end
    root=fileparts(fileparts(mfilename('fullpath')));originalPath=path;
    restore=onCleanup(@()path(originalPath));
    source=fullfile(directory,'source');
    if ~isfolder(source),mkdir(source);end
    for folder=["controller","config","scripts"]
        destination=fullfile(source,folder);if ~isfolder(destination),mkdir(destination);end
        files=dir(fullfile(root,folder,'*.m'));
        for index=1:numel(files),copyfile(fullfile(files(index).folder,files(index).name),destination);end
    end
    file=fullfile(source,'controller','solveHardCbfClf.m');code=fileread(file);
    needle='            [program,result,search]=localJointSearch(program,model,cfg);';
    assert(contains(code,needle),'The capture insertion point changed.');
    code=strrep(code,needle,['            standaloneControllerBenchmark.capture(program,cfg);',newline,needle]);
    fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s',code);fclose(fid);
    addpath(fullfile(source,'controller'),fullfile(source,'config'),fullfile(source,'scripts'));
    addpath(fullfile(root,'solver','clarabel','matlab'));
    clear solveHardCbfClf collisionAvoidanceController
    summary=struct('scope',"Prepared joint-certificate capture; capture-run timings excluded",'trials',{{}});
    for curvature=options.Curvatures
        for scenario=options.Scenarios
            folder=fullfile(directory,sprintf('%s-%g',scenario,curvature));
            standaloneControllerBenchmark.capture(fullfile(folder,'frames'),[]);
            try
                result=runExactStateRecursiveFeasibilityScenario(Scenario=scenario,RoadCurvature=curvature, ...
                    SampleCount=options.SampleCount,SampleTime=.05,HorizonSeconds=1.6, ...
                    DeadlineSeconds=30,SearchTimeLimitSeconds=30,OutputDirectory=folder);
            catch exception
                file=fullfile(folder,scenario+'-exact-state.mat');
                if ~isfile(file),rethrow(exception);end
                data=load(file,'report');result=data.report;
            end
            records=standaloneControllerBenchmark.capture([],[]);
            item=struct('scenario',scenario,'curvature',curvature,'completed',result.completed, ...
                'sampledCollisionFree',result.sampledCollisionFree,'executedHolds',result.executedHolds, ...
                'minimumSampledBodyGap',result.minimumSampledBodyGap,'failureMessage',result.failureMessage, ...
                'frames',{records});
            summary.trials{end+1}=item;
            fid=fopen(fullfile(directory,'capture.json'),'w');assert(fid>=0);
            fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);
        end
    end
    clear solveHardCbfClf collisionAvoidanceController
end
