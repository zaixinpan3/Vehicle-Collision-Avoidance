function summary=prepareStandaloneControllerReplay(captureDirectory,buildDirectory,options)
%prepareStandaloneControllerReplay Export identical prepared inputs and MATLAB timings.
% File I/O and serialization occur outside every measured numerical invocation.
    arguments
        captureDirectory (1,1) string
        buildDirectory (1,1) string
        options.Repetitions (1,1) double = 5
        options.Warmups (1,1) double = 2
    end
    addpath(buildDirectory);
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'solver','clarabel','matlab'));
    oldThreads=maxNumCompThreads(1);cleanup=onCleanup(@()maxNumCompThreads(oldThreads));
    capture=jsondecode(fileread(fullfile(captureDirectory,'capture.json')));
    calibration=load(fullfile(buildDirectory,'configuration.mat'),'c');
    summary=struct('scope',"Prepared joint-certificate numerical frame; preparation and I/O excluded", ...
        'repetitions',options.Repetitions,'warmups',options.Warmups,'matlabVersion',string(version),'frames',{{}});
    for trialIndex=1:numel(capture.trials)
        trial=capture.trials(trialIndex);
        for frameIndex=1:numel(trial.frames)
            item=trial.frames(frameIndex);data=load(item.file,'p','c');p=data.p;c=data.c;
            assert(isequal(c,calibration.c),'Compiled configuration does not match the recorded frame.');
            assert(~p.terminalOptimization,'This native entry excludes the distinct terminal-optimization mode.');
            binary=replace(string(item.file),'.mat','.bin');writeStandaloneFixture(binary,p);
            for warmup=1:options.Warmups,standaloneControllerFrame(p,c);end
            seconds=zeros(options.Repetitions,1);phaseSeconds=zeros(options.Repetitions,4);
            for repetition=1:options.Repetitions
                timer=tic;[decision,angles,status,metrics]=standaloneControllerFrame(p,c);
                seconds(repetition)=toc(timer);phaseSeconds(repetition,:)=metrics.';
            end
            record=struct('scenario',trial.scenario,'curvature',trial.curvature, ...
                'frame',frameIndex,'file',binary,'matFile',item.file,'inherited',item.inherited, ...
                'horizonSteps',item.horizonSteps,'status',status,'seconds',seconds, ...
                'metrics',phaseSeconds,'decision',decision,'angles',angles);
            summary.frames{end+1}=record;
        end
        fid=fopen(fullfile(captureDirectory,'matlab-replay.json'),'w');assert(fid>=0);
        fprintf(fid,'%s\n',jsonencode(summary));fclose(fid);
        fprintf('Prepared %s k=%g: %d frames\n',trial.scenario,trial.curvature,numel(trial.frames));
    end
end
