function runEncounterQueue(sourceRoot,outputDirectory,worker,workers,frameCount)
%runEncounterQueue This worker's share of the 98 campaign encounters, in one MATLAB session.
% The encounters (seeds 0, 20261003, 1..5 x speeds 8, 15 x seven scenarios,
% seed 0 = exact states) are dealt round robin to 'workers' workers; an
% encounter whose summary exists is skipped, so a stopped worker can be
% restarted. One session per worker avoids a licensing-service start per
% encounter. Each encounter runs through runOneEncounter.m.
    seeds=[0,20261003,1,2,3,4,5];speeds=[8,15];
    names=["headOn","acceleratingHeadOn","brakingLead","crossing","turningCrossing","curvedHeadOn","curvedCrossing"];
    [n,s,e]=ndgrid(1:numel(names),1:numel(speeds),1:numel(seeds));
    here=fileparts(mfilename('fullpath'));
    for j=worker:workers:numel(n)
        seed=seeds(e(j));speed=speeds(s(j));name=names(n(j));
        stem=fullfile(outputDirectory,sprintf('seed%d-speed%g-%s',seed,speed,name));
        if isfile(stem+"-summary.json"),continue;end
        fprintf('ENCOUNTER %s\n',stem);
        addpath(here);
        try
            runOneEncounter(sourceRoot,outputDirectory,speed,name,seed,frameCount,struct());
        catch exception
            fprintf('ENCOUNTER_ERROR %s: %s\n',stem,exception.message);
        end
        close all force;
    end
    fprintf('QUEUE_DONE %d\n',worker);
end
