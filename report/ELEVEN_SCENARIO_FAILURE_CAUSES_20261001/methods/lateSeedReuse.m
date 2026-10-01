output='/home/zai/.cache/collisionAvoidance/direct-two-stage-scenarios-20261001';diagnosis='/home/zai/.cache/collisionAvoidance/eleven-failure-causes-20261001';
cd(fullfile(output,'source'));addpath('controller','config','scripts','solver/controller');entries=struct([]);
for pair={ {8,"headOn"}, {8,"brakingLead"}, {15,"brakingLead"} }
    speed=pair{1}{1};name=pair{1}{2};
    data=load(fullfile(output,'campaign',"speed"+speed,name+".mat"),'continuation');c=data.continuation;
    [~,~,road,cfg]=collisionThreatScenario(name,c.result.configuration);
    entry=struct('speed',speed,'scenario',name,'returned',false,'initialization',"",'flags',[], ...
        'pcbfSlack',NaN,'clfSlack',NaN,'firstInput',[],'failure',"");
    try
        [command,~,p]=collisionAvoidanceController(c.ego,c.target,road,cfg,[]);
        entry.returned=true;entry.initialization=p.metadata.search.initialization;
        entry.flags=[p.metadata.search.stages.exitFlag];entry.pcbfSlack=p.solution.safety;
        entry.clfSlack=p.solution.clfSlack;entry.firstInput=command.actuatorInput;
    catch exception
        entry.failure=string(exception.identifier)+": "+string(exception.message);
    end
    if isempty(entries),entries=entry;else,entries(end+1)=entry;end
    fprintf('COLD-SEED %g %s returned=%d flags=[%s] %s\n',speed,name,entry.returned,num2str(entry.flags),entry.failure);
end
file=fopen(fullfile(diagnosis,'late-seed-reuse.json'),'w');fprintf(file,'%s\n',jsonencode(entries));fclose(file);
