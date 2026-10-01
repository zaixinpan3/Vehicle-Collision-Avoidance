root='/home/zai/Downloads/ResearchProjects/collisionAvoidance';cd(root);addpath('controller','config');entries=struct([]);
for lateral=[.1,1]
    for yaw=[.5,1]
        cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8,'controller',struct('horizonSteps',8),'nonlinear',struct('trustRadius',1)));
        ego=struct('position',[0;lateral],'yaw',yaw,'speed',8,'lateralVelocity',0,'yawRate',0);
        road=struct('centerline',[-100,0;1000,0]);
        try
            [command,inputs,p,state]=collisionAvoidanceController(ego,[],road,cfg,[]);u=command.actuatorInput;
            entry=struct('lateral',lateral,'yaw',yaw,'u',u,'maxSteering',max(abs(inputs(1,:))),'anchorU',p.model.linearization.inputs(:,1),'rho',p.solution.clfSlack,'flags',[p.metadata.search.stages.exitFlag]);entries=[entries,entry];
            fprintf('PROBE y=%.2f yaw=%.2f u=[%.7g %.7g] maxSteer=%.7g anchor=[%.7g %.7g] rho=%.7g flags=[%s]\n',lateral,yaw,u(1),u(2),entry.maxSteering,entry.anchorU(1),entry.anchorU(2),entry.rho,num2str(entry.flags));
        catch e,fprintf('PROBE y=%.2f yaw=%.2f FAILED %s\n',lateral,yaw,e.message);end
    end
end
file=fopen('/home/zai/.cache/collisionAvoidance/unbounded-steering-20261001/wide-state-probe.json','w');fprintf(file,'%s\n',jsonencode(entries));fclose(file);
