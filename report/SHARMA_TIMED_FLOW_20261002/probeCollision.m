function probeCollision(root,directory)
%probeCollision Inspect the fresh seed at the first positive-slack hold.
    cd(root);addpath('controller','config','scripts',fullfile(directory,'seed-audit'));
    data=jsondecode(fileread(fullfile(directory,'current-campaign','speed15-turningCrossing.json')));
    result=data.results;cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',15, ...
        'controller',struct('horizonSteps',16)));hold=result.trace(23);
    assert(isequal(cfg.vehicle,result.configuration.vehicle) && isequal(cfg.collision,result.configuration.collision));
    assert(abs(hold.time-1.1)<1e-12);
    [~,~,road]=collisionThreatScenario("turningCrossing",cfg);
    x=hold.state;ego=struct('position',x(1:2),'yaw',x(3),'longitudinalVelocity',x(4), ...
        'lateralVelocity',x(5),'yawRate',x(6));
    [~,lane,road]=readControllerInputs(ego,[],road,cfg);frame=predictiveSafetyGeometry.roadFrame(lane,road);
    epoch=result.targetInitialState;q=predictiveSafetyGeometry.targetFlow(epoch,hold.time);
    model=struct('cfg',cfg,'initialState',x,'previousInput',result.trace(22).input, ...
        'target',q,'targetEpoch',epoch,'sampleIndex',22,'lane',lane,'frame',frame, ...
        'terminal',terminalContinuation.build(cfg,0),'nominalReference',nonlinearBicycleModel.cruise(cfg,0));
    count=cfg.controller.horizonSteps+ceil(cfg.nonlinear.recoveryHorizonSeconds/cfg.controller.sampleTime);
    anchor=newFlowSeed(model,count);shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    entries=struct([]);
    for index=1:cfg.controller.horizonSteps
        midpoint=(anchor.states(:,index)+anchor.states(:,index+1))/2;
        for half=0:1
            x=anchor.states(:,index);if half,x=midpoint;end
            time=hold.time+(index-1+half/2)*cfg.controller.sampleTime;
            q=predictiveSafetyGeometry.targetFlow(epoch,time);
            gap=predictiveSafetyGeometry.rectangle(x(1:3),shape,q(1:3),q(8:11));
            if gap==0
                dual=predictiveSafetyGeometry.dualLinearization(x(1:3),shape,q(1:3),q(8:11));
                entries=[entries,struct('stage',index,'time',time,'pose',x(1:3),'target',q, ...
                    'distance',gap,'dualValue',dual.distance,'normalNorm',norm(dual.normal), ...
                    'maximumJacobianMagnitude',max(abs(dual.jacobian),[],'all'))]; %#ok<AGROW>
            end
        end
    end
    [solution,search]=solvePredictiveControl(model,[],tic);
    diagnosis=struct('holdTime',hold.time,'overlappingPrefixAnchors',entries, ...
        'coldReplaySolutionReturned',~isempty(solution),'coldReplaySearch',search, ...
        'scope','Fresh seed at recorded measured state/input; terminal core rebuilt from unchanged intrinsic configuration');
    fid=fopen(fullfile(directory,'collision-seed-probe.json'),'w');fprintf(fid,'%s\n',jsonencode(diagnosis));fclose(fid);
    save(fullfile(directory,'collision-seed-probe.mat'),'model','anchor','diagnosis');
end
