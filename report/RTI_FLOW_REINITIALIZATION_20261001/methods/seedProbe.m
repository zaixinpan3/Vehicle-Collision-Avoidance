source='/home/zai/.cache/collisionAvoidance/rti-controller-20261001/source';
output='/home/zai/.cache/collisionAvoidance/rti-controller-20261001';
cd(source);addpath('controller','config','scripts');results=struct([]);
for speed=[8,15]
    prefix=8;if speed==15,prefix=16;end
    config=struct('referenceSpeed',speed,'controller',struct('horizonSteps',prefix));
    for name=["headOn","acceleratingHeadOn","brakingLead","crossing","turningCrossing","curvedHeadOn","curvedCrossing"]
        [x,q,road,cfg]=collisionThreatScenario(name,config);
        ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6));
        [~,lane,normalizedRoad]=readControllerInputs(ego,[],road,cfg);frame=predictiveSafetyGeometry.roadFrame(lane,normalizedRoad);
        model=struct('cfg',cfg,'initialState',x,'previousInput',[0;0],'target',q,'targetEpoch',q, ...
            'sampleIndex',0,'lane',lane,'frame',frame,'terminal',terminalContinuation.build(cfg,0), ...
            'nominalReference',nonlinearBicycleModel.cruise(cfg,frame(4)));
        [solution,search,model]=solvePredictiveControl(model,[]);
        anchor=model.linearization;count=size(anchor.inputs,2);shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
        gap=Inf;defect=0;firstCollision=NaN;
        for index=1:count
            next=nonlinearBicycleModel.sample(anchor.states(:,index),anchor.inputs(:,index),cfg);
            defect=max(defect,norm(next-anchor.states(:,index+1),inf));
            for fraction=[0,.5,1]
                state=nonlinearBicycleModel.sample(anchor.states(:,index),anchor.inputs(:,index),cfg,[],fraction*cfg.controller.sampleTime);
                time=(index-1+fraction)*cfg.controller.sampleTime;target=predictiveSafetyGeometry.targetFlow(q,time);
                d=predictiveSafetyGeometry.rectangle(state(1:3),shape,target(1:3),target(8:11));gap=min(gap,d);
                if d<=0 && isnan(firstCollision),firstCollision=time;end
            end
        end
        intrinsic=[anchor.states(4:6,end);anchor.inputs(:,end)]-[model.terminal.base(4:6);model.terminal.reference.input];
        ratio=norm(model.terminal.quotientFactor*intrinsic)/model.terminal.radius;
        entry=struct('speed',speed,'scenario',name,'minimumSampledClearance',gap,'firstCollisionTime',firstCollision, ...
            'maximumDynamicsDefect',defect,'terminalNormOverRadius',ratio,'firstSteering',anchor.inputs(1,1), ...
            'returned',~isempty(solution),'primaryOptimum',search.primaryOptimum,'flags',[search.stages.exitFlag]);
        if isempty(results),results=entry;else,results(end+1)=entry;end
        fprintf('SEED speed=%g %s gap=%.9g terminalRatio=%.4g returned=%d\n',speed,name,gap,ratio,~isempty(solution));
    end
end
f=fopen(fullfile(output,'seeds.json'),'w');fprintf(f,'%s\n',jsonencode(results));fclose(f);
