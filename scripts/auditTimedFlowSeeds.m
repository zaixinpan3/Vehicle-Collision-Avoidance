function rows=auditTimedFlowSeeds(root,baselineRoot,outputDirectory)
%auditTimedFlowSeeds Compare exact initialization helpers without changing the solver.
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'));
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    extract(fullfile(root,'controller','solvePredictiveControl.m'),outputDirectory,'newFlowSeed');
    extract(fullfile(baselineRoot,'controller','solvePredictiveControl.m'),outputDirectory,'oldFlowSeed');
    addpath(outputDirectory);cleanup=onCleanup(@()rmpath(outputDirectory)); %#ok<NASGU>
    names=["headOn","acceleratingHeadOn","brakingLead","crossing","turningCrossing","curvedHeadOn","curvedCrossing"];
    rows=struct([]);
    for speed=[8,15]
        for name=names
            [x,target,road,cfg]=collisionThreatScenario(name,struct('referenceSpeed',speed, ...
                'controller',struct('horizonSteps',8+8*(speed==15))));
            ego=struct('position',x(1:2),'yaw',x(3),'longitudinalVelocity',x(4),'lateralVelocity',x(5),'yawRate',x(6));
            [~,lane,road]=readControllerInputs(ego,[],road,cfg);
            frame=predictiveSafetyGeometry.roadFrame(lane,road);
            model=struct('cfg',cfg,'initialState',x,'previousInput',[0;0],'target',target,'targetEpoch',target, ...
                'sampleIndex',0,'lane',lane,'terminal',terminalContinuation.build(cfg,0), ...
                'nominalReference',nonlinearBicycleModel.cruise(cfg,frame(4)));
            count=cfg.controller.horizonSteps+ceil(cfg.nonlinear.recoveryHorizonSeconds/cfg.controller.sampleTime);
            before=oldFlowSeed(model,count);after=newFlowSeed(model,count);
            guide=predictiveSafetyGeometry.movingGaussianGuide(x,lane,target,0,count*cfg.controller.sampleTime,cfg);
            elapsed=zeros(2,5);
            for trial=1:5
                timer=tic;oldFlowSeed(model,count);elapsed(1,trial)=toc(timer);
                timer=tic;newFlowSeed(model,count);elapsed(2,trial)=toc(timer);
            end
            row=struct('speed',speed,'scenario',name,'oldNodeGap',gap(before,model), ...
                'newNodeGap',gap(after,model),'oldMedianSeconds',median(elapsed(1,:)), ...
                'newMedianSeconds',median(elapsed(2,:)),'guide',guide);
            rows=[rows,row]; %#ok<AGROW>
            save(fullfile(outputDirectory,sprintf('seed-%g-%s.mat',speed,name)),'model','before','after','guide','elapsed');
        end
    end
    fid=fopen(fullfile(outputDirectory,'seeds.json'),'w');fprintf(fid,'%s\n',jsonencode(rows));fclose(fid);
end

function extract(source,directory,name)
    code=fileread(source);result="";
    for helper=["localFlowSeed","localShape","localClip","localBeyondRange","localTargetAt"]
        first=regexp(code,"(?m)^function[^\n]*\<"+helper+"\(",'start','once');
        starts=regexp(code,'(?m)^function','start');next=starts(find(starts>first,1));
        if isempty(next),next=numel(code)+1;end
        part=string(code(first:next-1));
        if helper=="localFlowSeed",part=replace(part,"localFlowSeed(",name+"(");end
        result=result+part+newline;
    end
    fid=fopen(fullfile(directory,name+".m"),'w');fprintf(fid,'%s',result);fclose(fid);
end

function minimum=gap(anchor,model)
    minimum=Inf;cfg=model.cfg;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    for index=1:size(anchor.states,2)
        q=predictiveSafetyGeometry.targetFlow(model.targetEpoch,(model.sampleIndex+index-1)*cfg.controller.sampleTime);
        minimum=min(minimum,predictiveSafetyGeometry.rectangle(anchor.states(1:3,index),shape,q(1:3),q(8:11)));
    end
end
