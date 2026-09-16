function summary = diagnoseDistanceDualInitialization(outputDirectory,options)
%diagnoseDistanceDualInitialization Audit first-admission geometry without changing control.
% Optional recorded input plans are offline feasibility witnesses, never an
% initializer used by the online controller or a stored certificate import.
    arguments
        outputDirectory (1,1) string
        options.WitnessDirectory (1,1) string = ""
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'), ...
        fullfile(root,'solver','clarabel','matlab'),fullfile(root,'solver','bicycle'));
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    summary=struct('matlabVersion',string(version),'cases',struct(), ...
        'scope',"Offline distance-dual and terminal-exit diagnosis; no online algorithm change or runtime claim");
    for scene=["stationary","oncoming"]
        [ego,target,route,cfg]=localFixture(scene);
        saved=[];
        if strlength(options.WitnessDirectory)>0
            saved=load(fullfile(options.WitnessDirectory,scene+'Admission.mat'),'current','baseline');
            ego=saved.current.ego;target=saved.current.target;route=saved.current.road;
        end
        model=localModel(ego,target,route,cfg);
        [prediction,anchor]=hardEncounterBarrier.predict(model,model.cruiseCertificate);
        [cells,raw]=localCells(model,prediction,anchor,true);
        inside=cells.signedDistance<0;strong=cells.signedDistance<-.1;
        relative=target.targetPositionInertial-ego.position;
        closing=ego.speed-target.targetVelocityInertial(1);
        overlapWindow=(relative(1)+[-1,1]*(cfg.vehicle.length+cfg.target.defaultLength)/2)/closing;
        initialGap=avoidanceSafetyGeometry.rectangleDistance(ego.position,ego.yaw, ...
            target.targetPositionInertial,target.targetHeadingInertial, ...
            [cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.target.defaultLength/2;cfg.target.defaultWidth/2]);
        item=struct('stateTime',ego.stateTime,'relativePosition',relative,'closingSpeed',closing, ...
            'initialBodyGap',initialGap,'horizonSeconds',prediction.stageCount*model.sampleTime, ...
            'anchorSteeringMaximum',max(abs(anchor(1:2:end))), ...
            'anchorLongitudinalInput',anchor(2),'overlapWindow',overlapWindow, ...
            'overlappingMidpoints',nnz(inside),'overlapMidpoints',cells.time(inside), ...
            'minimumSignedDistance',min(cells.signedDistance), ...
            'maximumStrongInteriorDualNorm',max(cells.rawNormalNorm(strong)), ...
            'maximumStrongInteriorObjectiveMagnitude',max(abs(cells.rawDualValue(strong))), ...
            'maximumStrongInteriorFaceResidual',max(cells.maximumFaceResidual(strong)));
        assert(all(cells.maximumFaceResidual(strong)<0));
        assert(all(ismember(cells.rawStatus,[1,4])),'A raw distance solve failed.');
        assert(item.maximumStrongInteriorDualNorm<1e-5);
        assert(item.maximumStrongInteriorObjectiveMagnitude<1e-5);
        item.freshAdmission=localAdmissionFailure(model);
        horizons=localHorizons(model);
        item.horizons=table2struct(horizons);
        item.recordedWitness=localWitness(model,saved);
        summary.cases.(scene)=item;
        writetable(cells,fullfile(outputDirectory,scene+'-midpoints.csv'));
        writetable(horizons,fullfile(outputDirectory,scene+'-horizons.csv'));
        save(fullfile(outputDirectory,scene+'-diagnosis.mat'),'model','prediction','anchor','cells','raw','item');
        fprintf('%s: %d overlap midpoints, first %.3f s, last %.3f s; raw interior norm %.3g; recorded plan provided %d, recheck %d\n', ...
            scene,nnz(inside),min(cells.time(inside)),max(cells.time(inside)), ...
            item.maximumStrongInteriorDualNorm,item.recordedWitness.provided,item.recordedWitness.hardCertified);
    end
    summary.pointPerturbation=localPointPerturbation(cfg);
    save(fullfile(outputDirectory,'summary.mat'),'summary');
    file=fopen(fullfile(outputDirectory,'summary.json'),'w');assert(file>=0);
    cleanup=onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(summary,PrettyPrint=true));
end

function result=localAdmissionFailure(model)
    result=struct('failed',false,'identifier',"",'message',"");
    try
        formulateAvoidanceProblem(model);
    catch exception
        result.failed=true;result.identifier=string(exception.identifier);
        result.message=string(exception.message);
    end
    assert(result.failed && result.identifier=="collisionAvoidanceController:optimizationFailed" ...
        && contains(result.message,"Distance-dual initialization is unavailable"), ...
        'Fresh admission did not reproduce the diagnosed initialization failure.');
end

function [ego,target,route,cfg]=localFixture(scene)
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'controller',struct('sampleTime',.1),'model',struct('lateralDomainRadius',4), ...
        'solver',struct('frameDeadlineSeconds',60)));
    ego=struct('position',[0;0],'yaw',0,'speed',8,'stateTime',0, ...
        'perception',struct('time',0,'range',16,'completeWithinRange',true));
    route=[-100,0;2000,0];
    position=[15;0];velocity=[0;0];heading=0;
    if scene=="oncoming"
        % Reproduce the actual target-free prefix using the current controller.
        stored=[];
        for step=1:26
            [~,~,problem,stored]=collisionAvoidanceController(ego,[],route,cfg,stored);
            x=stored.predictedState(:,2);
            [ego.position,ego.yaw]=laneGeometry.fromFrenet(x,problem.model.lane);
            ego.speed=x(4);ego.lateralVelocity=x(5);ego.yawRate=x(6);
            ego.stateTime=step*cfg.controller.sampleTime;ego.perception.time=ego.stateTime;
            ego.heldActuatorInput=stored.appliedInput;
        end
        position=[60-8*ego.stateTime;0];velocity=[-8;0];heading=pi;
    end
    target=struct('trackId',1,'targetPositionInertial',position,'targetVelocityInertial',velocity, ...
        'targetAccelerationInertial',[0;0],'targetHeadingInertial',heading,'targetYawRate',0, ...
        'predictionMotion',struct('kind',"finite-sensing-motion-v1", ...
        'jerkBound',[0;0],'yawAccelerationBound',0));
end

function model=localModel(egoInput,target,route,cfg)
% Assemble the public model inputs for an offline fresh-admission inspection.
    [ego,lane,road,observations]=readPlanningInputs(egoInput,target,route,cfg);
    projection=laneGeometry.project(ego.position,lane);
    heading=atan2(sin(ego.yaw-projection.heading),cos(ego.yaw-projection.heading));
    radius=stateUncertainty.toFrenet(ego.modelState,ego.stateErrorBound,lane);
    input=zeros(2,1);if ~isempty(ego.heldActuatorInput),input=ego.heldActuatorInput;end
    model=struct('cfg',cfg,'lane',lane,'road',road,'stateTime',ego.stateTime, ...
        'sampleTime',cfg.controller.sampleTime,'horizonSteps',cfg.controller.horizonSteps, ...
        'referenceSpeed',cfg.referenceSpeed, ...
        'initialEgoState',[projection.station;projection.lateralPosition;heading;ego.modelState(4:6)], ...
        'initialFrenetErrorBound',radius,'longitudinalAccelerationBias',ego.longitudinalAccelerationBias, ...
        'previousInput',input,'requiredMargin',0,'confirmation',[]);
    model=hardEncounterBarrier.prepare(model,ego,observations,[],struct());
    model.cruiseCertificate=ltvBicycleModel.sampledCruise(model);
end

function [rows,raw]=localCells(model,prediction,anchor,solveRaw)
    cfg=model.cfg;count=numel(prediction.cells);
    [frames,~]=laneGeometry.sweptCellFrames(model,prediction.cells,anchor);
    data=zeros(count,9);raw=cell(count,1);
    target=model.encounters(1);
    dimensions=[cfg.vehicle.length/2;cfg.vehicle.width/2;target.halfLength;target.halfWidth];
    for index=1:count
        tube=prediction.cells(index);frame=frames(index);degree=size(tube.offset,2)-1;
        weights=arrayfun(@(k)nchoosek(degree,k),0:degree).'/2^degree;
        z=(reshape(pagemtimes(tube.map,anchor),6,[])+tube.offset)*weights;
        position=frame.origin+[frame.tangent,frame.lateral]*z(1:2);
        yaw=frame.heading+z(3);time=tube.start+tube.duration/2;
        center=targetPrediction.finiteFlow(target,time);
        distance=avoidanceSafetyGeometry.rectangleDistance(position,yaw,center(1:2),center(7),dimensions);
        data(index,1:4)=[time,position.'-center(1:2).',distance];
        if solveRaw
            [~,dual]=avoidanceSafetyGeometry.distanceDual(position,yaw,center(1:2),center(7),dimensions,cfg);
            residual=dual.matrix*position-dual.bound;
            % Evaluate the SAME ordinary-distance dual even inside, bypassing
            % only the production overlap short-circuit in this offline audit.
            [lambda,status]=solveAvoidanceSocpMex(sparse(8,8),-residual, ...
                sparse([-eye(8);zeros(1,8);-dual.matrix.']),[zeros(8,1);1;0;0],[0;8;3], ...
                [cfg.solver.constraintTolerance,cfg.solver.optimalityTolerance,cfg.solver.maxIterations]);
            data(index,5:9)=[max(residual),residual.'*lambda,norm(dual.matrix.'*lambda),status.status,dual.available];
            raw{index}=struct('matrix',dual.matrix,'bound',dual.bound,'residual',residual,'multipliers',lambda);
        end
    end
    rows=array2table(data,VariableNames={'time','relativeX','relativeY','signedDistance', ...
        'maximumFaceResidual','rawDualValue','rawNormalNorm','rawStatus','productionDirectionAvailable'});
end

function rows=localHorizons(model)
    counts=[4,8,9,12,13,16,24,32,48,64];data=zeros(numel(counts),4);cfg=model.cfg;
    for index=1:numel(counts)
        trial=model;trial.horizonSteps=counts(index);
        [prediction,anchor]=hardEncounterBarrier.predict(trial,trial.cruiseCertificate);
        cells=localCells(trial,prediction,anchor,false);trial.anchorPlan=anchor;
        [matrix,bound]=hardEncounterBarrier.completionRows(trial,prediction,[]);
        row=matrix(end,:);limit=bound(end);
        lower=repmat([-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum],counts(index),1);
        upper=repmat([cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum],counts(index),1);
        best=min(row,0)*upper+max(row,0)*lower;
        data(index,:)=[counts(index),counts(index)*model.sampleTime,nnz(cells.signedDistance<0),limit-best];
    end
    rows=array2table(data,VariableNames={'holds','horizonSeconds','overlappingMidpoints','bestExitRowMarginOverInputBox'});
end

function result=localWitness(model,saved)
    result=struct('provided',~isempty(saved),'hardCertified',false,'trajectoryFeasible',false, ...
        'optimizedHardCertified',false,'minimumLinearMargin',NaN,'minimumTerminalMargin',NaN,'message',"");
    if isempty(saved),return;end
    inputs=reshape(saved.baseline(1:end-1),2,[]);
    assert(size(inputs,2)==model.horizonSteps,'Recorded input horizon does not match.');
    trial=model;trial.initializationPlan=inputs;
    try
        [program,~,~]=formulateAvoidanceProblem(trial);
        witness=[inputs(:);0];first=program.cones(2)+1;
        value=program.b(first:first+5)-program.A(first:first+5,:)*witness;
        witness(end)=max(0,norm(value(2:end))-value(1))+1;
        solveHardCbfClf.certify(program,witness);
        result.hardCertified=true;
        result.minimumLinearMargin=min(program.physicalBound-program.physicalMatrix*witness);
        terminal=reshape(program.terminalConePhysicalBound-program.terminalCone.matrix*inputs(:),3,[]);
        result.minimumTerminalMargin=min(terminal(1,:)-vecnorm(terminal(2:3,:),2,1));
        solve=solveHardCbfClf.constrained(program,model.cfg);result.trajectoryFeasible=solve.feasible;
        if solve.feasible
            solveHardCbfClf.certify(program,solve.decision);result.optimizedHardCertified=true;
        end
        result.message=solve.message;
    catch exception
        result.message=string(exception.message);
    end
end

function rows=localPointPerturbation(cfg)
% Pure geometry only: these offsets are not admissible initial ego states or a plan.
    offsets=[0,.001,.1,.5,1,1.8,2,2.2];data=zeros(numel(offsets),4);
    for index=1:numel(offsets)
        [normal,dual]=avoidanceSafetyGeometry.distanceDual([0;offsets(index)],0,[0;0],0,[2.4;.95;2.4;.95],cfg);
        data(index,:)=[offsets(index),dual.signedDistance,dual.distance,norm(normal)];
    end
    rows=table2struct(array2table(data,VariableNames={'lateralOffset','signedDistance','ordinaryDistance','normalNorm'}));
end
