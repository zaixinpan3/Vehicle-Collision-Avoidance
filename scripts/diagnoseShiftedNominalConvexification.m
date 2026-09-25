function results=diagnoseShiftedNominalConvexification(options)
%diagnoseShiftedNominalConvexification Offline fixed-normal feasibility audit.
% Removing constraint groups is diagnostic only; no control is issued.
% These are fresh-admission fixtures, including a straight oncoming target
% at 18.4 m relative distance. They do not replay a stored controller state.
    arguments
        options.OutputFile (1,1) string = ""
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'solver/bicycle'));
    scenarios=["stationary","stationary","crossing","oncoming"];
    curvature=[0,.01,.01,0];results=cell(4,1);
    for index=1:4
        cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
            'controller',struct('sampleTime',.1,'minimumHorizonSteps',1), ...
            'solver',struct('frameDeadlineSeconds',5)));
        k=curvature(index);curve=struct('origin',[0;0],'heading',0,'curvature',k,'length',150);
        road=struct('referenceCurve',curve,'centerline',laneGeometry.referencePose(0:2:150,0,curve).');
        z=ltvBicycleModel.cruiseEquilibrium(k,cfg);
        [position,yaw]=laneGeometry.referencePose(15,0,curve);velocity=[0;0];
        if scenarios(index)=="crossing"
            normal=[-sin(yaw);cos(yaw)];position=position-7.5*normal;velocity=4*normal;yaw=yaw+pi/2;
        elseif scenarios(index)=="oncoming"
            position=[18.4;0];velocity=[-8;0];yaw=pi;
        end
        target=struct('trackId',1,'targetPositionInertial',position,'targetVelocityInertial',velocity, ...
            'targetAccelerationInertial',[0;0],'targetHeadingInertial',yaw,'targetYawRate',0, ...
            'predictionMotion',struct('kind','nrmm-motion-v1','curvatureMaximum',0.05));
        measurement=struct('position',[0;0],'yaw',z(3),'speed',z(4),'lateralVelocity',z(5),'yawRate',z(6), ...
            'stateTime',0,'perception',struct('time',0,'range',16,'completeWithinRange',true));
        [ego,lane,parsedRoad,observations]=readPlanningInputs(measurement,target,road,cfg);
        model=struct('cfg',cfg,'lane',lane,'road',parsedRoad,'stateTime',0,'sampleTime',.1, ...
            'horizonSteps',16,'referenceSpeed',8,'initialEgoState',z,'initialFrenetErrorBound',zeros(6,1), ...
            'longitudinalAccelerationBias',0,'previousInput',[0;0],'requiredMargin',0,'confirmation',[]);
        [model,~]=hardEncounterBarrier.prepare(model,ego,observations,[],struct());
        [program,prediction]=formulateAvoidanceProblem(model);
        names=["full","withoutCollision","withoutExit","withoutTerminal","withoutPoseDomains"];
        labels=program.physicalLabels;status=zeros(1,numel(names));residual=status;
        for mode=1:numel(names)
            remove=false(numel(labels),1);dropCone=false;
            if mode==2,remove=startsWith(labels,"collision:");end
            if mode==3,remove=startsWith(labels,"exit:");end
            if mode==4,remove=startsWith(labels,"terminal") | startsWith(labels,"exit:");dropCone=true;end
            if mode==5,remove=contains(labels,"Domain");end
            trial=rmfield(program,'prediction');keep=true(size(program.b));keep(1:numel(remove))=~remove;
            if dropCone,keep(end-sum(program.terminalCone.sizes)+1:end)=false;trial.cones=trial.cones(1:3);end
            trial.A=program.A(keep,:);trial.b=program.b(keep);trial.cones(2)=program.cones(2)-nnz(remove);
            solved=solveHardCbfClf.constrained(trial,cfg);status(mode)=solved.exitFlag;
            residual(mode)=Inf;
            if solved.feasible
                value=trial.b-trial.A*solved.decision;cursor=trial.cones(2);residual(mode)=-min(value(1:cursor));
                for dimension=trial.cones(3:end).'
                    cone=value(cursor+(1:dimension));cursor=cursor+dimension;
                    residual(mode)=max(residual(mode),norm(cone(2:end))-cone(1));
                end
            end
        end
        normals=cell2mat(program.geometry.normals.');
        results{index}=struct('scenario',scenarios(index),'curvature',k,'horizon',prediction.stageCount, ...
            'overlappingNominalNodes',program.supportGeometry.overlappingNodes,'nominalMinimumDistance',program.supportGeometry.minimumAnchorDistance, ...
            'normalSwitches',program.supportGeometry.normalSwitchCount,'normalSequence',normals, ...
            'ablations',names,'solverExitFlags',status,'maximumConicResidual',residual);
    end
    if strlength(options.OutputFile)>0
        file=fopen(options.OutputFile,'w');assert(file>=0);
        cleanup=onCleanup(@()fclose(file));
        fprintf(file,'%s\n',jsonencode(results,PrettyPrint=true));
    end
end
