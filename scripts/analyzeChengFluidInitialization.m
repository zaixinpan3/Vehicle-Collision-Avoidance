function summary=analyzeChengFluidInitialization(directory,options)
%analyzeChengFluidInitialization Offline VFFM-inspired admission experiment.
% Cheng et al. (2021), DOI 10.1109/TITS.2020.2990211, Eqs. (34)-(36),
% imply a Gaussian lateral path through a velocity-extremum condition.
% Predicting its conflict location, lifting it to a curved Frenet chart,
% fitting affine dynamics and choosing separation normals are project extensions.
% This driver neither changes production initialization nor issues commands.
    arguments
        directory (1,1) string
        options.WidthScales (1,:) double {mustBePositive,mustBeFinite} = [.8,1,1.2,1.5,2]
        options.Curvatures (1,:) double {mustBeFinite} = [-.012,-.01,-.008,.008,.01,.012]
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'), ...
        fullfile(root,'tests'),fullfile(root,'solver','bicycle'),fullfile(root,'solver','clarabel','matlab'));
    if ~isfolder(directory),mkdir(directory);end
    summary=struct('scope',"Offline initial admission only; no runtime or closed-loop claim", ...
        'matlabVersion',string(version),'cases',{{}});
    artifacts=cell(0,1);
    for curvature=options.Curvatures
        assert(curvature~=0,'Use the mirrored circular-crossing fixtures in this study.');
        [ego,target,road,cfg]=encounterTestFixture.circularCrossing(curvature);
        program=localProgram(ego,target,road,cfg);
        nominal=localStates(program,program.anchorPlan);
        records=program.jointCertificate.records;
        residual=avoidanceSafetyGeometry.jointResidual(program,program.feasibleWitness,program.jointCertificate.angles);
        selected=find(~[records.isExit].' & residual>program.jointCertificate.upperBound);
        assert(~isempty(selected),'The study expects an active nominal conflict.');
        stages=[records(selected).stage];center=(min(stages)+max(stages))/2;
        station=interp1(0:size(nominal,2)-1,nominal(1,:),center);
        width=max(3,(nominal(1,max(stages)+1)-nominal(1,min(stages)+1))/2);
        % This transverse-target fixture projects its length onto road lateral.
        % The 0.2 m is a seed-shape allowance, not a new hard clearance margin.
        amplitudeMagnitude=cfg.vehicle.width/2+records(selected(1)).targetHalfSize(1)+.2;
        for widthScale=options.WidthScales
            actualWidth=width*widthScale;
            for side=[-1,1]
                amplitude=side*amplitudeMagnitude;
                desired=nominal;difference=nominal(1,:)-station;
                bump=amplitude*exp(-.5*(difference/actualWidth).^2);
                derivative=-difference/actualWidth^2.*bump;
                desired(2,:)=nominal(2,:)+bump;
                desired(3,:)=nominal(3,:)+atan2(derivative,1-curvature*bump);
                [point,terminalFitError]=localFitPlan(program,nominal,desired,cfg);
                states=localStates(program,point(program.layout.planIndex));
                angles=zeros(numel(records),1);
                for index=1:numel(records)
                    item=records(index);state=states(:,item.stage+1);
                    relative=item.positionOffset+item.positionMap*state;
                    yaw=item.yawOffset+item.yawRow*state;
                    normal=avoidanceSafetyGeometry.supportDirection(relative,yaw,[0;0],item.targetYaw, ...
                        [item.egoHalfSize;item.targetHalfSize]);
                    angles(index)=atan2(normal(2),normal(1));
                end
                conic=avoidanceStageQp.fixedDirections(program,point,angles,cfg);
                solve=solveHardCbfClf.constrained(conic,cfg);
                accepted=false;message=solve.message;decision=[];
                maximumSupport=NaN;maximumPhysical=NaN;
                if solve.feasible
                    decision=solve.decision(1:numel(program.q));accepted=true;
                    maximumSupport=max(avoidanceSafetyGeometry.jointResidual(program,decision,angles));
                    maximumPhysical=max(program.physicalMatrix*decision-program.physicalBound);
                end
                item=struct('curvature',curvature,'side',side,'amplitudeMeters',amplitude, ...
                    'widthMeters',actualWidth,'widthScale',widthScale,'centerStationMeters',station, ...
                    'conflictStages',[min(stages),max(stages)],'horizonSteps',program.prediction.stageCount, ...
                    'sampleTimeSeconds',cfg.controller.sampleTime,'seedTerminalFitError',terminalFitError, ...
                    'seedMaximumPhysicalExcess',max(program.physicalMatrix*point-program.physicalBound), ...
                    'seedMaximumSupportResidual',max(avoidanceSafetyGeometry.jointResidual(program,point,angles)), ...
                    'accepted',accepted,'message',message,'solverExitFlag',solve.exitFlag, ...
                    'maximumSupportResidual',maximumSupport,'maximumPhysicalResidual',maximumPhysical);
                summary.cases{end+1}=item;
                artifacts{end+1,1}=struct('program',program,'seed',point,'reference',desired,'angles',angles, ...
                    'decision',decision,'configuration',cfg,'ego',ego,'target',target,'road',road); %#ok<AGROW>
                fprintf('k=%g widthScale=%g side=%g accepted=%d seedExcess=%g\n', ...
                    curvature,widthScale,side,accepted,item.seedMaximumPhysicalExcess);
            end
        end
    end
    save(fullfile(directory,'admission-study.mat'),'summary','artifacts','-v7.3');
    file=fopen(fullfile(directory,'admission-study.json'),'w');assert(file>=0);
    cleanup=onCleanup(@()fclose(file));fprintf(file,'%s\n',jsonencode(summary,PrettyPrint=true));
end

function program=localProgram(egoInput,target,roadInput,cfg)
% Form the same initial problem without running the production initializer.
    [ego,lane,road,observations]=readPlanningInputs(egoInput,target,roadInput,cfg);
    projection=laneGeometry.project(ego.position,lane);
    heading=atan2(sin(ego.yaw-projection.heading),cos(ego.yaw-projection.heading));
    radius=stateUncertainty.toFrenet(ego.modelState,ego.stateErrorBound,lane);
    model=struct('cfg',cfg,'lane',lane,'road',road,'stateTime',ego.stateTime, ...
        'sampleTime',cfg.controller.sampleTime,'horizonSteps',cfg.controller.horizonSteps, ...
        'referenceSpeed',cfg.referenceSpeed, ...
        'initialEgoState',[projection.station;projection.lateralPosition;heading;ego.modelState(4:6)], ...
        'initialFrenetErrorBound',radius,'longitudinalAccelerationBias',ego.longitudinalAccelerationBias, ...
        'previousInput',zeros(2,1),'requiredMargin',0,'confirmation',[]);
    identity=struct('configuration',rmfield(cfg,'solver'),'lane',lane,'road',road, ...
        'accelerationBias',ego.longitudinalAccelerationBias);
    model=hardEncounterBarrier.prepare(model,ego,observations,[],identity);
    program=formulateAvoidanceProblem(model);
end

function states=localStates(program,plan)
    states=program.prediction.egoStateOffset+reshape(pagemtimes(program.prediction.egoStateMatrix,plan),6,[]);
end

function [point,terminalError]=localFitPlan(program,nominal,desired,cfg)
% Equality-constrained least squares matches the reference to the affine plant.
% It does not enforce every hard constraint; only the final verifier can admit.
    maps=program.prediction.egoStateMatrix;count=program.layout.planCount;
    rows=[2,3];weights=[1;4];
    map=reshape(permute(maps(rows,:,2:end).*reshape(weights,[],1,1),[1,3,2]),[],count);
    error=reshape(weights.*(desired(rows,2:end)-nominal(rows,2:end)),[],1);
    difference=speye(count)-spdiags(ones(count,1),-2,count,count);
    metric=diag(program.inputWeight)+cfg.encounter.inputRateWeight/cfg.controller.sampleTime^2*(difference.'*difference);
    hessian=map.'*map+.01*metric;
    constraint=[maps(:,:,end);zeros(2,count)];constraint(end-1:end,end-1:end)=eye(2);
    scale=max(vecnorm(constraint,2,2),eps);scaled=constraint./scale;
    inverse=hessian\[map.'*error,scaled.'];
    change=inverse(:,1)-inverse(:,2:end)*(pinv(scaled*inverse(:,2:end))*(scaled*inverse(:,1)));
    terminalError=norm(constraint*change,Inf);assert(terminalError<1e-8);
    plan=program.anchorPlan+change;
    first=program.cones(2)+1;clf=program.b(first:first+5)-program.A(first:first+5,:)*[plan;0];
    point=[plan;max(0,norm(clf(2:end))-clf(1))+1e-8];
end
