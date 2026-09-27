function summary = auditInputDeviationPenaltyExperiment(outputDirectory)
%auditInputDeviationPenaltyExperiment Replay issued inputs and audit tire tangents.
% This offline replay issues no new plant trajectory. Tire errors compare the
% affine and nonlinear controller models, not measured vehicle tire forces.
    arguments
        outputDirectory (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'));
    planDirectory=fullfile(outputDirectory,'audit-plans');
    if ~isfolder(planDirectory),mkdir(planDirectory);end
    files=dir(fullfile(outputDirectory,'*_w*_t*.mat'));
    summary=struct([]);
    for fileIndex=1:numel(files)
        data=load(fullfile(files(fileIndex).folder,files(fileIndex).name));r=data.result;
        clear ltvBicycleModel;
        stored=[];rows=struct([]);failureTime=NaN;failureIdentifier="";commandDifference=0;
        cfg=r.controllerConfiguration;
        scale=[cfg.model.frontWheelSteeringAngleMaximum; ...
            max(abs([cfg.actuation.brakingRatioMinimum,cfg.actuation.brakingRatioMaximum]))];
        tire=modifiedFialaTire.parameters(cfg);
        for k=1:numel(r.attempts.time)
            ego=r.attempts.controllerEgoEstimate{k};
            if ~isempty(stored),ego.heldActuatorInput=stored.appliedInput;end
            road=r.attempts.roadPerception{k}.roadGeometry;
            if isfield(r.scenario.geometry,'referenceCurve'),road.referenceCurve=r.scenario.geometry.referenceCurve;end
            try
                [command,~,problem,stored]=collisionAvoidanceController( ...
                    ego,r.attempts.targetEstimate{k},road,cfg,stored);
            catch exception
                failureTime=r.attempts.time(k);failureIdentifier=string(exception.identifier);
                break;
            end
            commandDifference=max(commandDifference,norm(command.actuatorInput-r.command{k}.actuatorInput,Inf));
            inputs=problem.inputPlan;center=reshape(problem.program.inputDeviationCenter,2,[]);
            deviation=(inputs-center)./scale;count=size(inputs,2);
            search=problem.metadata.admissionSearch;
            refreshes=0;anchorFailure="";anchorError=NaN;
            if isfield(search,'linearizationRefreshes'),refreshes=search.linearizationRefreshes;end
            if isfield(search,'previousAnchorFailure'),anchorFailure=search.previousAnchorFailure;end
            if problem.metadata.nominalSource=="flowTrajectory"
                reference=problem.model.initializationPlan;
                expected=ltvBicycleModel.nominalRollout(problem.model,reference);
                anchorError=max(abs(expected-problem.prediction.linearizationStates),[],'all');
                assert(isequal(reference,problem.prediction.linearizationInputs) && anchorError<1e-10, ...
                    'Flow reference and actual model linearization disagree.');
                operating=cell2mat(cellfun(@(t)t.operatingInput, ...
                    problem.prediction.tireModels.',UniformOutput=false));
                assert(isequal(center,operating),'Deviation cost lost the rebuilt tire operating inputs.');
            end
            mismatch=nan(2,count);excess=zeros(2,count);invalid=0;
            for stage=1:count
                state=problem.predictedState(:,stage);input=inputs(:,stage);
                model=problem.prediction.tireModels{stage};
                affine=model.state*state+model.input*input+model.constant;
                capacity=tire.longitudinalForceScale*sqrt(max(0,1-input(2)^2));
                excess(:,stage)=max(0,abs(affine)-capacity);
                slip=atan2(state(5)+[cfg.vehicle.lf;-cfg.vehicle.lr]*state(6), ...
                    max(state(4),cfg.model.scheduleSpeedFloor))-[input(1);0];
                if any(abs(slip)>=pi/2)
                    invalid=invalid+1;
                else
                    nonlinear=modifiedFialaTire.evaluate(slip,input(2),cfg);
                    mismatch(:,stage)=abs(affine-nonlinear);
                end
            end
            rows=[rows;struct('time',r.attempts.time(k),'visible',r.attempts.targetVisible(k), ...
                'stages',count,'nominalSource',problem.metadata.nominalSource, ...
                'linearizationRefreshes',refreshes,'previousAnchorFailure',anchorFailure, ...
                'referenceRolloutError',anchorError,'firstSteeringDeviationRad',inputs(1,1)-center(1,1), ...
                'firstBetaDeviation',inputs(2,1)-center(2,1), ...
                'maxNormalizedDeviation',max(abs(deviation),[],'all'), ...
                'meanSquaredNormalizedDeviation',mean(deviation.^2,'all'), ...
                'maxPlanAbsBeta',max(abs(inputs(2,:))), ...
                'firstForceMismatchN',max(mismatch(:,1),[],'omitnan'), ...
                'maxForceMismatchN',max(mismatch,[],'all','omitnan'), ...
                'maxForceCapacityExcessN',max(excess,[],'all'), ...
                'invalidPredictedSlipStages',invalid)]; %#ok<AGROW>
        end
        assert(commandDifference<1e-7,'Public-input replay changed an issued command.');
        if r.failure.occurred
            assert(abs(failureTime-r.failure.time)<1e-10 && failureIdentifier==string(r.failure.identifier), ...
                'The recorded failure did not reproduce.');
        else
            assert(isnan(failureTime),'Replay failed on a completed scenario.');
        end
        name=erase(string(files(fileIndex).name),'.mat');
        save(fullfile(planDirectory,name+'.mat'),'stored','failureTime','failureIdentifier','-v7.3');
        writetable(struct2table(rows),fullfile(outputDirectory,name+'-model-audit.csv'));
        x=r.controlState;
        course=r.controlTracking.headingError+atan2(x(:,5),x(:,4));
        course=atan2(sin(course),cos(course));
        commands=cell2mat(cellfun(@(c)c.actuatorInput,r.command.',UniformOutput=false)).';
        trace=array2table([r.controlTime,x(:,4),r.controlTracking.lateralError,course, ...
            [commands;nan(1,2)]],VariableNames={'timeSeconds','speedMps','lateralErrorMeters', ...
            'courseErrorRadians','steeringRadians','brakingRatio'});
        writetable(trace,fullfile(outputDirectory,name+'-control-trace.csv'));
        runtime=table(r.attempts.time,r.attempts.targetVisible, ...
            r.runtime.controllerSeconds,r.runtime.frameSeconds, ...
            VariableNames={'timeSeconds','targetVisible','controllerSeconds','frameSeconds'});
        writetable(runtime,fullfile(outputDirectory,name+'-runtime.csv'));
        plant=r.plantTrace;target=r.targetTruth;
        physical=array2table([plant.time,plant.positionX,plant.positionY,plant.yaw, ...
            target.position,target.yaw],VariableNames={'timeSeconds','egoX','egoY','egoYaw', ...
            'targetX','targetY','targetYaw'});
        writetable(physical,fullfile(outputDirectory,name+'-physical-trace.csv'));
        assert(isequal([cfg.vehicle.length,cfg.vehicle.width,target.length,target.width],[5,2,5,2]), ...
            'The standalone physical audit assumes 5-by-2-m rectangles.');
        visible=rows([rows.visible]);
        item=struct('name',name,'weight',data.weight,'commandDifference',commandDifference, ...
            'failureTime',failureTime,'failureIdentifier',failureIdentifier, ...
            'maxNormalizedDeviation',max([visible.maxNormalizedDeviation]), ...
            'meanSquaredNormalizedDeviation',mean([visible.meanSquaredNormalizedDeviation]), ...
            'maxFirstForceMismatchN',max([visible.firstForceMismatchN]), ...
            'maxForceMismatchN',max([visible.maxForceMismatchN]), ...
            'maxForceCapacityExcessN',max([visible.maxForceCapacityExcessN]), ...
            'invalidPredictedSlipStages',sum([visible.invalidPredictedSlipStages]));
        summary=[summary;item]; %#ok<AGROW>
        writetable(struct2table(summary),fullfile(outputDirectory,'model-audit-summary.csv'));
        fprintf('AUDIT %s command difference %.3g\n',name,commandDifference);
    end
end
