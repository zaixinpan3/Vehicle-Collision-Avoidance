function summary = diagnoseFluidLinearizationAnchors(snapshotDirectory,outputDirectory)
%diagnoseFluidLinearizationAnchors Test model anchors on saved failed frames.
% This offline experiment does not replace the online initialization policy.
% The flow fit uses a cruise bootstrap; bounded inputs then define a fresh
% nonlinear rollout and one trajectory-linearized hard optimization problem.
    arguments
        snapshotDirectory (1,1) string
        outputDirectory (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'));
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    summary=struct([]);
    for name=["straight","circular","sCurve"]
        data=load(fullfile(snapshotDirectory,name+'-replay.mat'),'snapshot');
        snapshot=data.snapshot;
        if isfield(snapshot,'diagnosticFrame')
            model=snapshot.diagnosticFrame.model;
        else
            model=snapshot.diagnosticModel;
        end
        model.cfg=collisionAvoidanceControllerConfig(model.cfg);
        cfg=model.cfg;cfg.solver.certificateSearchTimeLimit=30;
        bootstrap=model;bootstrap.cfg.model.linearizationPolicy="cruise";
        if isfield(bootstrap,'initializationPlan')
            bootstrap=rmfield(bootstrap,'initializationPlan');
        end
        frameTimer=tic;
        bootstrapProgram=formulateAvoidanceProblem(bootstrap);
        [point,~,initialization]=solveHardCbfClf.fluidInitialize(bootstrapProgram,cfg);
        assert(~isempty(point),'No finite fluid seed was fitted.');
        rawInput=reshape(point(bootstrapProgram.layout.planIndex),2,[]);
        lower=[-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum];
        upper=[cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum];
        % Projection is an explicitly labeled diagnostic. It does not retain
        % the exact flow path, terminal fit or a feasibility certificate.
        input=min(max(rawInput,lower),upper);
        model.initializationPlan=input;model.nominalSource="flowTrajectory";
        identifier="";program=[];result=[];
        physicalExcess=NaN;jointExcess=NaN;coneExcess=NaN;
        maximumDeviation=NaN;firstForceError=NaN;maximumForceError=NaN;
        candidateRolloutValid=false;candidateRolloutFailure="";
        anchorState=[];
        try
            anchorState=ltvBicycleModel.nominalRollout(model,input);
            program=formulateAvoidanceProblem(model);
            assert(max(abs(program.prediction.linearizationInputs-input),[],'all')==0);
            assert(max(abs(program.prediction.linearizationStates-anchorState),[],'all')==0);
            % Select this anchor once: no second flow fit may move the seed
            % while silently retaining the just-constructed tangent model.
            program.fluidReference.active=false;
            solverCfg=cfg;solverCfg.solver.workTimer=frameTimer;solverCfg.solver.workTimeLimit=30;
            [program,result,search]=solveHardCbfClf.fixedDirections(program,model,solverCfg); %#ok<ASGLU>
            feasible=result.feasible;message=string(result.message);
            if feasible
                decision=result.decision;
                physicalExcess=max(program.physicalMatrix*decision-program.physicalBound);
                jointExcess=max(avoidanceSafetyGeometry.jointResidual( ...
                    program,decision,program.jointCertificate.angles)-program.jointCertificate.upperBound);
                coneExcess=localConeExcess(program,decision);
                assert(physicalExcess<=1e-8 && jointExcess<=1e-8 && coneExcess<=1e-8);
                candidate=reshape(decision(program.layout.planIndex),2,[]);
                scale=[cfg.model.frontWheelSteeringAngleMaximum;max(abs([lower(2),upper(2)]))];
                maximumDeviation=max(abs((candidate-input)./scale),[],'all');
                [firstForceError,maximumForceError]=localForceError(program,candidate,cfg);
                try
                    ltvBicycleModel.nominalRollout(model,candidate);
                    candidateRolloutValid=true;
                catch rolloutException
                    candidateRolloutFailure=string(rolloutException.identifier);
                end
            end
        catch exception
            feasible=false;
            identifier=string(exception.identifier);message=string(exception.message);
        end
        row=struct('scenario',name,'failureTime',snapshot.failure.time, ...
            'selectedReference',initialization.selectedReference, ...
            'rawMaximumSteeringRad',max(abs(rawInput(1,:))), ...
            'rawMaximumInputExcess',max([rawInput-upper;lower-rawInput],[],'all'), ...
            'projectedMaximumSteeringRad',max(abs(input(1,:))), ...
            'projectedInputChangeNorm',norm(input-rawInput,'fro'), ...
            'anchorRolloutValid',~isempty(anchorState),'optimizationAccepted',feasible, ...
            'failureIdentifier',identifier,'solverMessage',message, ...
            'physicalExcess',physicalExcess,'jointExcess',jointExcess,'coneExcess',coneExcess, ...
            'maximumNormalizedCandidateDeviation',maximumDeviation, ...
            'firstForceMismatchN',firstForceError,'maximumForceMismatchN',maximumForceError, ...
            'candidateRolloutValid',candidateRolloutValid,'candidateRolloutFailure',candidateRolloutFailure, ...
            'diagnosticSeconds',toc(frameTimer));
        summary=[summary;row]; %#ok<AGROW>
        save(fullfile(outputDirectory,name+'.mat'),'model','bootstrapProgram','rawInput', ...
            'input','anchorState','initialization','program','result','row','-v7.3');
        writetable(struct2table(summary),fullfile(outputDirectory,'summary.csv'));
        disp(row);
    end
end

function excess=localConeExcess(program,decision)
    value=program.b-program.A*decision;cursor=program.cones(1)+program.cones(2);
    excess=max([0;abs(value(1:program.cones(1)));-value(program.cones(1)+1:cursor)]);
    for dimension=program.cones(3:end).'
        v=value(cursor+(1:dimension));cursor=cursor+dimension;
        excess=max(excess,norm(v(2:end))-v(1));
    end
end

function [first,maximum]=localForceError(program,inputs,cfg)
    prediction=program.prediction;
    states=prediction.egoStateOffset+reshape(pagemtimes(prediction.egoStateMatrix,inputs(:)),6,[]);
    errors=nan(2,size(inputs,2));
    for stage=1:size(inputs,2)
        x=states(:,stage);u=inputs(:,stage);tire=prediction.tireModels{stage};
        slip=atan2(x(5)+[cfg.vehicle.lf;-cfg.vehicle.lr]*x(6),max(x(4),cfg.model.scheduleSpeedFloor))-[u(1);0];
        if all(abs(slip)<pi/2)
            nonlinear=modifiedFialaTire.evaluate(slip,u(2),cfg);
            errors(:,stage)=abs(tire.state*x+tire.input*u+tire.constant-nonlinear);
        end
    end
    first=max(errors(:,1),[],'omitnan');maximum=max(errors,[],'all','omitnan');
end
