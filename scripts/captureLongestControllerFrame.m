function capture = captureLongestControllerFrame(traceDirectory, outputDirectory)
%captureLongestControllerFrame Recover a recorded frame and its numerical warm start.
% Replay saved measured states and original SCvx iteration counts. The wall
% deadline is disabled only during reconstruction, so host load cannot truncate
% a prefix call differently. Save the original configuration for later timing.
    arguments
        traceDirectory (1,1) string
        outputDirectory (1,1) string
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    longest = -Inf; ranking = struct([]);
    for cohort = ["baseline","default"]
        for file = ["short-replays.json","oncoming.json"]
            data = jsondecode(fileread(fullfile(traceDirectory,cohort,file)));
            for scenario = 1:numel(data.results)
                record = data.results(scenario);
                for frame = 1:numel(record.trace)
                    entry = record.trace(frame);
                    row = struct('cohort',cohort,'scenario',string(record.scenario), ...
                        'frame',frame,'time',entry.time,'seconds',entry.controllerSeconds);
                    if isempty(ranking),ranking=row;else,ranking(end+1)=row;end %#ok<AGROW>
                    if entry.controllerSeconds>longest
                        longest=entry.controllerSeconds;selected=record;
                        selectedFrame=frame;selectedCohort=cohort;
                    end
                end
            end
        end
    end
    [~,order]=sort([ranking.seconds],'descend');ranking=ranking(order);
    localWrite(fullfile(outputDirectory,'frame-ranking.json'),ranking);
    cfg=collisionAvoidanceControllerConfig(struct( ...
        'referenceSpeed',selected.configuration.referenceSpeed, ...
        'controller',struct('horizonSteps',selected.configuration.controller.horizonSteps)));
    assert(isequaln(jsondecode(jsonencode(cfg)),selected.configuration), ...
        'Recorded configuration differs from this controller source.');
    road=struct('centerline',[-100,0;1000,0],'lateralClearance',[4;4]);
    if string(selected.scenario)=="circular"
        road=struct('referenceCurve',struct('origin',[0;0],'heading',0, ...
            'curvature',.005,'length',200),'lateralClearance',[4;4]);
    end
    q=selected.targetInitialState;prior=[];stateTime=0;verification=struct([]);
    for frame=1:selectedFrame
        entry=selected.trace(frame);x=entry.state;
        held=zeros(2,1);if frame>1,held=selected.trace(frame-1).input;end
        ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4), ...
            'lateralVelocity',x(5),'yawRate',x(6),'stateTime',stateTime,'heldActuatorInput',held);
        target=localTarget(q);steps=entry.sequentialIterations;
        if iscell(steps),steps=[steps{:}];end
        assert(isempty(entry.solverFailures),'Prefix has unrecorded exception iterations.');
        fixedCfg=cfg;fixedCfg.solver.timeLimitSeconds=Inf;
        fixedCfg.nonlinear.maximumIterations=max([steps.iteration]);
        if frame==selectedFrame
            capture=struct('cohort',selectedCohort,'scenario',string(selected.scenario), ...
                'frame',frame,'originalSeconds',longest,'ego',ego,'target',target, ...
                'road',road,'configuration',cfg,'fixedWorkConfiguration',fixedCfg, ...
                'previousState',prior,'originalTrace',entry);
        end
        frameTimer=tic;
        [command,~,problem,prior]=collisionAvoidanceController(ego,target,road,fixedCfg,prior);
        elapsed=toc(frameTimer);actual=problem.metadata.search.sequentialIterations;
        actual=[actual{:}];
        inputError=max(abs(command.actuatorInput-entry.input));
        assert(inputError<1e-9,'Applied input differs from the recorded prefix.');
        assert(problem.metadata.solverCallCount==entry.solverCalls,'Solver work differs.');
        assert(isequal([actual.iteration],[steps.iteration]),'Iteration sequence differs.');
        assert(isequal([actual.acceptedIterate],[steps.acceptedIterate]),'Acceptance sequence differs.');
        row=struct('frame',frame,'inputMaximumError',inputError, ...
            'safetySlackError',abs(problem.solution.safety-entry.predictiveBarrierValue), ...
            'clfSlackError',abs(problem.solution.clfSlack-entry.clfSlack), ...
            'solverCalls',problem.metadata.solverCallCount,'seconds',elapsed);
        if isempty(verification),verification=row;else,verification(end+1)=row;end %#ok<AGROW>
        fprintf('Reconstructed frame %d/%d: input error %.3g, %d solver calls, %.3f s\n', ...
            frame,selectedFrame,inputError,row.solverCalls,elapsed);
        if ~isempty(q),q=predictiveSafetyGeometry.targetFlow(problem.model.target,cfg.controller.sampleTime);end
        stateTime=stateTime+cfg.controller.sampleTime;
    end
    capture.fixedWorkSolution=problem.solution;
    capture.fixedWorkSearch=problem.metadata.search;
    save(fullfile(outputDirectory,'captured-frame.mat'),'capture','verification');
    localWrite(fullfile(outputDirectory,'prefix-verification.json'),verification);
    localWrite(fullfile(outputDirectory,'selected-frame.json'),rmfield(capture, ...
        {'previousState','fixedWorkSolution','fixedWorkSearch'}));
end

function target=localTarget(q)
    target=[];if isempty(q),return;end
    direction=[cos(q(3)+q(6));sin(q(3)+q(6))];velocity=q(4)*direction;
    yawRate=q(4)*sin(q(6))/q(7);
    target=struct('targetPositionInertial',q(1:2),'targetVelocityInertial',velocity, ...
        'targetYawInertial',q(3),'targetYawRate',yawRate,'targetSideslip',q(6), ...
        'targetTangentialAcceleration',q(5),'targetRearAxleDistance',q(7), ...
        'targetAccelerationInertial',q(5)*direction+yawRate*[-velocity(2);velocity(1)], ...
        'targetLength',2*q(8),'targetWidth',2*q(9),'targetRectangleOffset',q(10:11));
end

function localWrite(path,value)
    file=fopen(path,'w');assert(file>=0);cleanup=onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(value,PrettyPrint=true));
end
