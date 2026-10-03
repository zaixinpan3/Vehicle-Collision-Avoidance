function report = auditFlowInitialization(outputDirectory)
%auditFlowInitialization Measure raw flow anchors before any optimization.
% Replays the current private seed helper verbatim, then changes only its
% post-flow guidance in a diagnostic copy. Neither copy enters the controller.
% Output files and extracted helpers belong outside the source repository.
    arguments
        outputDirectory (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    helperDirectory=fullfile(outputDirectory,'extractedHelpers');
    if ~isfolder(helperDirectory),mkdir(helperDirectory);end
    localExtract(fullfile(root,'controller','solvePredictiveControl.m'),helperDirectory);
    addpath(helperDirectory);cleanup=onCleanup(@()rmpath(helperDirectory));
    functions={@auditCurrentFlowSeed,@auditReturnFlowSeed};
    variants=["current","returnGuidanceAblation"];
    scenarios=["headOn","acceleratingHeadOn","brakingLead","crossing", ...
        "turningCrossing","curvedHeadOn","curvedCrossing"];
    rows=struct([]);traces=cell(14,2);caseIndex=0;
    for speed=[8,15]
        for scenario=scenarios
            caseIndex=caseIndex+1;
            [x,target,road,cfg]=collisionThreatScenario(scenario, ...
                struct('referenceSpeed',speed,'controller',struct('horizonSteps',8+8*(speed==15))));
            ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4), ...
                'lateralVelocity',x(5),'yawRate',x(6));
            [~,lane,parsedRoad]=readControllerInputs(ego,[],road,cfg);
            frame=predictiveSafetyGeometry.roadFrame(lane,parsedRoad);
            model=struct('cfg',cfg,'initialState',x,'previousInput',[0;0], ...
                'lane',lane,'target',target,'targetEpoch',target,'sampleIndex',0, ...
                'terminal',terminalContinuation.build(cfg,0), ...
                'nominalReference',nonlinearBicycleModel.cruise(cfg,frame(4)));
            count=cfg.controller.horizonSteps+ceil(cfg.nonlinear.recoveryHorizonSeconds/cfg.controller.sampleTime);
            guide=predictiveSafetyGeometry.movingGaussianGuide(x,lane,target,0,count*cfg.controller.sampleTime,cfg);
            anchors=cell(1,2);elapsed=zeros(2,7);
            for variant=1:2,anchors{variant}=functions{variant}(model,count);end
            for repeat=1:7
                order=1:2;if mod(repeat,2)==0,order=2:-1:1;end
                for variant=order
                    timer=tic;functions{variant}(model,count);elapsed(variant,repeat)=toc(timer);
                end
            end
            for variant=1:2
                [row,trace]=localMeasure(anchors{variant},model,guide);
                row.scenario=scenario;row.variant=variants(variant);
                row.medianWarmSeedMilliseconds=1000*median(elapsed(variant,:));
                rows=[rows,row]; %#ok<AGROW>
                traces{caseIndex,variant}=trace;
            end
            last=rows(end).handoffTimeSeconds/cfg.controller.sampleTime;
            if isnan(last),last=count;end
            assert(isequal(anchors{1}.inputs(:,1:round(last)),anchors{2}.inputs(:,1:round(last))), ...
                'The ablation must preserve every input preceding the handoff.');
            fprintf('%g %-20s gap %.6f / %.6f m; max lateral %.3f / %.3f m; core ratio %.3g / %.3g\n', ...
                speed,scenario,rows(end-1).minimumSampledClearanceMeters,rows(end).minimumSampledClearanceMeters, ...
                rows(end-1).maximumLateralMeters,rows(end).maximumLateralMeters, ...
                rows(end-1).endpointCoreRadiusRatio,rows(end).endpointCoreRadiusRatio);
        end
    end
    report=struct('scope',"Raw initialization and a post-flow-guidance ablation; no optimization or closed-loop run", ...
        'sampleTimeSeconds',.05,'replayIntegrator',"ode45",'relativeTolerance',1e-10, ...
        'absoluteTolerance',1e-11,'outputSamplesPerHold',21, ...
        'timingScope',"Seven alternating warm helper calls; excludes terminal construction, solver and audit", ...
        'rows',rows);
    writetable(struct2table(rows),fullfile(outputDirectory,'metrics.csv'));
    file=fopen(fullfile(outputDirectory,'metrics.json'),'w');assert(file>=0);
    fprintf(file,'%s\n',jsonencode(report));fclose(file);
    save(fullfile(outputDirectory,'traces.mat'),'report','traces','-v7.3');
end

function [row,trace]=localMeasure(anchor,model,guide)
    cfg=model.cfg;h=cfg.controller.sampleTime;count=size(anchor.inputs,2);
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    times=zeros(21*count,1);clearance=zeros(21*count,1);states=zeros(6,21*count);
    replay=model.initialState;integrationError=0;
    for step=1:count
        [t,x]=ode45(@(~,state)nonlinearBicycleModel.derivative(state,anchor.inputs(:,step),cfg), ...
            linspace(0,h,21),replay,odeset('RelTol',1e-10,'AbsTol',1e-11));
        indices=(step-1)*21+(1:21);times(indices)=(step-1)*h+t;states(:,indices)=x.';
        replay=x(end,:).';
        difference=replay-anchor.states(:,step+1);
        integrationError=max(integrationError,norm(difference(1:2))+norm(shape(1:2))*abs(difference(3)));
        for point=indices
            q=predictiveSafetyGeometry.targetFlow(model.targetEpoch,times(point));
            clearance(point)=predictiveSafetyGeometry.rectangle(states(1:3,point),shape,q(1:3),q(8:11));
        end
    end
    errors=zeros(5,count+1);stations=zeros(1,count+1);
    for step=1:count+1
        [errors(:,step),projection]=nonlinearBicycleModel.error(anchor.states(:,step),model.lane,model.nominalReference);
        stations(step)=projection.station;
    end
    active=guide.amplitude~=0 & (0:count-1)*h<=guide.endTime;
    handoff=find((1:count)>cfg.controller.horizonSteps & ~active,1);
    handoffTime=NaN;residual=NaN;jump=NaN;
    if ~isempty(handoff) && handoff>1
        handoffTime=(handoff-1)*h;
        residual=guide.amplitude*exp(-.5*((handoffTime-guide.centerTime)/guide.width)^2);
        jump=anchor.inputs(1,handoff)-anchor.inputs(1,handoff-1);
    end
    intrinsic=[anchor.states(4:6,end);anchor.inputs(:,end)] ...
        -[model.terminal.base(4:6);model.terminal.reference.input];
    [minimum,index]=min(clearance);
    row=struct('referenceSpeedMetersPerSecond',cfg.referenceSpeed,'durationSeconds',count*h, ...
        'gaussianAmplitudeMeters',guide.amplitude,'gaussianWidthSeconds',guide.width, ...
        'gaussianCenterSeconds',guide.centerTime,'gaussianInitialOffsetMeters', ...
        guide.amplitude*exp(-.5*(guide.centerTime/guide.width)^2), ...
        'flowEndSeconds',guide.endTime,'handoffTimeSeconds',handoffTime, ...
        'handoffResidualGaussianMeters',residual,'handoffSteeringJumpRadians',jump, ...
        'minimumSampledClearanceMeters',minimum,'minimumTimeSeconds',times(index), ...
        'collisionSamples',sum(clearance==0),'maximumLateralMeters',max(abs(errors(1,:))), ...
        'finalLateralMeters',errors(1,end),'finalHeadingErrorRadians',errors(2,end), ...
        'minimumSpeedMetersPerSecond',min(anchor.states(4,:)), ...
        'maximumSteeringStepRadians',max(abs(diff([0,anchor.inputs(1,:)]))), ...
        'maximumNominalStationErrorMeters',max(abs(stations-(stations(1)+cfg.referenceSpeed*(0:count)*h))), ...
        'endpointCoreRadiusRatio',norm(model.terminal.quotientFactor*intrinsic)/model.terminal.radius, ...
        'maximumRk4ReplayBodyDifferenceMeters',integrationError);
    trace=struct('model',model,'guide',guide,'anchor',anchor,'times',times,'states',states,'clearance',clearance);
end

function localExtract(source,directory)
    % The diagnostic follows the current source; fail if its handoff changes.
    code=fileread(source);result="";starts=regexp(code,'(?m)^function','start');
    for helper=["localFlowSeed","localShape","localClip"]
        first=regexp(code,"(?m)^function[^\n]*\<"+helper+"\(",'start','once');
        assert(~isempty(first),'The expected initialization helper is missing.');
        next=starts(find(starts>first,1));if isempty(next),next=numel(code)+1;end
        result=result+string(code(first:next-1))+newline;
    end
    current=replace(result,'function anchor=localFlowSeed(', 'function anchor=auditCurrentFlowSeed(');
    before="completionStarted=true;seed=terminalContinuation.fit(model.terminal,[x;previous],model.sampleIndex+index-1);" ...
        +newline+"            [~,~,deviation]=terminalContinuation.membership([x;previous],seed.epochIndex,seed);" ...
        +newline+"            u=seed.reference.input+seed.gain*deviation;";
    assert(count(current,before)==1,'The post-flow guidance changed; review this diagnostic ablation.');
    after="completionStarted=true;"+newline ...
        +"            u=nonlinearBicycleModel.nominalFeedback(x,previous,model.lane,reference,cfg,nominalTerminal);";
    altered=replace(replace(current,'function anchor=auditCurrentFlowSeed(', ...
        'function anchor=auditReturnFlowSeed('),before,after);
    file=fopen(fullfile(directory,'auditCurrentFlowSeed.m'),'w');assert(file>=0);
    fprintf(file,'%s',current);fclose(file);
    file=fopen(fullfile(directory,'auditReturnFlowSeed.m'),'w');assert(file>=0);
    fprintf(file,'%s',altered);fclose(file);
end
