function report = auditPotentialFieldInitialization(outputDirectory)
%auditPotentialFieldInitialization Measure the current raw seed before any optimization.
% Extracts the private constructor verbatim. Dense replay and timing are
% diagnostics only; they do not add controller admission tests. Output files
% and extracted helpers belong outside the source repository.
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
    scenarios=["headOn","acceleratingHeadOn","brakingLead","crossing", ...
        "turningCrossing","curvedHeadOn","curvedCrossing"];
    rows=struct([]);traces=cell(14,1);caseIndex=0;
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
            count=min(cfg.controller.maximumHorizonSteps,cfg.controller.horizonSteps ...
                +ceil(cfg.nonlinear.recoveryHorizonSeconds/cfg.controller.sampleTime));
            anchor=auditCurrentPotentialFieldSeed(model,count);elapsed=zeros(1,7);
            for repeat=1:7
                timer=tic;auditCurrentPotentialFieldSeed(model,count);elapsed(repeat)=toc(timer);
            end
            [row,traces{caseIndex}]=localMeasure(anchor,model);
            row.scenario=scenario;row.medianWarmSeedMilliseconds=1000*median(elapsed);
            rows=[rows,row]; %#ok<AGROW>
            fprintf('%g %-20s gap %.6f m; max lateral %.3f m; core ratio %.3g\n', ...
                speed,scenario,row.minimumSampledClearanceMeters,row.maximumLateralMeters,row.endpointCoreRadiusRatio);
        end
    end
    report=struct('scope',"Raw potential-field initialization; no optimization or closed-loop run", ...
        'sampleTimeSeconds',.05,'replayIntegrator',"ode45",'relativeTolerance',1e-10, ...
        'absoluteTolerance',1e-11,'outputSamplesPerHold',21, ...
        'timingScope',"Seven warm helper calls; excludes terminal construction, solver and audit",'rows',rows);
    writetable(struct2table(rows),fullfile(outputDirectory,'metrics.csv'));
    file=fopen(fullfile(outputDirectory,'metrics.json'),'w');assert(file>=0);
    fprintf(file,'%s\n',jsonencode(report));fclose(file);
    save(fullfile(outputDirectory,'traces.mat'),'report','traces','-v7.3');
end

function [row,trace]=localMeasure(anchor,model)
    cfg=model.cfg;h=cfg.controller.sampleTime;count=size(anchor.inputs,2);
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    times=zeros(21*count,1);clearance=zeros(21*count,1);states=zeros(6,21*count);
    replay=model.initialState;integrationError=0;
    for step=1:count
        [t,x]=ode45(@(~,state)nonlinearBicycleModel.derivative(state,anchor.inputs(:,step),cfg), ...
            linspace(0,h,21),replay,odeset('RelTol',1e-10,'AbsTol',1e-11));
        indices=(step-1)*21+(1:21);times(indices)=(step-1)*h+t;states(:,indices)=x.';
        replay=x(end,:).';difference=replay-anchor.states(:,step+1);
        integrationError=max(integrationError,norm(difference(1:2))+norm(shape(1:2))*abs(difference(3)));
        for point=indices
            q=predictiveSafetyGeometry.predictTarget(model.targetEpoch,times(point));
            clearance(point)=predictiveSafetyGeometry.rectangle(states(1:3,point),shape,q(1:3),q(8:11));
        end
    end
    errors=zeros(5,count+1);
    for step=1:count+1
        errors(:,step)=nonlinearBicycleModel.error(anchor.states(:,step),model.lane,model.nominalReference);
    end
    intrinsic=[anchor.states(4:6,end);anchor.inputs(:,end)] ...
        -[model.terminal.base(4:6);model.terminal.reference.input];
    [minimum,index]=min(clearance);
    row=struct('referenceSpeedMetersPerSecond',cfg.referenceSpeed,'durationSeconds',count*h, ...
        'minimumSampledClearanceMeters',minimum,'minimumTimeSeconds',times(index), ...
        'collisionSamples',sum(clearance==0),'maximumLateralMeters',max(abs(errors(1,:))), ...
        'finalLateralMeters',errors(1,end),'finalHeadingErrorRadians',errors(2,end), ...
        'minimumSpeedMetersPerSecond',min(anchor.states(4,:)), ...
        'maximumSteeringStepRadians',max(abs(diff([0,anchor.inputs(1,:)]))), ...
        'endpointCoreRadiusRatio',norm(model.terminal.quotientFactor*intrinsic)/model.terminal.radius, ...
        'maximumRk4ReplayBodyDifferenceMeters',integrationError);
    trace=struct('model',model,'anchor',anchor,'times',times,'states',states,'clearance',clearance);
end

function localExtract(source,directory)
    code=fileread(source);result="";starts=regexp(code,'(?m)^function','start');
    for helper=["localPotentialFieldSeed","localGuidanceInput","localClip"]
        first=regexp(code,"(?m)^function[^\n]*\<"+helper+"\(",'start','once');
        assert(~isempty(first),'The expected initialization helper is missing.');
        next=starts(find(starts>first,1));if isempty(next),next=numel(code)+1;end
        result=result+string(code(first:next-1))+newline;
    end
    current=replace(result,'function anchor=localPotentialFieldSeed(', 'function anchor=auditCurrentPotentialFieldSeed(');
    file=fopen(fullfile(directory,'auditCurrentPotentialFieldSeed.m'),'w');assert(file>=0);
    fprintf(file,'%s',current);fclose(file);
end
