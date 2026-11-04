function results=analyzeOutputFeedbackQpFailures(outputDirectory)
%analyzeOutputFeedbackQpFailures Offline diagnosis; never changes live policies.
% Reconstruct recorded failures, export the exact linearized constraints, and
% distinguish feasibility from QP convergence with independent linear programs.
    arguments
        outputDirectory (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(root,'controller'),fullfile(root,'config'));
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    sources=["report/TARGET_MOTION_CONTRACT_20261104/continuing/8-headOn.json", ...
        "report/TARGET_MOTION_CONTRACT_20261104/continuing/8-acceleratingHeadOn.json", ...
        "report/TARGET_MOTION_CONTRACT_20261104/continuing/8-curvedHeadOn.json", ...
        "report/SPARSE_RTI_20261104/high-speed/25-stoppedLead.json", ...
        "report/SPARSE_RTI_20261104/high-speed/25-hardBrakingLead.json"];
    results=struct([]);
    for source=sources
        record=jsondecode(fileread(fullfile(root,source)));
        [block,state,cfg,road,replayError]=localReplay(record);
        request=struct('operation',"linearizeStep",'block',block,'cfg',cfg,'reference',road,'previousState',state);
        [qp,~,problem]=solvePredictiveControl('outputFeedback',request);
        groups=localGroups(qp,cfg);tests=localAblations(qp,groups);
        [~,indices]=unique([problem.scenarios.rootIndex],'stable');hypotheses=[problem.scenarios(indices).initial];
        base=struct('operation',"linearize",'hypotheses',hypotheses,'memory',problem.memory, ...
            'input',problem.previousInput,'policies',qp.policies,'cfg',cfg,'reference',road);
        variants=struct([]);
        for count=[problem.horizon,8,16]
            for turn=[0,-1,1]
                probe=base;probe.policies=repmat(qp.policies(end),1,count);
                keep=min(count-1,numel(qp.policies)-1);probe.policies(1:keep)=qp.policies(1:keep);
                if turn~=0
                    for i=1:count-1
                        probe.policies(i).K(:)=0;
                        probe.policies(i).v=[max(-cfg.outputFeedback.steeringMaximum,min(cfg.outputFeedback.steeringMaximum, ...
                            problem.previousInput(1)+turn*min(i,count-i)*cfg.controller.sampleTime*cfg.outputFeedback.steeringRateMaximum));-.6];
                    end
                end
                try
                    q=solvePredictiveControl('outputFeedback',probe);g=localGroups(q,cfg);
                    v=localLp(q,true(size(q.bounds)),q.lower,q.upper,false(size(q.bounds)));
                    terminal=localLp(q,true(size(q.bounds)),q.lower,q.upper,g=="terminal");
                    entry=struct('horizon',count,'steeringSeedDirection',turn,'seedShape',"shiftOrTriangularSteering",'feasibility',v, ...
                        'minimumTerminalSlack',terminal,'error',"");
                catch exception
                    entry=struct('horizon',count,'steeringSeedDirection',turn,'seedShape',"shiftOrTriangularSteering",'feasibility',[], ...
                        'minimumTerminalSlack',[],'error',string(exception.identifier));
                end
                if isempty(variants),variants=entry;else,variants(end+1)=entry;end %#ok<AGROW>
            end
        end
        ego=problem.scenarios(1).initial.ego;target=problem.scenarios(1).initial.target;
        trace=qp.nominal{1};terminalTarget=trace.targets(:,end);terminalEgo=trace.ego(:,end);
        targetPreview=[];
        if terminalTarget(4)>0 && terminalTarget(5)>-cfg.outputFeedback.minimumTargetDeceleration
            for t=cfg.outputFeedback.terminalPreviewSeconds*[.5,1]
                q=predictiveSafetyGeometry.predictTarget(terminalTarget,t);delta=terminalEgo(1:2)-q(1:2);
                entry=struct('timeAfterTerminal',t,'targetPosition',q(1:2), ...
                    'nominalCenterDistance',norm(delta),'clearancePositionGradient',delta.'/norm(delta));
                if isempty(targetPreview),targetPreview=entry;else,targetPreview(end+1)=entry;end %#ok<AGROW>
            end
        end
        result=struct('source',source,'timeSeconds',block.time,'ego',ego,'target',target, ...
            'previousInput',problem.previousInput,'retainedHypotheses',numel(hypotheses), ...
            'maximumReplayCommandError',replayError,'variables',size(qp.inequality,2), ...
            'inequalities',size(qp.inequality,1),'equalities',size(qp.equality,1), ...
            'nominalInputs',trace.inputs,'nominalTerminalEgo',terminalEgo, ...
            'nominalTerminalTarget',terminalTarget,'targetPreview',targetPreview, ...
            'maximumNominalConstraint',max(qp.nominalConstraints),'ablations',tests,'variants',variants, ...
            'trialWithoutTrust',localTrial(qp,base));
        name=sprintf('%g-%s',record.referenceSpeedMetersPerSecond,record.scenario);
        localWrite(fullfile(outputDirectory,string(name)+".json"),result);
        if isempty(results),results=result;else,results(end+1)=result;end %#ok<AGROW>
    end
    localWrite(fullfile(outputDirectory,'summary.json'),results);
end

function [block,state,cfg,road,error]=localReplay(record)
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',record.referenceSpeedMetersPerSecond));
    cfg.model.speedMaximum=max(cfg.model.speedMaximum,1.2*cfg.referenceSpeed);
    [~,~,road,cfg]=collisionThreatScenario(string(record.scenario),cfg);
    hypotheses=record.initialInformationContract;truth=hypotheses(2);input=zeros(2,1);
    y=solvePredictiveControl('outputFeedback',struct('operation',"sensor",'hypothesis',truth,'input',input,'time',0,'cfg',cfg));
    stream=RandStream('mt19937ar','Seed',record.randomSeed);bounds=cfg.outputFeedback.noiseBounds;
    y.position=y.position+localNoise(stream,bounds(1));y.acceleration=y.acceleration+localNoise(stream,bounds(2));
    y.yawRate=y.yawRate+.5*bounds(3)*(2*rand(stream)-1);y.relativePosition=y.relativePosition+localNoise(stream,bounds(4));
    block=struct('kind',"jointSensorBlock-v1",'time',0,'heldInput',input,'samples',y,'initialHypotheses',hypotheses);
    state=[];error=0;
    for i=1:numel(record.trace)
        [command,~,~,state]=collisionAvoidanceController(block,[],road,cfg,state);
        error=max(error,norm(command.actuatorInput-record.trace(i).input,Inf));
        assert(error<1e-7,'Replay differs from the archived commands.');
        block=struct('kind',"jointSensorBlock-v1",'time',i*cfg.controller.sampleTime, ...
            'heldInput',command.actuatorInput,'samples',record.trace(i).rawSensorBlock);
    end
end

function noise=localNoise(stream,radius)
    angle=2*pi*rand(stream);noise=.5*radius*sqrt(rand(stream))*[cos(angle);sin(angle)];
end

function groups=localGroups(qp,cfg)
    groups=strings(0,1);n=size(qp.stateIndices,2);ns=size(qp.stateIndices,3);
    commandCount=6+2*isfinite(cfg.model.brakingRatioRateMaximum);
    for s=1:ns
        for i=1:n
            groups=[groups;repmat("command",commandCount,1); ...
                repmat("state",6*(cfg.outputFeedback.predictionSubsteps+1),1);repmat("collision",24,1)]; %#ok<AGROW>
            if i==n
                q=qp.nominal{s}.targets(:,end);count=2;
                if q(4)==0 || q(5)<=-cfg.outputFeedback.minimumTargetDeceleration,count=4;end
                groups=[groups;repmat("terminal",count,1)]; %#ok<AGROW>
            end
        end
    end
    assert(numel(groups)==numel(qp.nominalConstraints));groups=groups(qp.activeRows);
end

function tests=localAblations(qp,groups)
    all=true(size(qp.bounds));none=false(size(qp.bounds));
    brakeLower=qp.lower;brakeUpper=qp.upper;
    brake=qp.policyIndices(2,:);
    brakeLower(brake)=qp.lowerWithoutTrust(brake);brakeUpper(brake)=qp.upperWithoutTrust(brake);
    tests=struct('full',localLp(qp,all,qp.lower,qp.upper,none), ...
        'withoutTerminal',localLp(qp,groups~="terminal",qp.lower,qp.upper,none), ...
        'withoutCollision',localLp(qp,groups~="collision",qp.lower,qp.upper,none), ...
        'withoutCommand',localLp(qp,groups~="command",qp.lower,qp.upper,none), ...
        'withoutState',localLp(qp,groups~="state",qp.lower,qp.upper,none), ...
        'withoutTrustBox',localLp(qp,all,qp.lowerWithoutTrust,qp.upperWithoutTrust,none), ...
        'withoutBrakeCoefficientTrust',localLp(qp,all,brakeLower,brakeUpper,none), ...
        'terminalSlack',localLp(qp,all,qp.lower,qp.upper,groups=="terminal"), ...
        'terminalSlackWithoutTrust',localLp(qp,all,qp.lowerWithoutTrust,qp.upperWithoutTrust,groups=="terminal"));
end

function result=localLp(qp,keep,lower,upper,elastic)
    a=qp.inequality(keep,:);b=qp.bounds(keep);e=qp.equality;f=zeros(size(a,2),1);
    add=any(elastic);
    if add
        a=[a,-double(elastic(keep))];e=[e,sparse(size(e,1),1)];
        lower=[lower;0];upper=[upper;Inf];f=[f;1];
    end
    options=optimoptions('linprog','Display','none','ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9);
    [x,value,flag]=linprog(f,a,b,e,zeros(size(e,1),1),lower,upper,options);
    residual=NaN;slack=NaN;
    if flag>0
        residual=max([0;a*x-b;abs(e*x);lower-x;x-upper]);
        if add,slack=value;end
    end
    result=struct('exitFlag',flag,'maximumResidual',residual,'terminalSlackMeters',slack);
end

function result=localTrial(qp,base)
    options=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex', ...
        'ConstraintTolerance',1e-8,'MaxIterations',200);
    [step,~,flag]=quadprog(qp.hessian,qp.gradient,qp.inequality,qp.bounds, ...
        qp.equality,zeros(size(qp.equality,1),1),qp.lowerWithoutTrust,qp.upperWithoutTrust,[],options);
    result=struct('exitFlag',flag,'maximumLinearViolation',NaN,'maximumCoefficientStep',NaN, ...
        'nominalBrakeCoefficients',arrayfun(@(p)p.v(2),base.policies),'trialBrakeCoefficients',[], ...
        'nonlinearChecks',[]);
    if flag<=0,return;end
    result.maximumLinearViolation=max([0;qp.inequality*step-qp.bounds]);
    result.maximumCoefficientStep=max(abs(step(1:qp.policyCount)));
    trials=struct([]);nm=base.cfg.outputFeedback.memoryDimension;
    for scale=[1,.95]
        probe=base;
        point=qp.initialPoint+scale*step(1:numel(qp.initialPoint));
        for i=1:size(qp.policyIndices,2)
            v=point(qp.policyIndices(:,i));a=2;b=a+2*nm;c=b+nm*nm;d=c+8*nm;
            probe.policies(i)=struct('v',v(1:a),'K',reshape(v(a+1:b),2,nm), ...
                'A',reshape(v(b+1:c),nm,nm),'B',reshape(v(c+1:d),nm,8),'c',v(d+1:end));
        end
        if scale==1,result.trialBrakeCoefficients=arrayfun(@(p)p.v(2),probe.policies);end
        try
            q=solvePredictiveControl('outputFeedback',probe);
            groups=localGroups(q,probe.cfg);values=q.nominalConstraints(q.activeRows);
            entry=struct('stepScale',scale,'maximumNonlinearViolation',max([0;q.nominalConstraints]), ...
                'terminalViolationMeters',max([0;values(groups=="terminal")]), ...
                'collisionViolationMeters',max([0;values(groups=="collision")]), ...
                'commandViolation',max([0;values(groups=="command")]), ...
                'stateViolation',max([0;values(groups=="state")]), ...
                'predictedInputs',q.nominal{1}.inputs,'error',"");
        catch exception
            entry=struct('stepScale',scale,'maximumNonlinearViolation',NaN,'terminalViolationMeters',NaN, ...
                'collisionViolationMeters',NaN,'commandViolation',NaN,'stateViolation',NaN,'predictedInputs',[], ...
                'error',string(exception.identifier));
        end
        if isempty(trials),trials=entry;else,trials(end+1)=entry;end %#ok<AGROW>
    end
    result.nonlinearChecks=trials;
end

function localWrite(path,value)
    f=fopen(path,'w');assert(f>=0);cleanup=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(value,PrettyPrint=true));
end
