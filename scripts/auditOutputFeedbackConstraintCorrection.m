function results=auditOutputFeedbackConstraintCorrection(outputDirectory)
%auditOutputFeedbackConstraintCorrection Replay new failures after removing added constraints.
% Independent LP ablations are diagnostics only; no relaxed command is applied.
    arguments
        outputDirectory (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    files=[dir(fullfile(outputDirectory,'8-mps','8-*.json')); ...
        dir(fullfile(outputDirectory,'25-mps','25-*.json'))];
    results=struct([]);
    for source=files.'
        record=jsondecode(fileread(fullfile(source.folder,source.name)));
        if string(record.outcome)~="failed",continue;end
        [block,state,cfg,road,replayError]=localReplay(record);
        request=struct('operation',"linearizeStep",'block',block,'cfg',cfg,'reference',road,'previousState',state);
        qp=solvePredictiveControl('outputFeedback',request);
        commandCount=6+2*isfinite(cfg.model.brakingRatioRateMaximum);
        stage=[repmat("command",commandCount,1); ...
            repmat("state",6*(cfg.outputFeedback.predictionSubsteps+1),1);repmat("collision",24,1)];
        groups=repmat(stage,size(qp.stateIndices,2)*size(qp.stateIndices,3),1);
        assert(numel(groups)==numel(qp.nominalConstraints));groups=groups(qp.activeRows);
        allRows=true(size(qp.bounds));none=false(size(qp.bounds));
        entry=struct('source',source.name,'completedHolds',numel(record.trace), ...
            'maximumReplayedCommandError',replayError, ...
            'full',localLp(qp,allRows,qp.lower,qp.upper,none), ...
            'withoutCollision',localLp(qp,groups~="collision",qp.lower,qp.upper,none), ...
            'withoutOtherStepBoxes',localLp(qp,allRows,qp.lowerWithoutTrust,qp.upperWithoutTrust,none), ...
            'collisionSlack',localLp(qp,allRows,qp.lower,qp.upper,groups=="collision"));
        if isempty(results),results=entry;else,results(end+1)=entry;end %#ok<AGROW>
    end
    localWrite(fullfile(outputDirectory,'remaining-failures.json'),results);
end

function [block,state,cfg,road,error]=localReplay(record)
    % These regression runs use default configuration plus scenario/speed.
    % JSON maps Inf to null, so reconstruct defaults instead of importing nulls.
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
    result=struct('exitFlag',flag,'maximumResidual',residual,'collisionSlackMeters',slack);
end

function localWrite(path,value)
    f=fopen(path,'w');assert(f>=0);cleanup=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(value,PrettyPrint=true));
end
