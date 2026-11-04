function campaign = runJointOutputFeedbackScenario(options)
%runJointOutputFeedbackScenario Raw-sensor dynamic output-feedback experiments.
% The initial finite joint contract is declared once; no truth state or point
% observer estimate is supplied to subsequent controller calls. An independent
% ode45 plant supplies only position GNSS, body IMU, gyro and relative radar.
% The finite initial contract and future noise cases do not cover all bounded
% uncertainty. Reported checks are empirical, with no deterministic guarantee.
    arguments
        options.Scenarios (1,:) string = ["stoppedLead","hardBrakingLead"]
        options.Speeds (1,:) double {mustBePositive,mustBeFinite} = 1.5
        options.Frames (1,1) double {mustBeInteger,mustBePositive,mustBeFinite} = 12
        options.ControllerConfiguration (1,1) struct = struct()
        options.OutputDirectory (1,1) string = string(tempname)
        options.RandomSeed (1,1) double {mustBeInteger,mustBeNonnegative} = 20261104
        options.DropoutHolds (1,:) double {mustBeInteger,mustBeNonnegative} = []
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    if ~isfolder(options.OutputDirectory),mkdir(options.OutputDirectory);end
    summaries=struct([]);
    for speed=options.Speeds
        for name=options.Scenarios
            cfg=collisionAvoidanceControllerConfig(options.ControllerConfiguration);
            cfg.referenceSpeed=speed;cfg.model.speedMaximum=max(cfg.model.speedMaximum,1.2*speed);
            [x,q,road,cfg]=collisionThreatScenario(name,cfg);
            reference=road;input=zeros(2,1);time=0;state=[];
            stream=RandStream('mt19937ar','Seed',options.RandomSeed);
            initial=localInitialContract(x,q);
            truth=struct('ego',x,'target',q,'parameters',ones(4,1),'sensorBias',zeros(7,1));
            sample=localSensor(truth,input,cfg,time,stream,false);
            block=struct('kind',"jointSensorBlock-v1",'time',time,'heldInput',input, ...
                'samples',sample,'initialHypotheses',initial);
            trace=struct([]);failure="";minimumClearance=Inf;
            for frame=1:options.Frames
                try
                    [command,~,prediction,state]=collisionAvoidanceController(block,[],reference,cfg,state);
                catch exception
                    failure=string(exception.identifier)+": "+string(exception.message);break;
                end
                input=command.actuatorInput;
                [truth,samples,clearance]=localReplay(truth,input,cfg,time,stream, ...
                    ismember(frame,options.DropoutHolds));
                minimumClearance=min(minimumClearance,clearance);
                entry=struct('time',time,'input',input,'memory',state.memory, ...
                    'posteriorCount',numel(state.hypotheses),'metadata',prediction.metadata, ...
                    'minimumReplayClearanceMeters',clearance,'stateAfterHold',truth.ego, ...
                    'rawSensorBlock',samples);
                if isempty(trace),trace=entry;else,trace(end+1)=entry;end %#ok<AGROW>
                time=time+cfg.controller.sampleTime;
                block=struct('kind',"jointSensorBlock-v1",'time',time,'heldInput',input,'samples',samples);
            end
            outcome="completed";if strlength(failure)>0,outcome="failed";end
            file=fullfile(options.OutputDirectory,sprintf('%g-%s.json',speed,name));
            result=struct('scenario',name,'referenceSpeedMetersPerSecond',speed, ...
                'observationModel',"rawSensorJointDynamicOutputFeedback",'randomSeed',options.RandomSeed, ...
                'outcome',outcome,'executedFrames',numel(trace),'failure',failure, ...
                'minimumReplayClearanceMeters',minimumClearance,'configuration',cfg, ...
                'initialInformationContract',initial,'initialContractScope', ...
                "finiteExperimentalPriorIncludesActualInitialMember;notASensorDerivedEnclosure", ...
                'robustSafetyCertified',false,'trace',trace);
            localWriteJson(file,result);
            entry=rmfield(result,{'configuration','trace','initialInformationContract'});entry.source=file;
            if isempty(summaries),summaries=entry;else,summaries(end+1)=entry;end %#ok<AGROW>
        end
    end
    campaign=struct('scope',"joint dynamic output feedback with finite uncertainty scenarios", ...
        'pipeline',"dynamicOutputFeedback",'results',summaries);
    localWriteJson(fullfile(options.OutputDirectory,'campaign.json'),campaign);
end

function hypotheses=localInitialContract(x,q)
    % This is an experimental prior, not an estimate derived from truth at
    % every decision. Common translation and friction remain correlated.
    prototype=struct('ego',x,'target',q,'parameters',ones(4,1),'sensorBias',zeros(7,1));
    hypotheses=repmat(prototype,1,3);
    for j=1:3
        offset=(j-2)*.005;
        hypotheses(j).ego(1:2)=x(1:2)+[offset;offset];
        hypotheses(j).ego(4)=x(4)+offset;
        hypotheses(j).target(1:2)=q(1:2)+[offset;offset];
        hypotheses(j).parameters(3:4)=1+offset;
    end
end

function y=localSensor(truth,input,cfg,time,stream,dropout)
    y=solvePredictiveControl('outputFeedback',struct('operation',"sensor",'cfg',cfg, ...
        'hypothesis',truth,'input',input,'time',time));
    bounds=cfg.outputFeedback.noiseBounds;
    y.position=y.position+localNoise(stream,bounds(1));
    y.acceleration=y.acceleration+localNoise(stream,bounds(2));
    y.yawRate=y.yawRate+.5*bounds(3)*(2*rand(stream)-1);
    y.relativePosition=y.relativePosition+localNoise(stream,bounds(4));
    if dropout,y.radarAvailable=false;y.relativePosition=[NaN;NaN];end
end

function noise=localNoise(stream,radius)
    angle=2*pi*rand(stream);noise=.5*radius*sqrt(rand(stream))*[cos(angle);sin(angle)];
end

function [truth,samples,clearance]=localReplay(truth,input,cfg,time,stream,dropout)
    h=cfg.controller.sampleTime;dt=cfg.outputFeedback.sensorPeriod;count=round(h/dt);
    start=truth;ode=ode45(@(~,x)nonlinearBicycleModel.derivative(x,input,cfg),[0,h],truth.ego, ...
        odeset('RelTol',1e-10,'AbsTol',1e-12));
    samples=struct([]);
    for j=1:count
        truth.ego=deval(ode,j*dt);
        if j==count,truth.ego=nonlinearBicycleModel.sampleEndState(truth.ego,input,cfg);end
        if start.target(4)>0,truth.target=predictiveSafetyGeometry.predictTarget(start.target,j*dt);end
        y=localSensor(truth,input,cfg,time+j*dt,stream,dropout);
        if isempty(samples),samples=y;else,samples(end+1)=y;end %#ok<AGROW>
    end
    clearance=Inf;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    for t=linspace(0,h,41)
        x=deval(ode,t);q=start.target;
        if q(4)>0,q=predictiveSafetyGeometry.predictTarget(q,t);end
        gap=predictiveSafetyGeometry.rectangle(x(1:3),shape,q(1:3),q(8:11));
        clearance=min(clearance,gap);
    end
end

function localWriteJson(path,value)
    file=fopen(path,'w');assert(file>=0);cleanup=onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(value,PrettyPrint=true));
end
