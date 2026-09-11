function summary = evaluateFialaFeedbackSequence(outputDirectory)
%evaluateFialaFeedbackSequence Reproduce correlated finite-policy experiments.
% Bounds are in each coordinate's SI unit. Replays do not establish inclusion.
    arguments
        outputDirectory (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    nativeDirectory=fullfile(outputDirectory,'native');
    buildFialaIntervalVerifier(nativeDirectory);addpath(nativeDirectory);
    cfg=collisionAvoidanceControllerConfig(struct('controller',struct('sampleTime',.1)));
    state=[0;0;0;10;0;0];
    trim=longitudinalRoadLoad(10,cfg)/cfg.vehicle.m/modifiedFialaTire.accelerationGain(cfg);
    originalGain=[0,-.3,-1,0,-.03,-.03;-.2,0,0,-.3,0,0];
    radii=[.001,.005,.001,.01,.01];steering=[0,0,.04,0,0];
    gainFactors=[1,1,1,.25,1];budgets=[Inf,Inf,Inf,Inf,.08];
    sequences=cell(1,numel(radii));rows=cell(size(sequences));
    for index=1:numel(radii)
        inputs=repmat([0;trim],1,50);inputs(1,1:3)=steering(index);
        gain=originalGain;gain(1,:)=gainFactors(index)*gain(1,:);
        sequence=certifyFialaFeedbackSequence(state-radii(index),state+radii(index),state, ...
            inputs,gain,radii(index)/10*ones(6,1),inputs(:,1),cfg, ...
            maximumComputationTimePerSample=budgets(index));
        sequences{index}=sequence;
        times=cellfun(@(c)c.wallSeconds,sequence.samples);
        last=sequence.samples{end};
        row=struct('inletRadius',radii(index),'measurementRadius',radii(index)/10, ...
            'steeringPulseRad',steering(index),'lateralGainFactor',gainFactors(index), ...
            'accepted',sequence.accepted,'completedSamples',sequence.completedSamples, ...
            'requestedSeconds',5,'samplePeriodSeconds',.1,'lastReason',string(last.reason), ...
            'wallSecondsMinimum',min(times),'wallSecondsMedian',median(times), ...
            'wallSecondsMaximum',max(times),'finalLateralRadius',diff(last.endpoint(2,:))/2, ...
            'maximumGeneratorCount',max(cellfun(@(c)size(c.generators,2),sequence.samples)), ...
            'sampledViolation',localReplay(sequence,state,radii(index),inputs(:,1),cfg));
        rows{index}=row;disp(row);
    end
    summary=struct('scope',"Finite prescribed ego policies; no encounter safety or worst-case timing claim", ...
        'samplingMethod',"64 deterministic cosine inlet/measurement cases, 20 RK4 steps per cell; no RNG", ...
        'configuration',cfg,'originalGain',originalGain,'cases',[rows{:}]);
    save(fullfile(outputDirectory,'feedback-sequence.mat'),'summary','sequences');
    file=fopen(fullfile(outputDirectory,'feedback-sequence.json'),'w');cleanup=onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(summary,PrettyPrint=true));
end

function violation=localReplay(sequence,state,radius,previous,cfg)
    points=state+radius*cos((1:6).'*(1:64)*sqrt(2));violation=0;
    for sample=1:sequence.completedSamples
        certificate=sequence.samples{sample};
        noise=radius/10*cos((1:6).'*(1:64)*sqrt(2)+sample);
        commands=certificate.feedbackNominal(7:8)+certificate.feedbackGain* ...
            (points-certificate.feedbackNominal(1:6)+noise);
        change=commands-previous;
        violation=max([violation;reshape(certificate.inputChange(:,1)-change,[],1); ...
            reshape(change-certificate.inputChange(:,2),[],1)]);
        for index=1:numel(certificate.cells)
            cell=certificate.cells(index);dt=(cell.end-cell.start)/20;
            for step=1:20
                violation=max([violation;reshape(cell.swept(1:6,1)-points,[],1); ...
                    reshape(points-cell.swept(1:6,2),[],1)]);
                first=ltvBicycleModel.fialaWorldDynamics(points,commands,cfg);
                second=ltvBicycleModel.fialaWorldDynamics(points+dt*first/2,commands,cfg);
                third=ltvBicycleModel.fialaWorldDynamics(points+dt*second/2,commands,cfg);
                fourth=ltvBicycleModel.fialaWorldDynamics(points+dt*third,commands,cfg);
                points=points+dt*(first+2*second+2*third+fourth)/6;
            end
            violation=max([violation;reshape(cell.endpoint(1:6,1)-points,[],1); ...
                reshape(points-cell.endpoint(1:6,2),[],1)]);
        end
        previous=commands;
    end
end
