function summary = evaluateFialaFeedbackSample(outputDirectory)
%evaluateFialaFeedbackSample Reproduce complete held-feedback sample checks.
% Numerical trajectories are falsification checks, not the inclusion proof.
    arguments
        outputDirectory (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    if ~isfolder(outputDirectory)
        mkdir(outputDirectory);
    end
    nativeDirectory=fullfile(outputDirectory,'native');
    buildFialaIntervalVerifier(nativeDirectory);
    addpath(nativeDirectory);
    cfg=collisionAvoidanceControllerConfig(struct('controller',struct('sampleTime',.1)));
    state=[0;0;0;10;0;0];
    trim=ltvBicycleModel.roadLoad(10,cfg)/cfg.vehicle.m/modifiedFialaTire.accelerationGain(cfg);
    gain=[0,-.3,-1,0,-.03,-.03;-.2,0,0,-.3,0,0];
    steering=[0,.04,-.04,0,.04,-.04,0,.04,-.04,0];
    radii=[repmat(1e-4,1,3),repmat(1e-3,1,3),repmat(1e-2,1,3),1e-4];
    maximumCellDuration=[repmat(.005,1,9),.1];
    certificates=cell(1,numel(steering));
    rows=cell(1,numel(steering));
    for index=1:numel(steering)
        input=[steering(index);trim];
        inletRadius=radii(index);measurementRadius=inletRadius/10;
        timer=tic;
        certificate=fialaCertificate.sample(state-inletRadius,state+inletRadius, ...
            [state;input],gain,measurementRadius*ones(6,1),input,cfg, ...
            maximumCellDuration=maximumCellDuration(index));
        elapsed=toc(timer);
        certificates{index}=certificate;
        row=struct('steeringRad',steering(index),'inletRadius',inletRadius, ...
            'measurementRadius',measurementRadius,'sampleTimeSeconds',.1, ...
            'maximumCellDurationSeconds',maximumCellDuration(index), ...
            'accepted',certificate.accepted,'reason',string(certificate.reason), ...
            'verifiedThroughSeconds',certificate.verifiedThrough, ...
            'cellCount',numel(certificate.cells),'rejectedCells',certificate.rejectedCells, ...
            'wallSeconds',elapsed,'endpointRadius',(certificate.endpoint(1:6,2)-certificate.endpoint(1:6,1))/2, ...
            'sampledContainmentViolation',[],'heldCommandPreserved',false);
        if certificate.accepted
            row.sampledContainmentViolation=localReplay(certificate,state,input,gain, ...
                inletRadius,measurementRadius,cfg);
            commands=cat(3,certificate.cells.endpoint);
            row.heldCommandPreserved=isequal(commands(7:8,:,:), ...
                repmat(certificate.initialBox(7:8,:),1,1,numel(certificate.cells)));
        end
        rows{index}=row;
    end
    summary=struct('scope',"One complete sample; no collision, road, encounter or real-time claim", ...
        'samplingMethod',"64 deterministic cosine points; 20 RK4 steps per certified cell; no random seed", ...
        'gain',gain,'configuration',cfg,'cases',[rows{:}]);
    save(fullfile(outputDirectory,'feedback-sample.mat'),'summary','certificates');
    file=fopen(fullfile(outputDirectory,'feedback-sample.json'),'w');
    cleanup=onCleanup(@() fclose(file));
    fprintf(file,'%s\n',jsonencode(summary,PrettyPrint=true));
    disp(struct2table(rmfield(summary.cases,{'endpointRadius','sampledContainmentViolation'})));
end

function violation=localReplay(certificate,state,input,gain,inletRadius,measurementRadius,cfg)
    phases=cos((1:12).'*(1:64)*sqrt(2));
    points=state+inletRadius*phases(1:6,:);
    noise=measurementRadius*phases(7:12,:);
    commands=input+gain*(points-state+noise);
    violation=0;
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
end
