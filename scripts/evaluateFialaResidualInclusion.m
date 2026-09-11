function result = evaluateFialaResidualInclusion(outputDirectory, priorGeometryDirectory)
%evaluateFialaResidualInclusion Verify nonzero Fiala remainders and one held cell.
% Bounds come from directed interval arithmetic, never sampled maxima.
% Optional prior affine-certificate audit is diagnostic, not a controller run.
    arguments
        outputDirectory (1,1) string
        priorGeometryDirectory (1,1) string = ""
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    buildFialaIntervalVerifier(fullfile(outputDirectory,'native'));
    addpath(fullfile(outputDirectory,'native'));
    cfg=collisionAvoidanceControllerConfig();state=[0;0;0;10;0;0];
    input=[0;longitudinalRoadLoad(10,cfg)/cfg.vehicle.m/modifiedFialaTire.accelerationGain(cfg)];
    [a,b,c]=ltvBicycleModel.continuousMatrices(0,10,cfg,input(2),0,struct('state',state,'input',input));
    center=[state;input];radius=[.001;.001;.001;.03;.01;.01;.005;.003];
    scales=[1,.5,.25,.125];bounds=zeros(6,numel(scales));seconds=zeros(size(scales));certificates=cell(size(scales));
    for index=1:numel(scales)
        timer=tic;
        certificates{index}=fialaResidualCertificate(center-scales(index)*radius,center+scales(index)*radius,a,b,c,cfg);
        seconds(index)=toc(timer);bounds(:,index)=certificates{index}.residualRateBound;
    end
    domainRadius=[.1;.01;.01;.2;.02;.02;.02;.03];
    feedback=struct('inletLower',state-1e-4,'inletUpper',state+1e-4, ...
        'nominal',center,'gain',[0,-.3,-1,0,-.03,-.03;-.2,0,0,-.3,0,0], ...
        'measurementRadius',1e-5*ones(6,1),'duration',.005);
    cellCertificate=fialaResidualCertificate(center-domainRadius,center+domainRadius,a,b,c,cfg,feedback);
    assert(cellCertificate.heldCell.accepted);
    feedback.duration=.5;
    rejected=fialaResidualCertificate(center-domainRadius,center+domainRadius,a,b,c,cfg,feedback);
    assert(~rejected.heldCell.accepted);
    result=struct('scope',"domain-wide residual certificates and a single held-feedback cell; no encounter safety claim", ...
        'stateInputCenter',center,'domainRadius',radius,'scales',scales, ...
        'residualRateBounds',bounds,'seconds',seconds,'feedbackCell',cellCertificate, ...
        'oversizedCellRejected',~rejected.heldCell.accepted);
    audit=[];
    if strlength(priorGeometryDirectory)>0
        prior=load(fullfile(priorGeometryDirectory,'continuous-separation.mat'),'stored');
        stored=prior.stored;prediction=stored.prediction;model=stored.model;
        assert(all(abs(prediction.continuousA(1,2,:))<1e-12));
        plan=stored.decision(stored.qp.layout.planIndex);
        audit=struct('bounds',zeros(6,prediction.stageCount),'pointResidual',zeros(6,2,prediction.stageCount));
        for stage=1:prediction.stageCount
            cells=prediction.cells([prediction.cells.stage]==stage);points=[];
            for k=1:numel(cells)
                points=[points,reshape(pagemtimes(cells(k).map,plan),6,[])+cells(k).offset]; %#ok<AGROW>
            end
            command=plan(2*stage-1:2*stage);padding=[.005;.005;.001;.01;.005;.005];
            lower=[min(points,[],2)-padding;command-.001];upper=[max(points,[],2)+padding;command+.001];
            aa=prediction.continuousA(:,:,stage);bb=prediction.continuousB(:,:,stage);cc=prediction.continuousC(:,stage);
            domain=fialaResidualCertificate(lower,upper,aa,bb,cc,model.cfg);
            point=[points(:,1);command];pointCheck=fialaResidualCertificate(point,point,aa,bb,cc,model.cfg);
            audit.bounds(:,stage)=domain.residualRateBound;audit.pointResidual(:,:,stage)=pointCheck.residual;
        end
        result.oncoming=struct('maximumDomainResidual',max(audit.bounds,[],2), ...
            'maximumPointResidualMagnitude',max(abs(audit.pointResidual),[],[2,3]), ...
            'scope',"prior affine oncoming witness in its straight Cartesian-equivalent chart; no tube closure assumed");
    end
    save(fullfile(outputDirectory,'inclusion.mat'),'result','certificates','audit','cfg');
    file=fopen(fullfile(outputDirectory,'inclusion.json'),'w');assert(file>=0);
    cleanup=onCleanup(@()fclose(file));fprintf(file,'%s\n',jsonencode(result,PrettyPrint=true));
    disp(bounds);disp(result.feedbackCell.heldCell);if isfield(result,'oncoming'),disp(result.oncoming);end
end
