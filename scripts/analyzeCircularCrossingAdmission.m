function summary = analyzeCircularCrossingAdmission(outputDirectory)
%analyzeCircularCrossingAdmission Diagnose the scalar admission restriction.
% Offline diagnostics only: do not issue controls or change production code.
    arguments
        outputDirectory (1,1) string
    end
    originalPath = path;
    restorePath = onCleanup(@()path(originalPath));
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'), ...
        fullfile(root,'scripts'),fullfile(root,'solver','bicycle'), ...
        fullfile(root,'solver','clarabel','matlab'));
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    cfg = collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'controller',struct('sampleTime',.05,'horizonSteps',32,'minimumHorizonSteps',1), ...
        'model',struct('lateralDomainRadius',4), ...
        'solver',struct('frameDeadlineSeconds',30,'certificateSearchTimeLimit',30)));
    curve = struct('origin',[0;0],'heading',0,'curvature',.01,'length',340);
    road = struct('referenceCurve',curve, ...
        'centerline',laneGeometry.referencePose(linspace(0,340,201),0,curve).');
    [initial,trim] = ltvBicycleModel.cruiseEquilibrium(.01,cfg);
    [point,heading] = laneGeometry.referencePose(15,0,curve);
    normal = [-sin(heading);cos(heading)];
    egoMeasurement = struct('position',[0;0],'yaw',initial(3),'speed',initial(4), ...
        'lateralVelocity',initial(5),'yawRate',initial(6),'stateTime',0, ...
        'controllerStateErrorBound',zeros(6,1), ...
        'perception',struct('time',0,'range',16,'completeWithinRange',true));
    target = struct('trackId',1,'targetPositionInertial',point-7.5*normal, ...
        'targetVelocityInertial',4*normal,'targetAccelerationInertial',zeros(2,1), ...
        'targetHeadingInertial',heading+pi/2,'targetYawRate',0, ...
        'targetPositionInertialErrorBound',zeros(2,1),'targetVelocityInertialErrorBound',zeros(2,1), ...
        'targetAccelerationInertialErrorBound',zeros(2,1),'targetYawErrorBound',0, ...
        'targetYawRateErrorBound',0,'predictionMotion',struct('kind','finite-sensing-motion-v1', ...
        'jerkBound',zeros(2,1),'yawAccelerationBound',0));
    originalFailure = "";
    try
        collisionAvoidanceController(egoMeasurement,target,road,cfg,[]);
    catch exception
        assert(strcmp(exception.identifier,'collisionAvoidanceController:optimizationFailed'));
        originalFailure = string(exception.message);
    end
    assert(contains(originalFailure,'collisionExcluded'));
    [ego,lane,parsedRoad,observations] = readPlanningInputs(egoMeasurement,target,road,cfg);
    projection = laneGeometry.project(ego.position,lane);
    radius = stateUncertainty.toFrenet(ego.modelState,ego.stateErrorBound,lane);
    model = struct('cfg',cfg,'lane',lane,'road',parsedRoad,'stateTime',0,'sampleTime',.05, ...
        'horizonSteps',32,'referenceSpeed',8, ...
        'initialEgoState',[projection.station;projection.lateralPosition; ...
            atan2(sin(ego.yaw-projection.heading),cos(ego.yaw-projection.heading));ego.modelState(4:6)], ...
        'initialFrenetErrorBound',radius,'longitudinalAccelerationBias',0, ...
        'previousInput',zeros(2,1),'requiredMargin',0,'confirmation',[]);
    identity = struct('configuration',rmfield(cfg,'solver'),'lane',lane,'road',parsedRoad,'accelerationBias',0);
    model = hardEncounterBarrier.prepare(model,ego,observations,[],identity);
    [program,prediction,clf] = formulateAvoidanceProblem(model);
    [~,result,search] = solveHardCbfClf.fixedDirections(program,model,cfg);
    assert(~result.feasible && search.section.status=="collisionExcluded");
    % Expose local computations through a renamed disposable copy. The
    % production class is neither edited nor shadowed by the diagnostic class.
    source = fileread(fullfile(root,'controller','solveHardCbfClf.m'));
    source = strrep(source,'solveHardCbfClf','diagnosticSectionSolver');
    insertion = sprintf(['    methods (Static)\n' ...
        '        function [origin,states,direction,slope,residual] = sectionData(program,cfg)\n' ...
        '            origin=program.anchorPlan;states=localStates(program.prediction,origin);\n' ...
        '            [direction,residual]=localDirection(program,cfg,states);\n' ...
        '            slope=localDirections(program.prediction,direction);\n' ...
        '        end\n']);
    source = strrep(source,'    methods (Static)',insertion);
    needle = 'function [direction,residual]=localDirection(program,cfg,states)';
    source = strrep(source,needle,sprintf(['%s\n' ...
        '    if isfield(cfg,''diagnosticDirection''),direction=cfg.diagnosticDirection;residual=NaN;return;end'],needle));
    file = fopen(fullfile(outputDirectory,'diagnosticSectionSolver.m'),'w');assert(file>=0);
    fprintf(file,'%s',source);fclose(file);
    clear diagnosticSectionSolver
    addpath(outputDirectory);
    [origin,states,direction,slope,basisResidual] = diagnosticSectionSolver.sectionData(program,cfg);
    [decision,~,information] = diagnosticSectionSolver.admitSection(program,cfg);
    assert(isempty(decision) && isequaln(information,search.section));
    records = program.jointCertificate.records;
    values = avoidanceSafetyGeometry.jointResidual(program,program.feasibleWitness,program.jointCertificate.angles);
    selected = ~[records.isExit].' & values>program.jointCertificate.upperBound;
    stages = [records(selected).stage];
    samples = unique([min(stages),round((min(stages)+max(stages))/2),max(stages)]);
    physical = program.physicalMatrix(:,1:program.layout.planCount);
    coefficient = physical*direction;
    bound = program.safetyBound-physical*origin;
    lowerRows = find(coefficient<0);upperRows = find(coefficient>0);
    [lower,at] = max(bound(lowerRows)./coefficient(lowerRows));lowerRow = lowerRows(at);
    [upper,at] = min(bound(upperRows)./coefficient(upperRows));upperRow = upperRows(at);
    [gitStatus,sourceRevision] = system(sprintf('git -C "%s" rev-parse HEAD',root));
    assert(gitStatus==0,'Could not identify the source revision.');
    summary = struct('sourceRevision',string(strtrim(sourceRevision)), ...
        'originalFailure',originalFailure,'search',search, ...
        'horizonSteps',prediction.stageCount,'horizonSeconds',.05*prediction.stageCount, ...
        'controlVariables',program.layout.planCount,'recordCount',numel(records), ...
        'violatingStages',stages,'deformationStages',samples,'basisResidual',basisResidual, ...
        'deformationPositions',slope(1:2,samples+1),'terminalDeformation',slope(:,end), ...
        'initialInputDirection',direction(1:2),'terminalInputDirection',direction(end-1:end), ...
        'linearBase',[lower,upper],'lowerLimitingRow',program.physicalLabels(lowerRow), ...
        'upperLimitingRow',program.physicalLabels(upperRow), ...
        'lowerLimitingRowNumber',lowerRow,'upperLimitingRowNumber',upperRow, ...
        'finalBase',information.baseInterval,'maximumPositionBall',max([records.positionBall]), ...
        'maximumEgoYawRadius',max([records.egoYawRadius]), ...
        'maximumTargetYawRadius',max([records.targetYawRadius]), ...
        'dictionarySweeps',{{}});
    save(fullfile(outputDirectory,'diagnosis.mat'),'program','prediction','clf','model','cfg', ...
        'egoMeasurement','target','road','initial','trim','origin','states','direction','slope','summary');
    disp(summary);
    for normalCount = [32,128,512]
        for amplitudeCells = [16,64,256]
            trial = cfg;trial.admission.normalCount = normalCount;trial.admission.amplitudeCells = amplitudeCells;
            timer = tic;
            [candidate,~,info] = solveHardCbfClf.admitSection(program,trial);
            entry = struct('normals',normalCount,'cells',amplitudeCells,'status',info.status, ...
                'found',~isempty(candidate),'baseInterval',info.baseInterval,'seconds',toc(timer));
            summary.dictionarySweeps{end+1} = entry;
            fprintf('normals=%d cells=%d status=%s\n',normalCount,amplitudeCells,info.status);
        end
    end
    summary.sectionStudy = localSectionStudy(outputDirectory);
    summary.witnessStudy = localWitnessStudy(outputDirectory);
    summary.verification = localVerifyDiagnosis(outputDirectory);
    localWrite(fullfile(outputDirectory,'diagnosis.json'),summary);
    scan = load(fullfile(outputDirectory,'section-scan.mat'),'alpha','worst');
    exact = load(fullfile(outputDirectory,'exact-line.mat'),'grid','minGaps');
    plotData = struct('alpha',scan.alpha,'physicalNodeSatMargin',scan.worst, ...
        'certificateAlpha',exact.grid,'optimisticCertificateMargin',exact.minGaps, ...
        'time',(0:prediction.stageCount)*cfg.controller.sampleTime, ...
        'originalExtremeStates',states+information.baseInterval(1)*slope,'witnesses',{{}});
    for index = [4,7]
        saved = load(fullfile(outputDirectory,sprintf('shape-%02d-witness.mat',index)),'candidate');
        plan = saved.candidate(1:program.layout.planCount);
        values = prediction.egoStateOffset+reshape(pagemtimes(prediction.egoStateMatrix,plan),6,[]);
        plotData.witnesses{end+1} = struct('shapeIndex',index,'states',values,'inputs',reshape(plan,2,[]));
    end
    localWrite(fullfile(outputDirectory,'plot-data.json'),plotData);
end

function localWrite(file,value)
    handle = fopen(file,'w');assert(handle>=0);
    cleanup = onCleanup(@()fclose(handle));
    fprintf(handle,'%s\n',jsonencode(value,PrettyPrint=true));
end

function results = localSectionStudy(directory)
    d = load(fullfile(directory,'diagnosis.mat'));
    addpath(directory);
    p = d.program;cfg = d.cfg;
    n = p.layout.planCount;h = cfg.controller.sampleTime;
    alpha = linspace(d.summary.finalBase(1),d.summary.finalBase(2),20001);
    nodes = p.prediction.stageCount;
    physicalGap = zeros(nodes,numel(alpha));chartGap = physicalGap;
    for k = 1:nodes
        x = d.states(:,k+1)+d.slope(:,k+1)*alpha;
        [position,yaw] = laneGeometry.fromFrenet(x,d.road);
        targetPosition = d.target.targetPositionInertial+k*h*d.target.targetVelocityInertial;
        physicalGap(k,:) = localSat(position-targetPosition,yaw,d.target.targetHeadingInertial);
        rec = p.jointCertificate.records(k);
        chartPosition = rec.positionOffset+rec.positionMap*x;
        chartYaw = rec.yawOffset+rec.yawRow*x;
        chartGap(k,:) = localSat(chartPosition,chartYaw,rec.targetYaw)-rec.positionBall-rec.clearance;
    end
    [worst,stage] = min(physicalGap,[],1);[best,at] = max(worst);
    [chartWorst,chartStage] = min(chartGap,[],1);[chartBest,chartAt] = max(chartWorst);
    plan = d.origin+alpha(at)*d.direction;
    results = struct('physicalBestMinimumNodeSatGap',best,'physicalBestAlpha',alpha(at), ...
        'physicalLimitingStage',stage(at),'chartBestMinimumNodeSatGap',chartBest, ...
        'chartBestAlpha',alpha(chartAt),'chartLimitingStage',chartStage(chartAt), ...
        'numberPhysicalSafeSamples',nnz(worst>0),'numberChartSafeSamples',nnz(chartWorst>0), ...
        'samples',numel(alpha),'step',alpha(2)-alpha(1), ...
        'endpointMinima',worst([1,end]),'physicalDomain',d.summary.finalBase);
    % Isolate each base row family on the actual scalar line.
    a = p.physicalMatrix(:,1:n)*d.direction;b = p.safetyBound-p.physicalMatrix(:,1:n)*d.origin;
    labels = unique(p.physicalLabels);
    limits = cell(numel(labels),1);
    for k = 1:numel(labels)
        keep = p.physicalLabels==labels(k);
        interval = solveHardCbfClf.linearInterval(a(keep),b(keep),[-Inf,Inf]);
        limits{k} = struct('label',labels(k),'interval',interval,'rows',nnz(keep));
    end
    results.rowFamilies = limits;
    % Arbitrary angle choices for offline section construction isolate the
    % selected direction from production geometry and all hard constraints.
    maps = p.prediction.egoStateMatrix;samples = d.summary.deformationStages;
    constraint = [reshape(permute(maps(1:2,:,samples+1),[1,3,2]),[],n);maps(:,:,end);zeros(2,n)];
    constraint(end-1:end,end-1:end) = eye(2);
    scale = max(vecnorm(constraint,2,2),eps);constraint = constraint./scale;
    difference = speye(n)-spdiags(ones(n,1),-2,n,n);
    metric = spdiags(p.inputWeight,0,n,n)+cfg.encounter.inputRateWeight/h^2*(difference.'*difference);
    root = chol(metric);lift = root\pinv(full(constraint/root));
    angles = 0:15:165;directions = cell(size(angles));
    for k = 1:numel(angles)
        transverse = [cosd(angles(k));sind(angles(k))];
        desired = [repmat(transverse,numel(samples),1);zeros(8,1)]./scale;
        trial = cfg;trial.diagnosticDirection = lift*desired;
        [candidate,certAngles,info] = diagnosticSectionSolver.admitSection(p,trial);
        certified = false;
        if ~isempty(candidate)
            test = p;test.jointCertificate.angles = certAngles;
            solveHardCbfClf.certify(test,candidate);certified = true;
            save(fullfile(directory,sprintf('direction-%03d-witness.mat',angles(k))),'candidate','certAngles','test','info');
        end
        directions{k} = struct('degrees',angles(k),'status',info.status,'certified',certified, ...
            'amplitude',info.amplitude,'baseInterval',info.baseInterval);
        fprintf('Direction %d deg: %s; certified=%d\n',angles(k),info.status,certified);
    end
    results.directionAblation = directions;
    save(fullfile(directory,'section-scan.mat'),'alpha','physicalGap','chartGap','worst','stage','chartWorst','results','plan');
    fid = fopen(fullfile(directory,'section-scan.json'),'w');assert(fid>=0);
    fprintf(fid,'%s\n',jsonencode(results,PrettyPrint=true));fclose(fid);
    disp(results);
end

function gap = localSat(relative,yaw,targetYaw)
    e1 = [cos(yaw);sin(yaw)];e2 = [-sin(yaw);cos(yaw)];
    t1 = [cos(targetYaw);sin(targetYaw)];t2 = [-sin(targetYaw);cos(targetYaw)];
    a = abs(sum(relative.*e1,1))-2.4-2.4*abs(t1.'*e1)-.95*abs(t2.'*e1);
    b = abs(sum(relative.*e2,1))-.95-2.4*abs(t1.'*e2)-.95*abs(t2.'*e2);
    c = abs(t1.'*relative)-2.4-2.4*abs(t1.'*e1)-.95*abs(t1.'*e2);
    d = abs(t2.'*relative)-.95-2.4*abs(t2.'*e1)-.95*abs(t2.'*e2);
    gap = max([a;b;c;d],[],1);
end

function summary = localWitnessStudy(directory)
    d = load(fullfile(directory,'diagnosis.mat'));p = d.program;cfg = d.cfg;
    addpath(directory);
    n = p.layout.planCount;h = cfg.controller.sampleTime;
    assert(all([p.jointCertificate.records.egoYawRadius]==0) && all([p.jointCertificate.records.targetYawRadius]==0));
    summary = struct('exactLine',[],'chartSensitivity',{{}},'shapeSensitivity',{{}}, ...
        'maximumGeneratorRadius',max(arrayfun(@(r)sum(vecnorm(r.generators)),p.jointCertificate.records)));
    grid = linspace(d.summary.finalBase(1),d.summary.finalBase(2),2001);
    minGaps = zeros(size(grid));stages = minGaps;
    for k = 1:numel(grid)
        states = d.states+d.slope*grid(k);
        [gaps,~] = localGaps(p,states);
        [minGaps(k),stages(k)] = min(gaps(~[p.jointCertificate.records.isExit]));
    end
    [best,at] = max(minGaps);alpha = grid(at);
    [gaps,angles] = localGaps(p,d.states+d.slope*alpha);
    [~,limiting] = min(gaps);rec = p.jointCertificate.records(limiting);
    states = d.states+d.slope*alpha;
    [actualPosition,actualYaw] = laneGeometry.fromFrenet(states(:,rec.stage+1),d.road);
    targetPosition = d.target.targetPositionInertial+rec.stage*h*d.target.targetVelocityInertial;
    approximateRelative = rec.positionOffset+rec.positionMap*states(:,rec.stage+1);
    physicalGap = avoidanceSafetyGeometry.rectangleDistance(actualPosition,actualYaw,targetPosition,rec.targetYaw,[2.4;.95;2.4;.95]);
    summary.exactLine = struct('samples',numel(grid),'bestAlpha',alpha,'bestMinimumOptimisticCertificateGap',best, ...
        'limitingStage',limiting,'limitingTime',rec.stage*h,'actualBodyGapAtLimitingStage',physicalGap, ...
        'chartPositionErrorAtLimitingStage',norm(approximateRelative-(actualPosition-targetPosition)), ...
        'positionBall',rec.positionBall,'exitGap',gaps(end),'angleRadians',angles(limiting));
    % Finite alpha sampling plus a global per-node Lipschitz bound proves that
    % every unsampled amplitude also has an overlapping robust enclosure.
    lipschitz = zeros(numel(p.jointCertificate.records),1);
    for k = 1:numel(lipschitz)
        r = p.jointCertificate.records(k);dx = d.slope(:,r.stage+1);
        lipschitz(k) = norm(r.positionMap*dx)+norm(r.egoHalfSize)*abs(r.yawRow*dx);
    end
    upper = minGaps+reshape(lipschitz(stages),size(minGaps))*diff(grid(1:2))/2+1e-8;
    summary.exactLine.unsampledCertifiedGapUpperBound = max(upper);
    summary.exactLine.maximumNodeGapLipschitzConstant = max(lipschitz);
    summary.exactLine.entireLineExcluded = max(upper)<0;
    candidate = [d.origin+alpha*d.direction;1e6];
    baseStatus = solveHardCbfClf.inspect(p,candidate);
    summary.exactLine.baseInspectionStatus = baseStatus;
    candidateProgram = p;candidateProgram.jointCertificate.angles = angles;
    nominalAudit = localAudit(candidate,p,d);
    summary.exactLine.densePhysicalAudit = nominalAudit;
    save(fullfile(directory,'physical-section-witness.mat'),'candidate','candidateProgram','angles','alpha','nominalAudit');
    fprintf('Exact optimized direction gap %.9g; all-alpha bound %.9g; physical dense gap %.9g\n',best,max(upper),nominalAudit.minimumSampledSatGap);
    % Valid chart changes rebuild both hard domains and their error enclosures.
    for radius = [3.5,4,4.01,4.05,4.1,4.2,4.5,5]
        trial = cfg;trial.controller.poseTrustRadius(2) = radius;
        trialModel = d.model;trialModel.cfg = trial;
        [program,~,~] = formulateAvoidanceProblem(trialModel);
        [candidate,angles,info] = solveHardCbfClf.admitSection(program,trial);
        certified = false;
        if ~isempty(candidate)
            program.jointCertificate.angles = angles;
            solveHardCbfClf.certify(program,candidate);certified = true;
        end
        entry = struct('lateralTrustRadius',radius,'status',info.status,'baseInterval',info.baseInterval, ...
            'amplitude',info.amplitude,'certified',certified,'positionBall',max([program.jointCertificate.records.positionBall]));
        summary.chartSensitivity{end+1} = entry;
        fprintf('Rebuilt lateral chart radius %g: %s, certified=%d\n',radius,info.status,certified);
    end
    % Change only the selected deformation shape. All acceptance constraints,
    % dynamics, chart bounds, normal count, cells and terminal rows stay fixed.
    maps = p.prediction.egoStateMatrix;
    delta = d.summary.deformationPositions(:,1);samples = d.summary.deformationStages;
    constraint = [reshape(permute(maps(1:2,:,samples+1),[1,3,2]),[],n);maps(:,:,end);zeros(2,n)];
    constraint(end-1:end,end-1:end) = eye(2);
    scale = max(vecnorm(constraint,2,2),eps);constraint = constraint./scale;
    difference = speye(n)-spdiags(ones(n,1),-2,n,n);
    metric = spdiags(p.inputWeight,0,n,n)+cfg.encounter.inputRateWeight/h^2*(difference.'*difference);
    root = chol(metric);lift = root\pinv(full(constraint/root));
    weights = [1,1,1; .9,1,1;1,.9,1;1,1,.9;.8,1,1;1,1,.8;.8,1,.8;1,.8,1;.5,1,.5;1,1.2,1];
    for k = 1:size(weights,1)
        desired = [reshape(delta*weights(k,:),[],1);zeros(8,1)]./scale;
        trial = cfg;trial.diagnosticDirection = lift*desired;
        [candidate,angles,info] = diagnosticSectionSolver.admitSection(p,trial);
        certified = false;audit = struct();
        if ~isempty(candidate)
            program = p;program.jointCertificate.angles = angles;
            certifiedProgram = solveHardCbfClf.certify(program,candidate);certified = true;
            audit = localAudit(candidate,p,d);
            save(fullfile(directory,sprintf('shape-%02d-witness.mat',k)),'candidate','angles','certifiedProgram','info','audit');
        end
        summary.shapeSensitivity{end+1} = struct('weights',weights(k,:),'status',info.status, ...
            'amplitude',info.amplitude,'baseInterval',info.baseInterval,'certified',certified,'audit',audit);
        fprintf('Shape [%s]: %s; certified=%d\n',num2str(weights(k,:)),info.status,certified);
    end
    save(fullfile(directory,'exact-line.mat'),'grid','minGaps','stages','upper','summary');
    fid = fopen(fullfile(directory,'witness-probes.json'),'w');assert(fid>=0);
    fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);
end

function [gaps,angles] = localGaps(p,states)
    % Omit only the tiny nonnegative generator support for an OPTIMISTIC
    % upper bound: exclusion here also excludes the original larger sets.
    % Actual witness acceptance below always uses the unmodified verifier.
    records = p.jointCertificate.records;gaps = zeros(numel(records),1);angles = gaps;
    for k = 1:numel(records)
        r = records(k);x = states(:,r.stage+1);
        [distance,normal] = avoidanceSafetyGeometry.rectangleDistance(r.positionOffset+r.positionMap*x, ...
            r.yawOffset+r.yawRow*x,zeros(2,1),r.targetYaw,[r.egoHalfSize;r.targetHalfSize]);
        gaps(k) = distance-r.positionBall-r.clearance+p.jointCertificate.upperBound(k);
        angles(k) = atan2(normal(2),normal(1));
    end
end

function audit = localAudit(candidate,p,d)
    states = p.prediction.egoStateOffset+reshape(pagemtimes(p.prediction.egoStateMatrix,candidate(1:p.layout.planCount)),6,[]);
    h = d.cfg.controller.sampleTime;
    [a,b,c] = ltvBicycleModel.continuousMatrices(.01,8,d.cfg,[],0,struct('state',d.initial,'input',d.trim));
    generator = [a,b,c;zeros(3,9)];parts = 500;flows = zeros(9,9,parts+1);
    for j = 0:parts,flows(:,:,j+1) = expm(h*j/parts*generator);end
    minimum = Inf;when = NaN;nodeMinimum = Inf;endpointError = 0;
    inputs = reshape(candidate(1:p.layout.planCount),2,[]);
    for k = 1:p.prediction.stageCount
        origin = [states(:,k);inputs(:,k);1];
        % Refine the encounter region and use 5 ms elsewhere.
        if k>=20 && k<=60,indices=0:parts;else,indices=0:50:parts;end
        xs = zeros(6,numel(indices));
        for j = 1:numel(indices)
            value = flows(:,:,indices(j)+1)*origin;xs(:,j) = value(1:6);
        end
        endpointError = max(endpointError,max(abs(xs(:,end)-states(:,k+1))));
        times = (k-1)*h+indices*h/parts;
        [positions,yaws] = laneGeometry.fromFrenet(xs,d.road);
        targets = d.target.targetPositionInertial+d.target.targetVelocityInertial*times;
        independent = rectangleSeparationMargin(positions.',yaws.',targets.', ...
            repmat(d.target.targetHeadingInertial,numel(times),1),4.8,1.9,4.8,1.9);
        [gap,at] = min(independent);
        if gap<minimum,minimum=gap;when=times(at);end
        nodeMinimum = min([nodeMinimum;independent([1,end])]);
    end
    assert(endpointError<1e-9);
    audit = struct('minimumSampledSatGap',minimum,'time',when,'minimumNodeSatGap',nodeMinimum, ...
        'endpointError',endpointError,'denseStepSeconds',h/parts, ...
        'scope','Independent SAT; 0.1 ms in holds 20:60 and 5 ms elsewhere; physical sample only');
end

function verification = localVerifyDiagnosis(directory)
    d = load(fullfile(directory,'diagnosis.mat'));
    p = d.program;alpha = d.summary.finalBase(1);
    x = d.states+alpha*d.slope;
    [maximumLateral,at] = max(abs(x(2,:)-d.states(2,:)));
    verification = struct('originalMaximumLateralDeviation',maximumLateral, ...
        'originalLateralLimitStage',at-1,'originalLateralLimitTime',(at-1)*.05, ...
        'witnesses',{{}});
    for index = [4,6,7,10]
        data = load(fullfile(directory,sprintf('shape-%02d-witness.mat',index)));
        fields = {'physicalMatrix','physicalBound','terminalConePhysicalBound','layout'};
        for k = 1:numel(fields)
            assert(isequaln(p.(fields{k}),data.certifiedProgram.(fields{k})));
        end
        for name = {'stageMatrixA','stageMatrixB','stageAffine','egoStateMatrix','egoStateOffset'}
            assert(isequaln(p.prediction.(name{1}),data.certifiedProgram.prediction.(name{1})));
        end
        original = p;original.jointCertificate.angles = data.angles;
        solveHardCbfClf.certify(original,data.candidate);
        n = p.layout.planCount;
        states = p.prediction.egoStateOffset+reshape(pagemtimes(p.prediction.egoStateMatrix,data.candidate(1:n)),6,[]);
        input = reshape(data.candidate(1:n),2,[]);
        residual = avoidanceSafetyGeometry.jointResidual(original,data.candidate,data.angles);
        physicalExcess = max(p.physicalMatrix*data.candidate-p.physicalBound);
        cone = p.terminalConePhysicalBound-p.terminalCone.matrix*data.candidate(1:n);
        terminalMargin = min(cone(1:3:end)-vecnorm(reshape(cone(setdiff(1:numel(cone),1:3:numel(cone))),2,[])).');
        first = p.cones(2)+1;
        clf = p.b(first:first+5)-p.A(first:first+5,:)*data.candidate;
        [baseStatus,~,~] = solveHardCbfClf.inspect(p,data.candidate);
        assert(baseStatus==0 && physicalExcess<=0 && max(residual)<0 && terminalMargin>0);
        entry = struct('shapeIndex',index,'firstInput',input(:,1),'clfSlack',data.candidate(end), ...
            'minimumNodeCertificateGap',-max(residual),'maximumPhysicalExcess',physicalExcess, ...
            'minimumTerminalMargin',terminalMargin,'clfConeResidual',norm(clf(2:end))-clf(1), ...
            'maximumStationDeviation',max(abs(states(1,:)-d.states(1,:))), ...
            'maximumLateralDeviation',max(abs(states(2,:)-d.states(2,:))), ...
            'maximumHeadingDeviation',max(abs(states(3,:)-d.states(3,:))), ...
            'minimumSpeed',min(states(4,:)),'maximumSpeed',max(states(4,:)), ...
            'terminalDeviation',norm(states(:,end)-d.states(:,end),Inf), ...
            'unchangedPhysicalRowsAndDynamics',true,'passesOriginalIndependentVerifier',true);
        verification.witnesses{end+1} = entry;
        fprintf('Shape %d: verified unchanged original constraints, min certificate gap=%g m\n',index,entry.minimumNodeCertificateGap);
    end
    fprintf('Original lateral limit at %.3f s: %.9f m\n',verification.originalLateralLimitTime,maximumLateral);
    fid = fopen(fullfile(directory,'verification.json'),'w');assert(fid>=0);
    fprintf(fid,'%s\n',jsonencode(verification,PrettyPrint=true));fclose(fid);
end
