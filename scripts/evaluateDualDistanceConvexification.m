function summary = evaluateDualDistanceConvexification(fixtureDirectory,outputDirectory)
%evaluateDualDistanceConvexification Audit Li-style distance-dual proposals.
% This offline experiment retains full rectangular footprints, whole-hold
% collision rows, the soft CLF and the existing terminal/exit conditions.
% A known certified plan is an explicit initialized comparison, not an
% independent first-admission solution. Production control is unchanged.
    arguments
        fixtureDirectory (1,1) string
        outputDirectory (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    cases=["oncomingAdmission","stationaryAdmission"];
    summary=struct('source',"Li et al. (2023), DOI 10.1007/s42154-023-00222-7, equations 12-13", ...
        'scope',"Offline frozen-frame adaptation, no closed-loop or full-frame latency claim", ...
        'matlabVersion',string(version),'cases',struct());
    for name=cases
        saved=load(fullfile(fixtureDirectory,name+'.mat'),'current');
        fixture=saved.current;
        % Re-admit the frozen measured state; never reinterpret a certificate
        % produced by an earlier controller format as a current witness.
        fixture.previous=[];
        [~,~,problem]=collisionAvoidanceController(fixture.ego,fixture.target, ...
            fixture.road,fixture.cfg,fixture.previous);
        model=problem.model;cfg=model.cfg;
        baseline=problem.program;prediction=problem.prediction;
        anchors={repmat(baseline.cruiseCertificate.input,prediction.stageCount,1), ...
            problem.decision(baseline.layout.planIndex)};
        trials=struct([]);anchorNames=["cruise","certified"];
        for mode=1:2
            anchor=anchors{mode};timer=tic;
            [normals,dual]=localDistanceDirections(model,prediction,baseline,anchor);
            dualSeconds=toc(timer);
            trial=struct('anchor',anchorNames(mode), ...
                'dualSeconds',dualSeconds,'minimumDualNorm',min(dual.normalNorm), ...
                'overlappingMidpoints',nnz(dual.signedDistance<=0), ...
                'degenerateDirections',nnz(dual.normalNorm<1e-6), ...
                'maximumDualPrimalGap',max(abs(dual.distance-max(dual.signedDistance,0))), ...
                'formulationSeconds',0,'solveSeconds',0,'feasible',false,'certified',false, ...
                'anchorPhysicalViolation',NaN,'objective',NaN,'rows',0);
            candidate=[];result=[];
            if trial.degenerateDirections==0
                timer=tic;
                candidate=localReplaceCollisionRows(model,prediction,baseline,anchor,normals);
                trial.formulationSeconds=toc(timer);
                witness=[anchor;problem.decision(end)];
                trial.anchorPhysicalViolation=max(candidate.physicalMatrix*witness-candidate.physicalBound);
                timer=tic;result=solveHardCbfClf.constrained(candidate,cfg);
                trial.solveSeconds=toc(timer);trial.feasible=result.feasible;
                trial.rows=size(candidate.A,1);
                if result.feasible
                    candidate=solveHardCbfClf.certify(candidate,result.decision);
                    trial.certified=true;
                    trial.objective=.5*result.decision.'*candidate.P*result.decision+candidate.q.'*result.decision;
                end
            end
            trials=[trials;trial]; %#ok<AGROW>
            save(fullfile(outputDirectory,name+'-'+trial.anchor+'.mat'), ...
                'fixture','problem','anchor','dual','normals','candidate','result','trial');
            fprintf('%s %s: overlap %d, degenerate %d, certified %d, dual %.3f ms, solve %.3f ms\n', ...
                name,trial.anchor,trial.overlappingMidpoints,trial.degenerateDirections, ...
                trial.certified,1e3*dualSeconds,1e3*trial.solveSeconds);
        end
        summary.cases.(name)=trials;
    end
    save(fullfile(outputDirectory,'summary.mat'),'summary');
    file=fopen(fullfile(outputDirectory,'summary.json'),'w');assert(file>=0);
    cleanup=onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(summary,PrettyPrint=true));
end

function [normals,information]=localDistanceDirections(model,prediction,program,anchor)
% Solve the paper's nonnegative distance dual on the exact configuration
% polygon at each hold midpoint. Its outside distance agrees with the
% independent signed rectangle-distance routine. Interior distance is zero.
    cfg=model.cfg;cells=prediction.cells;
    assert(isscalar(model.encounters),'The frozen comparison has one target.');
    target=model.encounters(1);frames=program.geometry.frames;
    count=numel(cells);normals=cell(count,1);
    information=struct('distance',zeros(count,1),'signedDistance',zeros(count,1), ...
        'normalNorm',zeros(count,1),'multipliers',zeros(8,count),'status',zeros(count,1));
    axes=[1,0,-1,0;0,1,0,-1];
    for index=1:count
        tube=cells(index);frame=frames(index);degree=size(tube.offset,2)-1;
        weights=arrayfun(@(k)nchoosek(degree,k),0:degree).'/2^degree;
        points=reshape(pagemtimes(tube.map,anchor),6,[])+tube.offset;
        state=points*weights;
        egoPosition=frame.origin+[frame.tangent,frame.lateral]*state(1:2);
        egoYaw=frame.heading+state(3);
        center=targetPrediction.finiteFlow(target,tube.start+tube.duration/2);
        egoRotation=[cos(egoYaw),-sin(egoYaw);sin(egoYaw),cos(egoYaw)];
        targetRotation=[cos(center(7)),-sin(center(7));sin(center(7)),cos(center(7))];
        matrix=[egoRotation*axes,targetRotation*axes].';
        egoSupport=abs(matrix*egoRotation)*[cfg.vehicle.length/2;cfg.vehicle.width/2];
        targetSupport=abs(matrix*targetRotation)*[target.halfLength;target.halfWidth];
        bound=matrix*center(1:2)+egoSupport+targetSupport;
        residual=matrix*egoPosition-bound;
        [lambda,status]=solveAvoidanceSocpMex(sparse(8,8),-residual, ...
            sparse([-eye(8);zeros(1,8);-matrix.']),[zeros(8,1);1;0;0],[0;8;3], ...
            [cfg.solver.constraintTolerance,cfg.solver.optimalityTolerance,cfg.solver.maxIterations]);
        assert(any(status.status==[1,4]),'Distance dual did not solve.');
        normal=matrix.'*lambda;length=norm(normal);
        normals{index}=normal/max(length,realmin);
        information.distance(index)=residual.'*lambda;
        information.normalNorm(index)=length;
        information.multipliers(:,index)=lambda;
        information.status(index)=status.status;
        information.signedDistance(index)=avoidanceSafetyGeometry.rectangleDistance( ...
            egoPosition,egoYaw,center(1:2),center(7), ...
            [cfg.vehicle.length/2;cfg.vehicle.width/2;target.halfLength;target.halfWidth]);
    end
end

function candidate=localReplaceCollisionRows(model,prediction,baseline,anchor,normals)
    model.anchorPlan=anchor;prediction.geometryAnchor=anchor;
    prediction.geometryFrames=baseline.geometry.frames;
    prediction.geometryNominal=cell(numel(prediction.cells),1);
    for index=1:numel(prediction.cells)
        tube=prediction.cells(index);
        prediction.geometryNominal{index}=reshape(pagemtimes(tube.map,anchor),6,[])+tube.offset;
    end
    prediction.separationNormals=normals;
    geometry=avoidanceSafetyGeometry.build(model,prediction);
    assert(all(startsWith(geometry.label,"collision:")),'This comparison has no roads.');
    keep=~startsWith(baseline.physicalLabels,"collision:");
    matrix=[geometry.matrix;baseline.physicalMatrix(keep,baseline.layout.planIndex)];
    physicalBound=[geometry.physicalBound;baseline.physicalBound(keep)];
    scale=1+abs(geometry.physicalBound)+abs(geometry.matrix)*baseline.decisionRadius;
    reserve=4*max(model.cfg.encounter.numericalMargin,model.cfg.solver.constraintTolerance) ...
        *scale.*any(geometry.matrix~=0,2);
    bound=[geometry.physicalBound-reserve;baseline.safetyBound(keep)];
    candidate=baseline;
    candidate.physicalMatrix=[matrix,zeros(numel(bound),1)];
    candidate.physicalBound=physicalBound;candidate.safetyBound=bound;
    candidate.physicalLabels=[geometry.label;baseline.physicalLabels(keep)];
    coneStart=baseline.cones(2)+1;
    candidate.A=sparse([candidate.physicalMatrix;zeros(1,baseline.layout.planCount),-1; ...
        baseline.A(coneStart:end,:)]);
    candidate.b=[bound;0;baseline.b(coneStart:end)];
    candidate.cones(2)=numel(bound)+1;
    candidate.geometry=geometry;candidate.anchorPlan=anchor;
    candidate.obstacleCbfRowCount=nnz(startsWith(geometry.label,"collision:"));
end
