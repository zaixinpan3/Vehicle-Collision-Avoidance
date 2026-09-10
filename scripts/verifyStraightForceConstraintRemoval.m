function result = verifyStraightForceConstraintRemoval(inputDirectory, outputDirectory)
%verifyStraightForceConstraintRemoval Compare one actual failed LP with current admission.
% inputDirectory contains the September 9 straight-scene MAT exports. The
% old polygon configuration field is removed only from these archived input
% copies. Current runtime configuration still rejects that obsolete field.
% No physical plant is advanced and no other model allowance is changed.
    arguments
        inputDirectory (1,1) string
        outputDirectory (1,1) string
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    original = load(fullfile(inputDirectory,'straight-realtime-validation.mat'),'report');
    captured = load(fullfile(inputDirectory,'captured-program.mat'),'capturedProgram');
    cfg = original.report.controllerOnly.controllerConfiguration;
    cfg.model = rmfield(cfg.model,'frictionPolygonSides');
    context = original.report.controllerOnly.failureContext;
    [command,~,problem,certificate] = collisionAvoidanceController(context.controllerState, ...
        context.targetEstimate,context.controllerRoadGeometry,cfg,[]);
    oldMatrix = full(captured.capturedProgram.A(1:end-2,1:end-1));
    oldBound = captured.capturedProgram.b(1:end-2);
    newMatrix = problem.qp.inequalityMatrix;
    newBound = problem.qp.barrier.baseBound;
    % Version 13 has 12 domain, 4 slip, 24 force and 8 curb rows
    % per Bernstein point in this saved two-curb, target-free scene.
    trailingRows = size(newMatrix,1)-size(problem.qp.geometry.matrix,1);
    oldGeometryRows = size(oldMatrix,1)-trailingRows;
    oldMask = [repmat([true(16,1);false(24,1);true(8,1)],oldGeometryRows/48,1); ...
        true(trailingRows,1)];
    oldIndex = find(oldMask);
    matrixDifference = max(abs(newMatrix-oldMatrix(oldMask,:)),[],'all');
    boundDifference = max(abs(newBound-oldBound(oldMask)));
    % Recompiling the shorter row kernel can change final floating sums.
    assert(matrixDifference<1e-12 && boundDifference==0, ...
        'Only force rows may be removed; remaining coefficients must agree within roundoff.');
    opts = optimoptions('linprog','Display','none');
    [~,~,oldFlag] = linprog(zeros(size(oldMatrix,2),1),oldMatrix,oldBound,[],[],[],[],opts);
    [~,~,newFlag] = linprog(zeros(size(newMatrix,2),1),newMatrix,newBound,[],[],[],[],opts);
    result = struct('oldHardLpExitFlag',oldFlag,'currentHardLpExitFlag',newFlag, ...
        'oldHardRowCount',size(oldMatrix,1),'currentHardRowCount',size(newMatrix,1), ...
        'removedRowCount',size(oldMatrix,1)-size(newMatrix,1), ...
        'remainingMatrixMaximumDifference',matrixDifference,'remainingBoundMaximumDifference',boundDifference, ...
        'combinedForceRows',nnz(problem.qp.geometry.label=="combinedTireForce"), ...
        'firstFrameCertified',problem.metadata.planCertified,'margin',certificate.margin, ...
        'certificateVersion',certificate.version,'issuedInput',command.actuatorInput, ...
        'residualRateBound',cfg.model.plantModelResidualRateBound, ...
        'scope',"Declared inclusion admission only; no physical-plant guarantee");
    assert(oldFlag==-2 && newFlag==1 && result.firstFrameCertified && result.combinedForceRows==0);
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    save(fullfile(outputDirectory,'row-removal-comparison.mat'), ...
        'result','oldIndex','command','problem','certificate','-v7.3');
    file = fopen(fullfile(outputDirectory,'row-removal-summary.json'),'w');
    assert(file>=0);
    cleanup = onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(result,PrettyPrint=true));
    disp(result);
end
