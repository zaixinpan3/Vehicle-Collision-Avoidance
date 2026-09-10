function result = profileNativeAvoidanceSolver(inputDirectory, outputDirectory)
%profileNativeAvoidanceSolver Separate native setup, solve and bridge costs.
% Builds an instrumented COPY in the output directory; production is unchanged.
% Replays saved target-free first-admission data, not the physical plant.
    arguments
        inputDirectory (1,1) string
        outputDirectory (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    source=fileread(fullfile(root,'controller','solveAvoidanceSocpMex.cpp'));
    source=replace(source,'#include <cmath>',sprintf('#include <cmath>\n#include <chrono>'));
    source=replace(source,'    const auto hessian =',sprintf('    const auto begin = std::chrono::steady_clock::now();\n    const auto hessian ='));
    source=replace(source,'    auto solver =',sprintf('    const auto setupBegin = std::chrono::steady_clock::now();\n    auto solver ='));
    source=replace(source,'    clarabel_DefaultSolver_solve(solver);',sprintf(['    const auto solveBegin = std::chrono::steady_clock::now();\n' ...
        '    clarabel_DefaultSolver_solve(solver);\n    const auto solveEnd = std::chrono::steady_clock::now();']));
    source=replace(source,'    clarabel_DefaultSolver_free(solver);',sprintf(['    clarabel_DefaultSolver_free(solver);\n' ...
        '    const auto finish = std::chrono::steady_clock::now();\n' ...
        '    const char* timingNames[] = {"inputSeconds", "setupSeconds", "iterationCallSeconds", "outputAndFreeSeconds"};\n' ...
        '    const double timings[] = {std::chrono::duration<double>(setupBegin-begin).count(),\n' ...
        '        std::chrono::duration<double>(solveBegin-setupBegin).count(),\n' ...
        '        std::chrono::duration<double>(solveEnd-solveBegin).count(),\n' ...
        '        std::chrono::duration<double>(finish-solveEnd).count()};\n' ...
        '    for (int i = 0; i < 4; ++i) { mxAddField(result[1], timingNames[i]);\n' ...
        '        mxSetField(result[1], 0, timingNames[i], mxCreateDoubleScalar(timings[i])); }']));
    sourcePath=fullfile(outputDirectory,'profileAvoidanceNativeMex.cpp');
    file=fopen(sourcePath,'w');assert(file>=0);fprintf(file,'%s',source);fclose(file);
    dependency=fullfile(root,'solver','clarabel');
    binary=fullfile(outputDirectory,'profileAvoidanceNativeMex.'+string(mexext));
    clear profileAvoidanceNativeMex
    if isfile(binary),delete(binary);end
    inspectionMessage="";
    try
        mex('-R2018a',"-I"+fullfile(dependency,'include'),sourcePath, ...
            fullfile(dependency,'rust_wrapper','target','release','libclarabel_c.a'), ...
            '-ldl','-lpthread','-lm','-outdir',outputDirectory);
    catch exception
        % Match the host's known post-link inspection issue. Fresh binary
        % execution and exact decision comparisons below remain mandatory.
        if string(exception.identifier)~="MATLAB:mex:Error" || ~contains(exception.message,"is not a MEX file")
            rethrow(exception);
        end
        inspectionMessage=string(exception.message);
    end
    rehash;
    addpath(outputDirectory);cleanup=onCleanup(@()rmpath(outputDirectory));
    loaded=load(fullfile(inputDirectory,'original-functional.mat'),'unboundedRuntime');
    trial=loaded.unboundedRuntime;
    ego=trial.attempts.controllerEgoEstimate{1};
    road=trial.attempts.roadPerception{1}.roadGeometry;
    target=trial.attempts.targetEstimate{1};cfg=trial.controllerConfiguration;
    cfg.solver.jointFunction=@capture;
    programs={};baseline={};
    [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
    assert(problem.metadata.planCertified && numel(programs)==2);
    raw=struct();result=struct('buildInspectionMessage',inspectionMessage,'scope',"same-data native replay, no plant or controller-policy change");
    for phase=1:2
        program=programs{phase};retained=true(numel(program.q),1);
        if isfield(program,'inactiveSlackIndex'),retained(program.inactiveSlackIndex)=false;end
        assert(~isfield(program,'fixedDecisionIndex'));
        p=program.P(retained,retained);q=program.q(retained);a=program.A(:,retained);b=program.b;
        options=[cfg.solver.constraintTolerance,cfg.solver.optimalityTolerance,cfg.solver.maxIterations];
        for replay=1:6
            timer=tic;
            [point,information]=profileAvoidanceNativeMex(p,q,a,b,program.cones,options);
            information.wallSeconds=toc(timer);
            information.decisionDifference=norm(point-baseline{phase}.decision(retained),inf);
            assert(information.status==baseline{phase}.output.status && information.decisionDifference<1e-8);
            samples(replay)=information; %#ok<AGROW>
        end
        stage=struct('variables',numel(q),'rows',size(a,1),'nonzeros',nnz(a), ...
            'secondOrderCones',numel(program.cones)-2,'samples',samples);
        if phase==1,name='hardMargin';else,name='performance';end
        result.(name)=stage;
        raw.(name)=struct('P',p,'q',q,'A',a,'b',b,'cones',program.cones,'options',options);
    end
    save(fullfile(outputDirectory,'native-profile.mat'),'result','raw');
    file=fopen(fullfile(outputDirectory,'native-summary.json'),'w');assert(file>=0);
    fileCleanup=onCleanup(@()fclose(file));fprintf(file,'%s\n',jsonencode(result,PrettyPrint=true));
    for name=["hardMargin","performance"]
        s=result.(name).samples(2:end);
        fprintf('%s: setup %.3f ms, solve call %.3f ms, input %.3f ms, output/free %.3f ms, total %.3f ms\n', ...
            name,1e3*median([s.setupSeconds]),1e3*median([s.iterationCallSeconds]), ...
            1e3*median([s.inputSeconds]),1e3*median([s.outputAndFreeSeconds]),1e3*median([s.wallSeconds]));
    end
    function solve=capture(~,program)
        solve=program.defaultSolver();
        programs{end+1}=rmfield(program,'defaultSolver');baseline{end+1}=solve;
    end
end
