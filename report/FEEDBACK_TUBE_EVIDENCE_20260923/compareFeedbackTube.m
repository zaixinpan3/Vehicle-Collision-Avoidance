function compareFeedbackTube(outputFolder)
%compareFeedbackTube Open-loop error box versus a feedback tube on captured frames.
% Offline estimate only: e(k+1) = (A_k + B_k K) e(k) + B_k K eta(k), with the
% estimation error eta bounded by the same declared box at every hold.
    root = pwd;
    addpath(fullfile(root,"scripts"),fullfile(root,"controller"),fullfile(root,"config"));
    egoBase = [.01;.01;.001;.01;.01;.001];targetBase = [.1;.1;.05;.05;.01;.01;.01;.01];
    specs = {"39 stationary straight x10","stationary",0,10;"42 stationary k0.01 x10","stationary",.01,10; ...
        "53 crossing k0.01 x3","crossing",.01,3;"crossing k0.01 x1","crossing",.01,1};
    for index = 1:size(specs,1)
        k = specs{index,4};
        if isappdata(0,"diagnosticFrame"),rmappdata(0,"diagnosticFrame");end
        try
            runExactStateRecursiveFeasibilityScenario(Scenario=specs{index,2},RoadCurvature=specs{index,3}, ...
                SampleCount=1,DeadlineSeconds=Inf,SearchTimeLimitSeconds=30, ...
                OutputDirectory=fullfile(outputFolder,sprintf("tube%02d",index)), ...
                EgoErrorBound=k*egoBase,TargetErrorBound=k*targetBase,TargetJerkAmplitude=k*[.02;.02]);
        catch
        end
        frame = getappdata(0,"diagnosticFrame");model = frame.model;
        [program,~,~] = formulateAvoidanceProblem(model);
        prediction = program.prediction;terminal = program.terminal;
        epsilon = model.initialFrenetErrorBound;count = prediction.stageCount;
        full = terminal.feedback;if size(full,2)~=6,g = zeros(2,6);g(:,terminal.stateIndex) = full;full = g;end
        modal = zeros(size(terminal.modalMatrix,1),6);modal(:,terminal.stateIndex) = terminal.modalMatrix;
        variants = {"open loop",0,true;"LQR x1.0",1,true;"LQR x0.5",.5,true;"LQR x0.25",.25,true;"LQR x1.0 no noise",1,false};
        for v = 1:size(variants,1)
            gain = variants{v,2}*full;noisy = variants{v,3};
            G = diag(epsilon);maxInput = zeros(2,1);
            for s = 1:count
                A = prediction.stageMatrixA(:,:,s);B = prediction.stageMatrixB(:,:,s);
                noise = zeros(6,0);if noisy,noise = diag(epsilon);end
                maxInput = max(maxInput,sum(abs(gain*[G,noise]),2));
                G = [(A+B*gain)*G,B*gain*noise];
            end
            box = sum(abs(G),2);
            support = sum(abs(modal*G),2);
            margin = terminal.radius(:)-terminal.reserve-support;
            fprintf("TUBE %-28s %-18s | terminal lateral %.3f m heading %.4f rad | best-case modal margin %.4f | feedback input use steer %.4f rad braking %.4f\n", ...
                specs{index,1},variants{v,1},box(2),box(3),min(margin),maxInput(1),maxInput(2));
        end
    end
end
