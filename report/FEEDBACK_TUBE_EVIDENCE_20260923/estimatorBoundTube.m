function estimatorBoundTube(outputFolder)
%estimatorBoundTube Open-loop box versus feedback tube with the recorded estimator ego bound.
% The estimator re-anchors its bound to fresh GNSS/IMU/gyro samples at every
% controller step, so the same per-channel bound is used as the estimation
% error at every future hold.
    root = pwd;
    addpath(fullfile(root,"scripts"),fullfile(root,"controller"),fullfile(root,"config"));
    estimatorBound = [0.076;0.076;0.048;0.089;0.497;0.0015];
    specs = {"stationary straight","stationary",0;"crossing curved 0.01","crossing",.01};
    for index = 1:size(specs,1)
        if isappdata(0,"diagnosticFrame"),rmappdata(0,"diagnosticFrame");end
        try
            runExactStateRecursiveFeasibilityScenario(Scenario=specs{index,2},RoadCurvature=specs{index,3}, ...
                SampleCount=1,DeadlineSeconds=Inf,SearchTimeLimitSeconds=30, ...
                OutputDirectory=fullfile(outputFolder,sprintf("estimator%02d",index)),EgoErrorBound=estimatorBound);
        catch
        end
        frame = getappdata(0,"diagnosticFrame");model = frame.model;
        [program,~,~] = formulateAvoidanceProblem(model);
        prediction = program.prediction;terminal = program.terminal;count = prediction.stageCount;
        epsilon = model.initialFrenetErrorBound;
        full = terminal.feedback;if size(full,2)~=6,g = zeros(2,6);g(:,terminal.stateIndex) = full;full = g;end
        modal = zeros(size(terminal.modalMatrix,1),6);modal(:,terminal.stateIndex) = terminal.modalMatrix;
        fprintf("EST %s: horizon %.2f s, estimator bound in controller coordinates [%s]\n", ...
            specs{index,1},count*model.sampleTime,num2str(epsilon.',3));
        noVy = full;noVy(:,5) = 0;
        variants = {"open loop (current)",zeros(2,6),"none";
            "LQR, independent error",full,"independent";
            "LQR, bias-like error",full,"bias";
            "no vy feedback, independent",noVy,"independent";
            "no vy feedback, bias-like",noVy,"bias"};
        for v = 1:size(variants,1)
            gain = variants{v,2};mode = variants{v,3};
            G = diag(epsilon);H = zeros(6,6);maxInput = zeros(2,1);maxLateral = 0;maxHeading = 0;stable = true;
            for s = 1:count
                A = prediction.stageMatrixA(:,:,s);B = prediction.stageMatrixB(:,:,s);
                switch mode
                    case "none"
                        G = A*G;
                    case "independent"
                        maxInput = max(maxInput,sum(abs(gain*[G,diag(epsilon)]),2));
                        G = [(A+B*gain)*G,B*gain*diag(epsilon)];
                    case "bias"
                        % eta_k = eta constant: its columns accumulate coherently.
                        maxInput = max(maxInput,sum(abs(gain*G),2)+sum(abs(gain*(H+diag(epsilon))),2));
                        G = (A+B*gain)*G;H = (A+B*gain)*H+B*gain*diag(epsilon);
                end
                all = [G,H];box = sum(abs(all),2);
                maxLateral = max(maxLateral,box(2));maxHeading = max(maxHeading,box(3));
            end
            if max(abs(eig(prediction.stageMatrixA(:,:,1)+prediction.stageMatrixB(:,:,1)*gain)))>=1,stable = false;end
            margin = min(terminal.radius(:)-terminal.reserve-sum(abs(modal*[G,H]),2));
            fprintf("EST   %-28s | max lateral %.3f m, max heading %.4f rad | terminal margin %.4f | feedback use steer %.4f rad braking %.4f | stage-1 closed loop stable %d\n", ...
                variants{v,1},maxLateral,maxHeading,margin,maxInput(1),maxInput(2),stable);
        end
    end
end
