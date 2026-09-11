classdef continuousTimeClfTest < matlab.unittest.TestCase
    % Independent continuous-flow checks of the predictive dissipation bound.
    properties (TestParameter)
        invalidRateFraction = {0, -0.1, 1.01, NaN, Inf, [0.1, 0.2], 1i};
        referenceRate = {[0;0;0;0;0], [0.02;0.001;0.3;0.01;0.001]};
        certificateReferenceSpeed = struct("aboveFloor",15,"belowFloor",3);
    end
    methods (TestClassSetup)
        function addControllerPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "controller")));
        end
    end
    methods (Test)
        function theDecayFractionMustBeFiniteAndWithinItsCertifiedRange(testCase, invalidRateFraction)
            testCase.verifyError(@() collisionAvoidanceControllerConfig( ...
                struct("clf", struct("decreaseRateFraction", invalidRateFraction))), ...
                "collisionAvoidanceController:invalidConfiguration");
        end

        function theRiccatiCertificateHasAPositiveContinuousDecreaseRate(testCase)
            [problem, cfg] = localProblem(zeros(5, 1));
            certificate = problem.qp.clf.certificate;
            [a, b] = localCertificateMatrices(certificate,cfg);
            closedLoop = a(2:6, 2:6)-b(2:6, :)*certificate.feedbackGain;
            derivative = closedLoop.'*certificate.lyapunovMatrix+certificate.lyapunovMatrix*closedLoop;
            testCase.verifyEqual(derivative, -certificate.decreaseMatrix, AbsTol=1e-10);
            testCase.verifyGreaterThan(min(eig(certificate.lyapunovMatrix)), 0);
            testCase.verifyGreaterThan(min(eig(certificate.decreaseMatrix-problem.qp.clf.decayRate*certificate.lyapunovMatrix)), 0);
        end

        function theCertificateRecordsItsActualSynthesisOperatingPoint(testCase, certificateReferenceSpeed)
            [problem, cfg] = localProblem(zeros(5, 1));
            model = problem.model;
            model.referenceSpeed = certificateReferenceSpeed;
            model.cfg.referenceSpeed = certificateReferenceSpeed;
            qp = formulateAvoidanceProblem(model,problem.prediction,problem.inputPlan(:));
            certificate = qp.clf.certificate;
            speed = max(certificateReferenceSpeed,cfg.clf.certificateSpeedFloor);
            [a,b,c] = localCertificateMatrices(certificate,cfg);
            flow = a*certificate.operatingState+b*certificate.operatingInput+c;
            closedLoop = a(2:6,2:6)-b(2:6,:)*certificate.feedbackGain;

            testCase.verifyEqual(certificate.operatingState,[0;0;0;speed;0;0],AbsTol=0);
            testCase.verifyEqual(certificate.operatingCurvature,0,AbsTol=0);
            testCase.verifyEqual(certificate.operatingAccelerationBias,model.longitudinalAccelerationBias,AbsTol=0);
            testCase.verifyEqual(flow,[speed;zeros(5,1)],AbsTol=1e-12);
            testCase.verifyEqual(closedLoop.'*certificate.lyapunovMatrix ...
                +certificate.lyapunovMatrix*closedLoop,-certificate.decreaseMatrix,AbsTol=1e-10);
        end

        function predictionSeedsDoNotChangeTheInputCostCenter(testCase)
            [problem, ~] = localProblem(zeros(5, 1));
            prediction = problem.prediction;
            anchor = problem.inputPlan(:);
            first = formulateAvoidanceProblem(problem.model,prediction,anchor);
            prediction.referencePlan = prediction.referencePlan+repmat([0.02;0.1],prediction.stageCount,1);
            second = formulateAvoidanceProblem(problem.model,prediction,anchor);

            testCase.verifyEqual(second.Hessian,first.Hessian,AbsTol=0);
            testCase.verifyEqual(second.linear,first.linear,AbsTol=0);
            testCase.verifyEqual(second.constant,first.constant,AbsTol=0);
            testCase.verifyEqual(second.stageProgram.P,first.stageProgram.P,AbsTol=0);
            testCase.verifyEqual(second.stageProgram.q,first.stageProgram.q,AbsTol=0);
        end

        function everyHeldIntervalBoundsDissipationWithReferenceDerivatives(testCase, referenceRate)
            [problem, cfg] = localProblem(referenceRate);
            [residual, integrated, derivativeError] = localContinuousResiduals(problem, cfg);
            testCase.verifyLessThanOrEqual(residual, 1e-9);
            testCase.verifyLessThanOrEqual(integrated, 1e-9);
            testCase.verifyLessThanOrEqual(derivativeError, 1e-7);
            testCase.verifyEqual(numel(problem.metadata.clfRelaxation), problem.prediction.stageCount);
        end

        function safetyAcceptanceRejectsInsufficientPredictiveSlack(testCase)
            [problem, ~] = localProblem(zeros(5, 1));
            decision = problem.decision;
            decision(problem.layout.relaxationIndex) = 0;
            rejected = solveHardCbfClf.certify(problem.qp, problem.prediction, problem.model, decision);
            testCase.verifyFalse(rejected.accepted);
            testCase.verifyTrue(any(rejected.failedConditions == "sampledDataClf"));
            testCase.verifyEqual(rejected.hardRowViolation, 0);
            testCase.verifyGreaterThan(max(problem.metadata.clfRelaxation), 0);
        end

        function aConcaveDissipationPeakInsideACellIsStillBounded(testCase)
            [qp, result, peakResidual, endpointResidual] = localConcaveCell();
            testCase.verifyTrue(result.feasible);
            testCase.verifyGreaterThan(peakResidual, endpointResidual+1e-5);
            testCase.verifyGreaterThanOrEqual(result.decision(qp.layout.relaxationIndex), peakResidual);
        end

        function changingClfDecayPreservesAllHardSafetyConstraints(testCase)
            [problem, cfg] = localProblem(zeros(5, 1));
            model = problem.model;
            model.cfg.clf.decreaseRateFraction = cfg.clf.decreaseRateFraction/2;
            first = formulateAvoidanceProblem(problem.model,problem.prediction,problem.inputPlan(:));
            second = formulateAvoidanceProblem(model, problem.prediction, problem.inputPlan(:));
            testCase.verifyEqual(first.inequalityMatrix, second.inequalityMatrix, AbsTol=0);
            testCase.verifyEqual(first.physicalBound, second.physicalBound, AbsTol=0);
            testCase.verifyEqual(problem.qp.clf.lyapunovMatrix, second.clf.lyapunovMatrix, AbsTol=0);
            testCase.verifyEqual(problem.qp.clf.decayRate, 2*second.clf.decayRate, AbsTol=1e-12);
        end
    end
end

function [a,b,c] = localCertificateMatrices(certificate,cfg)
    [a,b,c] = ltvBicycleModel.continuousMatrices(certificate.operatingCurvature, ...
        certificate.operatingState(4),cfg,[],certificate.operatingAccelerationBias, ...
        struct("state",certificate.operatingState,"input",certificate.operatingInput));
end

function [problem, cfg] = localProblem(referenceRate)
    cfg = collisionAvoidanceControllerConfig(struct("referenceSpeed",3,"controller", struct("horizonSteps",8,"sampleTime",0.1,"stationTrustRadius",5), ...
        "clf", struct("referenceRate", referenceRate)));
    ego = struct("position", [10;0.15], "yaw", 0.005, "speed", 2, ...
        "stateTime", 2, "perception",struct("time",2,"range",30,"completeWithinRange",true), ...
        "longitudinalAccelerationBias", 0, ...
        "controllerStateErrorBound", zeros(6,1));
    [~, ~, problem] = collisionAvoidanceController(ego, encounterTestFixture.stationaryTarget(), [0,0;2000,0], cfg, []);
end

function [worst, integrated, derivativeError] = localContinuousResiduals(problem, cfg)
    prediction = problem.prediction;
    p = problem.qp.clf.lyapunovMatrix;
    rate = problem.qp.clf.decayRate;
    initial = problem.model.initialEgoState;
    radius = problem.model.initialFrenetErrorBound;
    signs = 2*double(dec2bin(0:63, 6).'-'0')-1;
    states = initial+radius.*signs;
    disturbance = cfg.model.plantModelResidualRateBound.*signs;
    worst = -inf;
    integrated = -inf;
    derivativeError = 0;
    for stage = 1:prediction.stageCount
        a = prediction.continuousA(:, :, stage);
        b = prediction.continuousB(:, :, stage);
        c = prediction.continuousC(:, stage);
        input = problem.inputPlan(:, stage);
        reference = problem.qp.clf.referenceStart+cfg.clf.referenceRate*(stage-1)*cfg.controller.sampleTime;
        initialError = states(2:6, :)-reference;
        initialValue = sum(initialError.*(p*initialError), 1);
        slack = problem.metadata.clfRelaxation(stage);
        forcing = b*input+c+disturbance;
        for time = linspace(0, cfg.controller.sampleTime, 31)
            transition = expm(time*[a, eye(6); zeros(6, 12)]);
            state = transition(1:6, 1:6)*states+transition(1:6, 7:12)*forcing;
            error = state(2:6, :)-reference-cfg.clf.referenceRate*time;
            derivative = a*state+forcing;
            value = sum(error.*(p*error), 1);
            dissipation = 2*sum(error.*(p*(derivative(2:6, :)-cfg.clf.referenceRate)), 1);
            worst = max(worst, max(dissipation+rate*value-slack));
            bound = exp(-rate*time)*initialValue+(1-exp(-rate*time))*slack/rate;
            integrated = max(integrated, max(value-bound));
            step = 1e-6;
            forward = error+step*(derivative(2:6, :)-cfg.clf.referenceRate);
            backward = error-step*(derivative(2:6, :)-cfg.clf.referenceRate);
            finiteDifference = (sum(forward.*(p*forward),1)-sum(backward.*(p*backward),1))/(2*step);
            derivativeError = max(derivativeError, max(abs(finiteDifference-dissipation)));
        end
        states = state;
    end
end

function [qp,result,peak,endpoint] = localConcaveCell()
% Isolate the interval proof on a stable affine scalar channel whose CLF
% residual is concave and peaks strictly between the two sample endpoints.
    cfg = collisionAvoidanceControllerConfig(struct("referenceSpeed",0, ...
        "controller",struct("horizonSteps",1,"sampleTime",0.02)));
    ego = struct("position",[0;0],"yaw",0,"speed",0.5,"stateTime",0);
    [~,lane,road] = readPlanningInputs(ego,[],[-100,0;100,0],cfg);
    x = [100;0;0;0.5;0;0];
    a = zeros(6); a(4,4) = -10;
    b = zeros(6,2); c = [0;0;0;10;0;0];
    dt = cfg.controller.sampleTime;
    tube = stateUncertainty.flowTube(a,b,c,zeros(6,2),x,zeros(6,1),zeros(6,1), ...
        dt,6,[200;12;0.4;18;12;5],[1;1]);
    tube.stage=1; tube.start=0; tube.duration=dt; tube.time=(0:7)*dt/7;
    prediction = struct("stageCount",1,"nodeCount",2,"planCount",2, ...
        "referencePlan",zeros(2,1),"egoStateMatrix",cat(3,zeros(6,2),tube.endMap), ...
        "egoStateOffset",[x,tube.endOffset],"egoStateErrorBound",zeros(6,2),"cells",tube,"continuousA",a, ...
        "continuousB",b,"continuousC",c,"scheduleSpeedProfile",[0.5,0.5], ...
        "scheduleCurvature",0,"scheduleBrakingRatio",0);
    model = struct("cfg",cfg,"lane",lane,"road",road,"horizonSteps",1,"stateTime",0, ...
        "sampleTime",dt,"referenceSpeed",0,"initialEgoState",x,"encounters",[], ...
        "previousInput",[0;0], "initialFrenetErrorBound",zeros(6,1), ...
        "requiredMargin",0,"exitMargin",inf,"longitudinalAccelerationBias",0);
    qp = formulateAvoidanceProblem(model,prediction,zeros(2,1));
    result = solveHardCbfClf.solve(qp,cfg);
    time = linspace(0,dt,2001);
    velocity = 1-0.5*exp(-10*time);
    residual = qp.clf.lyapunovMatrix(3,3)*(2*velocity.*(10-10*velocity)+qp.clf.decayRate*velocity.^2);
    peak = max(residual);
    endpoint = max(residual([1,end]));
end
