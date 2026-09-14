classdef controllerRepairTest < matlab.unittest.TestCase
    % Regressions for the September 14 declared-plant simulation failures.
    properties (TestParameter)
        errorBound = struct('speedOnly',[0;0;0;.05;0;0], ...
            'fullEgo',[.05;.05;.005;.05;.02;.005]);
    end
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'solver','clarabel','matlab')));
        end
    end
    methods (Test)
        function uncertainTerminalVerticesPreserveSpeedAndInputSlew(testCase)
            audit = localTerminalVertices();
            testCase.verifyTrue(audit.membershipHeld);
            testCase.verifyGreaterThanOrEqual(audit.minimumSpeed,-1e-12);
            testCase.verifyGreaterThanOrEqual(audit.minimumVelocityMargin,-1e-10);
            testCase.verifyLessThanOrEqual(audit.maximumSlewViolation,1e-12);
        end

        function longUncertainBackupRetainsTruthAndNonnegativeSpeed(testCase,errorBound)
            report = runExactStateRecursiveFeasibilityScenario(Scenario='crossing', ...
                SampleCount=300,Seed=20260914,FailAfterAdmission=true,EgoErrorBound=errorBound);
            testCase.verifyTrue(report.completed,report.failureMessage);
            testCase.verifyTrue(report.passed);
            testCase.verifyTrue(report.modelDomainHeld);
            testCase.verifyTrue(report.truthContained);
            testCase.verifyGreaterThanOrEqual(report.minimumSampledSpeed,0);
        end

        function stationaryAdmissionFindsACertifiedPassingPlan(testCase)
            [ego,target,road,cfg] = localScene("stationary");
            [~,~,problem,certificate] = collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyTrue(problem.metadata.safetyCertified);
            testCase.verifyEqual(certificate.value,0,AbsTol=0);
            testCase.verifyGreaterThan(max(abs(certificate.predictedState(2,:))),2);
            testCase.verifyGreaterThan(certificate.predictedState(1,end)-certificate.predictedState(1,1),33.4);
        end

        function aPositiveValueAttemptCanTryAnotherPassingSide(testCase)
            [ego,target,road,cfg] = localScene("stationary");
            [~,~,problem] = collisionAvoidanceController(ego,target,road,cfg,[]);
            model = problem.model;
            model.confirmation.passingRequired = false;
            [~,~,~,~,check,timing,failure] = hardEncounterBarrier.plan(model);
            testCase.verifyEmpty(char(failure));
            testCase.verifyTrue(check.safetyCertified);
            testCase.verifyGreaterThan(timing.attempts,1);
        end

        function anApproachingExteriorTargetKeepsItsEncounterWitness(testCase)
            [ego,target,road,cfg] = localScene("oncoming");
            [~,~,problem,certificate] = collisionAvoidanceController(ego,target,road,cfg,[]);
            ego = encounterTestFixture.nextEgo(certificate,problem.model.lane);
            target.targetPositionInertial = target.targetPositionInertial+cfg.controller.sampleTime*target.targetVelocityInertial;
            cfg.solver.jointFunction = @encounterTestFixture.fail;
            [~,~,next,carried] = collisionAvoidanceController(ego,target,road,cfg,certificate);
            testCase.verifyTrue(next.metadata.hasTarget);
            testCase.verifyFalse(next.metadata.confirmedRelease);
            testCase.verifyTrue(next.metadata.candidateVerified);
            testCase.verifyEqual(carried.completion.deadline,certificate.completion.deadline,AbsTol=0);
            testCase.verifyEqual(next.metadata.certificateSource,"carriedWitness");
        end

        function anExpiredFrameStartsNoFreshSolverWork(testCase)
            [ego,target,road,cfg] = encounterTestFixture.crossing();
            [~,~,problem,certificate] = collisionAvoidanceController(ego,target,road,cfg,[]);
            ego = encounterTestFixture.nextEgo(certificate,problem.model.lane);
            target.targetPositionInertial = target.targetPositionInertial+cfg.controller.sampleTime*target.targetVelocityInertial;
            cfg.solver.frameDeadlineSeconds = 1e-6;
            cfg.solver.jointFunction = @encounterTestFixture.fail;
            [command,~,next] = collisionAvoidanceController(ego,target,road,cfg,certificate);
            testCase.verifyEqual(command.actuatorInput,certificate.plan(:,2),AbsTol=0);
            testCase.verifyEqual(next.metadata.certificateSearchAttempts,0);
            testCase.verifyTrue(next.metadata.frameDeadlineHit);
        end

        function exteriorApproachUsesCurrentEgoVelocity(testCase)
            [ego,target,road,cfg] = encounterTestFixture.crossing();
            [~,~,problem] = collisionAvoidanceController(ego,target,road,cfg,[]);
            model = problem.model;
            model.initialEgoState(4) = 1;
            [position,~] = laneGeometry.fromFrenet(model.initialEgoState,model.lane);
            model.encounters.center(1:4) = [position+[-30;0];2;0];
            confirmation = hardEncounterBarrier.admitConfirmation(ego,model);
            testCase.verifyEqual(confirmation.exitDirection,[1;0],AbsTol=1e-12);
            testCase.verifyTrue(confirmation.passingRequired);
        end

        function nativeTimeBudgetPreservesTheOrdinarySolution(testCase)
            [legacy,legacyInfo] = localNativeBudget([]);
            [budgeted,budgetedInfo] = localNativeBudget(1);
            testCase.verifyEqual(legacyInfo.status,1);
            testCase.verifyEqual(budgetedInfo.status,1);
            testCase.verifyEqual(budgeted,legacy,AbsTol=1e-8);
            testCase.verifyEqual(budgeted,2,AbsTol=1e-7);
        end

        function anExpiredNativeBudgetReturnsATimeLimitStatus(testCase)
            [~,information] = localNativeBudget(0);
            testCase.verifyEqual(information.status,8);
        end

        function aCertifiedFlagCannotOverrideANegativeTrueSpeed(testCase)
            report = localAuditRecord();
            report.minimumSampledSpeed = -0.01;
            report.minimumSampledModelDomainMargin = -0.01;
            assessment = assessExactStateSafety(report);
            testCase.verifyFalse(assessment.modelDomainHeld);
            testCase.verifyFalse(assessment.passed);
        end

        function aCertifiedFlagCannotOverrideLossOfTruthContainment(testCase)
            report = localAuditRecord();
            report.egoContainmentMargin(2) = -0.001;
            assessment = assessExactStateSafety(report);
            testCase.verifyFalse(assessment.truthContained);
            testCase.verifyFalse(assessment.passed);
        end

        function aNewAdmissionStartsItsOwnDescentObligation(testCase)
            report = localAuditRecord();
            report.admissionFrame(2) = true;
            report.candidateVerified(2) = false;
            report.descentResidual(2) = NaN;
            assessment = assessExactStateSafety(report);
            testCase.verifyTrue(assessment.passed);
        end

        function oldNominalOnlyTerminalCertificatesAreRejected(testCase)
            [ego,target,road,cfg] = encounterTestFixture.crossing();
            [~,~,problem,certificate] = collisionAvoidanceController(ego,target,road,cfg,[]);
            ego = encounterTestFixture.nextEgo(certificate,problem.model.lane);
            certificate.version = 21;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,certificate), ...
                'collisionAvoidanceController:invalidStoredCertificate');
        end
    end
end

function [ego,target,road,cfg] = localScene(scene)
    [ego,target,route] = encounterTestFixture.crossing();
    cfg = collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'controller',struct('sampleTime',.1,'horizonSteps',16), ...
        'model',struct('lateralDomainRadius',4),'solver',struct('certificateSearchTimeLimit',15)));
    target.targetPositionInertial = [15;0];
    target.targetVelocityInertial = [0;0];
    target.targetHeadingInertial = 0;
    if scene=="oncoming"
        target.targetPositionInertial = [60;0];
        target.targetVelocityInertial = [-8;0];
        target.targetHeadingInertial = pi;
    end
    boundary = struct('origin',zeros(2,1),'longitudinalDirection',[1;0], ...
        'lateralDirection',[0;1],'coefficients',[0;0;-5], ...
        'parameterRange',[-100;2000],'safeSideSign',1);
    boundaries = [boundary;boundary];
    boundaries(2).coefficients(3) = 5;
    boundaries(2).safeSideSign = -1;
    road = struct('centerline',route,'boundaries',boundaries);
end

function audit = localTerminalVertices()
    [ego,target,road,cfg] = encounterTestFixture.crossing();
    ego.controllerStateErrorBound = [.05;.05;.005;.05;.02;.005];
    [~,~,~,certificate] = collisionAvoidanceController(ego,target,road,cfg,[]);
    terminal = certificate.terminal;
    center = certificate.predictedState(:,end);
    radius = certificate.stateErrorBound(:,end);
    signs = (2*double(dec2bin(0:63,6)-'0')-1).';
    truth = center+radius.*signs;
    generator = [terminal.continuousA,terminal.continuousB,terminal.continuousC;zeros(3,9)];
    halfFlow = expm(.5*cfg.controller.sampleTime*generator);
    fullFlow = expm(cfg.controller.sampleTime*generator);
    previous = certificate.plan(:,end);
    rate = cfg.controller.sampleTime*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
    audit = struct('membershipHeld',true,'minimumSpeed',inf, ...
        'minimumVelocityMargin',inf,'maximumSlewViolation',0);
    for index = 1:600
        audit.membershipHeld = audit.membershipHeld && hardEncounterBarrier.terminalMembership(terminal,center,radius);
        step = hardEncounterBarrier.terminalStep(terminal,center,radius,cfg.controller.sampleTime);
        midpoint = halfFlow*[truth;repmat(step.input,1,64);ones(1,64)];
        endpoint = fullFlow*[truth;repmat(step.input,1,64);ones(1,64)];
        audit.minimumSpeed = min([audit.minimumSpeed,midpoint(4,:),endpoint(4,:)]);
        audit.minimumVelocityMargin = min([audit.minimumVelocityMargin; ...
            reshape(terminal.velocityLimit-abs([midpoint(4:6,:),endpoint(4:6,:)]),[],1)]);
        audit.maximumSlewViolation = max([audit.maximumSlewViolation;abs(step.input-previous)-rate]);
        truth = endpoint(1:6,:);
        center = step.successor;
        radius = step.successorRadius;
        previous = step.input;
    end
end

function report = localAuditRecord()
    report = struct('completed',true,'planCertified',[true,true], ...
        'candidateVerified',[false,true],'admissionFrame',[true,false], ...
        'descentResidual',[NaN,0],'configuration',collisionAvoidanceControllerConfig(), ...
        'minimumSampledModelDomainMargin',0.1,'minimumSampledSpeed',1, ...
        'egoContainmentMargin',[0,0],'targetContainmentMargin',[0,inf], ...
        'minimumSampledSeparationMargin',1,'minimumSampledRoadMargin',1,'maximumSlewViolation',0);
end

function [decision,information] = localNativeBudget(seconds)
    options = [1e-8,1e-8,100,seconds];
    [decision,information] = solveAvoidanceSocpMex(sparse(1),-2,sparse(-1),0,[0;1],options);
end
