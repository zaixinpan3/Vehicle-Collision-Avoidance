classdef collisionThreatScenarioTest < matlab.unittest.TestCase
    % Baseline danger is required independently of avoidance performance.
    properties (TestParameter)
        scenario = {"headOn","acceleratingHeadOn","brakingLead","stoppedLead","crossing", ...
            "turningCrossing","curvedHeadOn","curvedCrossing"};
        referenceSpeed = {8,15};
    end
    methods (TestClassSetup)
        function preparePaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function everyThreatStartsSeparatedAndCollidesWithoutAvoidance(testCase,scenario,referenceSpeed)
            [~,target,road,cfg] = collisionThreatScenario(scenario,struct('referenceSpeed',referenceSpeed));
            baseline = givenPathCollisionBaseline(road,target,cfg,8);
            testCase.verifyGreaterThan(baseline.initialClearanceMeters,0);
            testCase.verifyTrue(baseline.collisionDetected);
            testCase.verifyGreaterThan(baseline.strictOverlapSamples,0);
            testCase.verifyGreaterThan(baseline.maximumOverlapDepthMeters,.1);
            testCase.verifyGreaterThan(baseline.firstCollisionSeconds,0);
            testCase.verifyLessThan(baseline.firstCollisionSeconds,8);
        end
        function targetActuallyReachesTheCruisingEgo(testCase,scenario,referenceSpeed)
            [~,target,road,cfg,impactTime] = collisionThreatScenario(scenario,struct('referenceSpeed',referenceSpeed));
            baseline = givenPathCollisionBaseline(road,target,cfg,8);
            future = predictiveSafetyGeometry.predictTarget(target,impactTime);
            egoPosition = laneGeometry.referencePose(baseline.initialStationMeters+referenceSpeed*impactTime,0,baseline.referenceCurve);
            testCase.verifyEqual(future(1:2),egoPosition,AbsTol=1e-11);
        end
        function fastStoppedLeadRequiresMoreThanStraightMaximumBraking(testCase)
            [x,q,~,cfg]=collisionThreatScenario("stoppedLead", ...
                struct('referenceSpeed',25,'model',struct('speedMaximum',30)));
            shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
            minimum=Inf;
            for step=1:100
                x=nonlinearBicycleModel.sample(x,[0;-1],cfg);
                minimum=min(minimum,abs(q(1)-x(1))-shape(1)-q(8));
                if x(4)==0,break;end
            end
            testCase.verifyLessThan(minimum,0);
            testCase.verifyEqual(x(4),0);
            testCase.verifyGreaterThan(x(1),q(1)-cfg.vehicle.length);
        end
        function benignSideBrakingIsNotAdmitted(testCase)
            testCase.verifyError(@()runNonlinearPredictiveSafetyValidation( ...
                Scenarios="brakingTarget",Frames=160,RequireCollisionThreat=true), ...
                'runNonlinearPredictiveSafetyValidation:notCollisionThreat');
        end
        function touchingWithoutOverlapDoesNotQualifyAsActualDanger(testCase)
            cfg = collisionAvoidanceControllerConfig(struct('referenceSpeed',8));
            road = struct('centerline',[-100,0;1000,0]);
            target = [4.8;0;0;8;0;0;1.6;2.4;.95;0;0];
            baseline = givenPathCollisionBaseline(road,target,cfg,2);
            testCase.verifyFalse(baseline.collisionDetected);
            testCase.verifyEqual(baseline.strictOverlapSamples,0);
        end
        function noTargetIsNotAnAvoidanceThreat(testCase)
            cfg = collisionAvoidanceControllerConfig();
            baseline = givenPathCollisionBaseline(struct('centerline',[-100,0;1000,0]),[],cfg,2);
            testCase.verifyFalse(baseline.collisionDetected);
            testCase.verifyEqual(baseline.strictOverlapSamples,0);
        end
    end
end
