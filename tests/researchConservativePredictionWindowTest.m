classdef researchConservativePredictionWindowTest < matlab.unittest.TestCase
    properties (TestParameter)
        speed = {0, .1, 2.5, 8, 17.5}
        prefix = {0, 4, 16}
    end
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, 'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, 'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, 'scripts')));
        end
    end
    methods (Test)
        function theAllocatedTailIncludesTheLatestDeclaredStop(testCase, speed, prefix)
            cfg = collisionAvoidanceControllerConfig();
            window = researchConservativePredictionWindow(cfg, speed, prefix, [.86; .86], 4);
            context = struct('sampleTime', cfg.controller.sampleTime, 'deceleration', 4);
            entry = [0; 0; .2; window.entrySpeedUpperMetersPerSecond; 0; .1];
            path = terminalSafeSet.continuation(context, entry, window.tailSteps);
            testCase.verifyGreaterThanOrEqual(path.time(end), path.speed/4);
            testCase.verifyEqual(path.arc(end), path.speed^2/8, AbsTol=1e-11);
            testCase.verifyLessThanOrEqual(window.tailSteps, window.domainTailSteps);
        end
        function aRestingEntryNeedsNoMovingTail(testCase)
            cfg = collisionAvoidanceControllerConfig();
            window = researchConservativePredictionWindow(cfg, 0, 0, [.85; .85], 4);
            testCase.verifyEqual(window.tailSteps, 0);
        end
        function aSubHoldStopIsRoundedUp(testCase)
            cfg = collisionAvoidanceControllerConfig();
            window = researchConservativePredictionWindow(cfg, .01, 0, [.85; .85], 4);
            testCase.verifyEqual(window.tailSteps, 1);
        end
        function moreUncertaintyNeverShortensTheTail(testCase)
            cfg = collisionAvoidanceControllerConfig();
            narrow = researchConservativePredictionWindow(cfg, 14, 4, [.85; .85], 4);
            broad = researchConservativePredictionWindow(cfg, 15, 4, [1; 1], 3);
            testCase.verifyGreaterThanOrEqual(broad.tailSteps, narrow.tailSteps);
        end
        function highCurrentSpeedLengthensTheWindowWithoutAnotherSolve(testCase)
            cfg = collisionAvoidanceControllerConfig();
            slow = researchConservativePredictionWindow(cfg, 8, 4, [.85; .85], 4);
            fast = researchConservativePredictionWindow(cfg, 17, 4, [.85; .85], 4);
            testCase.verifyGreaterThan(fast.tailSteps, slow.tailSteps);
        end
        function aLongPrefixStillUsesTheFiniteDomainSpeedCap(testCase)
            cfg = collisionAvoidanceControllerConfig();
            window = researchConservativePredictionWindow(cfg, 8, 100000, [.85; .85], 4);
            testCase.verifyEqual(window.tailSteps, window.domainTailSteps);
        end
        function theUnknownLowSpeedBlendIsCoveredWithoutASideslipCone(testCase)
            cfg = collisionAvoidanceControllerConfig();
            window = researchConservativePredictionWindow(cfg, 0, 4, [.85; .85], 4);
            testCase.verifyGreaterThanOrEqual(window.entrySpeedUpperMetersPerSecond, ...
                hypot(cfg.model.kinematicSpeedRange(2), cfg.model.lateralVelocityMaximum));
        end
        function aNonpassiveLoadCannotUseThisEnvelope(testCase)
            cfg = collisionAvoidanceControllerConfig();
            cfg.roadLoad.rollingCoefficient = -.01;
            testCase.verifyError(@()researchConservativePredictionWindow(cfg, 8, 4, [.85; .85], 4), ...
                'collisionAvoidanceResearch:windowContract');
        end
    end
end
