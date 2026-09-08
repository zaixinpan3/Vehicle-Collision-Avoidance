classdef straightControllerRecoveryTest < matlab.unittest.TestCase
    % Declared bicycle regression for the September 7 heading/rate failure.
    % This test complements, and does not replace, PassVeh14DOF validation.
    properties (TestParameter)
        targetSpeedPrior = struct('matched',10,'overestimated',15);
    end
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            for folder = ["controller","config","scripts"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,folder)));
            end
        end
    end
    methods (Test)
        function boundedNoisyEstimatesSupportTheCompleteEncounter(testCase, targetSpeedPrior)
            cfg = localConfiguration();
            estimator = estimatorControllerIntegrationConfig();
            estimator.randomSeed = 20260907;
            estimator.initialization.targetSpeedPrior = targetSpeedPrior;
            estimator.vehicle.targetSpeed = targetSpeedPrior;
            result = runFiniteBicycleDiagnostic(cfg,12,EstimatorConfiguration=estimator);
            testCase.assertFalse(result.failure.occurred,result.failure.message);
            audits = cellfun(@(value)value.truthEnclosure,result.measurementAudit);
            testCase.verifyTrue(all([audits.egoPremisesSatisfied]));
            testCase.verifyTrue(all([audits.egoContained]));
            testCase.verifyTrue(all([audits.checkedTargetComponentsContained]));
            testCase.verifyGreaterThan(result.minimumSampledSeparationMargin,0.25);
            testCase.verifyTrue(all(cellfun(@(m)m.planCertified,result.metadata)));
            testCase.verifyFalse(any(cellfun(@(m)m.fallbackUsed,result.metadata)));
            tail = result.time>=11;
            testCase.verifyLessThan(max(abs(result.state(tail,4)-10)),0.02);
            testCase.verifyLessThan(max(abs(result.state(tail,2))),0.02);
            testCase.verifyLessThan(max(abs(result.state(tail,3))),0.002);
        end
        function rateLimitedAvoidanceCompletesAndReturnsToCruise(testCase)
            cfg = localConfiguration();
            result = runFiniteBicycleDiagnostic(cfg,12);
            testCase.assertFalse(result.failure.occurred,result.failure.message);
            testCase.verifyEqual(result.time(end),12,AbsTol=1e-12);
            testCase.verifyGreaterThan(result.minimumSampledSeparationMargin,0);
            trace = result.plantTrace;
            near = find(abs(trace.state(:,1)-(100-10*trace.time))<8);
            distance = zeros(numel(near),1);
            for index = 1:numel(near)
                row = near(index);
                distance(index) = rectangleConfigurationDistance(trace.state(row,1:2).', ...
                    trace.state(row,3),[100-10*trace.time(row);0.8],pi,[2.5;1;2.5;1]);
            end
            testCase.verifyGreaterThan(min(distance),0.25);
            testCase.verifyLessThan(max(abs(result.state(:,3))),0.4);
            testCase.verifyLessThanOrEqual(max(abs(diff(result.input(:,1))))/0.05,0.5);
            testCase.verifyLessThanOrEqual(max(abs(diff(result.input(:,2))))/0.05,2);
            tail = result.time>=11;
            testCase.verifyLessThan(max(abs(result.state(tail,4)-10)),0.01);
            testCase.verifyLessThan(max(abs(result.state(tail,2))),0.02);
            testCase.verifyLessThan(max(abs(result.state(tail,3))),0.002);
            testCase.verifyTrue(all(cellfun(@(m)m.planCertified,result.metadata)));
            testCase.verifyFalse(any(cellfun(@(m)m.fallbackUsed,result.metadata)));
        end
    end
end

function cfg = localConfiguration()
    % Freeze the independently loaded physical parameters behind the failed
    % straight trial. No MathWorks model or recorded trajectory is needed.
    cfg = finiteSensingValidationConfig();
    cfg.referenceSpeed = 10;
    cfg.model.speedMaximum = 18;
    cfg.vehicle = struct("m",1181,"Iz",2066,"lf",1.515,"lr",1.504, ...
        "wheelbase",3.075,"length",5,"width",2,"gravity",9.81, ...
        "centerOfGravityHeight",0.134);
    cfg.tire = struct("corneringStiffness",[134958.69931334612;158363.4557764245], ...
        "frictionCoefficient",[1.1270986189302326;1.1102947429302326]);
    cfg.roadLoad = struct("airDensity",1.2929576815620751,"dragCoefficient",0.3, ...
        "frontalArea",2.11,"rollingCoefficient",0.011617846554233433, ...
        "rollingSpeedCoefficient",2.7827177375409424e-5, ...
        "rollingQuarticCoefficient",5.974750067882327e-10,"rollingTransitionSpeed",0.5);
end
