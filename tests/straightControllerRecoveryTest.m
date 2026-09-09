classdef straightControllerRecoveryTest < matlab.unittest.TestCase
    % Full-horizon rejection of the old empirical short-prefix profile.
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
        function theOldEmpiricalProfileCannotClaimFullHorizonAdmission(testCase,targetSpeedPrior)
            cfg = localConfiguration();
            estimator = estimatorControllerIntegrationConfig();
            estimator.randomSeed = 20260907;
            estimator.initialization.targetSpeedPrior = targetSpeedPrior;
            estimator.vehicle.targetSpeed = targetSpeedPrior;
            result = runFiniteBicycleDiagnostic(cfg,0.1,EstimatorConfiguration=estimator,PrepareController=false);
            testCase.verifyTrue(result.failure.occurred);
            testCase.verifyEqual(result.failure.identifier,"collisionAvoidanceController:noCertifiedContinuation");
            testCase.verifyEqual(result.time,0);
            testCase.verifyEmpty(result.input);
        end
        function exactObservationsCannotHideAnUncertifiedResidualHorizon(testCase)
            result = runFiniteBicycleDiagnostic(localConfiguration(),0.1,PrepareController=false);
            testCase.verifyTrue(result.failure.occurred);
            testCase.verifyEqual(result.failure.identifier,"collisionAvoidanceController:noCertifiedContinuation");
            testCase.verifyEqual(result.time,0);
            testCase.verifyEmpty(result.input);
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
