classdef curvedCruiseCertificateTest < matlab.unittest.TestCase
    properties (TestParameter)
        curvature = struct("straight",0,"gentleLeft",1/400,"left",1/100,"right",-1/100);
        bias = struct("none",0,"declared",0.2);
    end
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"config")));
        end
    end
    methods (Test)
        function theTrimBalancesTheNonlinearVehicle(testCase,curvature,bias)
            cfg = collisionAvoidanceControllerConfig();
            [state,input] = ltvBicycleModel.cruiseEquilibrium(curvature,cfg,bias);
            parameters = modifiedFialaTire.parameters(cfg);
            final = ltvBicycleModel.nominalKernel(state,input,0.1,0,curvature,cfg,parameters,bias,false);
            testCase.verifyEqual(final(2:6,end),state(2:6),AbsTol=1e-10);
            testCase.verifyEqual(final(1,end),0.1*hypot(state(4),state(5)),AbsTol=1e-10);
        end

        function theCertificateAndCostUseTheRoadCruisePoint(testCase,curvature)
            [ego,road,cfg,state,input] = localFixture(curvature);
            [command,~,problem] = collisionAvoidanceController(ego,[],road,cfg,[]);
            cert = problem.qp.clf.certificate;
            [a,b] = ltvBicycleModel.continuousMatrices(curvature,cfg.referenceSpeed,cfg,[],0, ...
                struct("state",state,"input",input));
            closedLoop = a(2:6,2:6)-b(2:6,:)*cert.feedbackGain;
            testCase.verifyEqual(cert.operatingCurvature,curvature,AbsTol=1e-14);
            testCase.verifyEqual(cert.operatingState,state,AbsTol=1e-12);
            testCase.verifyEqual(cert.operatingInput,input,AbsTol=1e-12);
            testCase.verifyEqual(problem.qp.clf.referenceStart,state(2:6),AbsTol=1e-12);
            testCase.verifyEqual(closedLoop.'*cert.lyapunovMatrix+cert.lyapunovMatrix*closedLoop, ...
                -cert.decreaseMatrix,AbsTol=1e-10);
            testCase.verifyLessThan(norm(command.actuatorInput-input,inf),1e-3);
            testCase.verifyTrue(problem.metadata.planCertified);
        end

        function oppositeBendsHaveOppositeSteeringCenters(testCase)
            [ego,road,cfg] = localFixture(1/100);
            [~,~,left] = collisionAvoidanceController(ego,[],road,cfg,[]);
            [ego,road] = localFixture(-1/100);
            [~,~,right] = collisionAvoidanceController(ego,[],road,cfg,[]);
            testCase.verifyGreaterThan(left.qp.clf.certificate.operatingInput(1),0);
            testCase.verifyEqual(right.qp.clf.certificate.operatingInput, ...
                [-1;1].*left.qp.clf.certificate.operatingInput,AbsTol=1e-12);
        end

        function anUnattainableTurnDoesNotProduceASubstituteInput(testCase)
            cfg = collisionAvoidanceControllerConfig();
            testCase.verifyError(@() ltvBicycleModel.cruiseEquilibrium(1,cfg), ...
                "collisionAvoidanceController:invalidCruiseOperatingPoint");
        end
    end
end

function [ego,road,cfg,state,input] = localFixture(curvature)
    cfg = collisionAvoidanceControllerConfig(struct("referenceSpeed",10, ...
        "controller",struct("horizonSteps",8,"sampleTime",0.1), ...
        "model",struct("lateralDomainRadius",3)));
    curve = struct("origin",[0;0],"heading",0,"curvature",curvature,"length",300);
    [position,heading] = laneGeometry.referencePose(20,0,curve);
    [state,input] = ltvBicycleModel.cruiseEquilibrium(curvature,cfg);
    ego = struct("position",position,"yaw",heading+state(3),"speed",state(4), ...
        "lateralVelocity",state(5),"yawRate",state(6),"heldActuatorInput",input,"stateTime",0);
            ego.stateTime = 0;
            ego.perception = struct("time",0,"range",30,"completeWithinRange",true);
    centerline = laneGeometry.referencePose(0:2:300,0,curve).';
    road = struct("centerline",centerline,"referenceCurve",curve);
end
