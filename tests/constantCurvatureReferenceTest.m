classdef constantCurvatureReferenceTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"config")));
        end
    end
    methods (Test)
        function footprintTangentContainsEveryAllowedOrientation(testCase)
            for normalAngle = linspace(-pi,pi,17)
                normal = [cos(normalAngle);sin(normalAngle)];
                for anchor = [-0.4,-0.15,0,0.3,0.4]
                    [offset,slope] = targetPrediction.rectangleSupportMajorant(normal,0,2.5,1,anchor,0.4);
                    yaw = linspace(-0.4,0.4,401);
                    exact = 2.5*abs(cos(normalAngle-yaw))+abs(sin(normalAngle-yaw));
                    testCase.verifyLessThanOrEqual(max(exact-max(offset+slope*yaw,[],1)),1e-12);
                    anchorSupport = 2.5*abs(cos(normalAngle-anchor))+abs(sin(normalAngle-anchor));
                    testCase.verifyEqual(max(offset+slope*anchor),anchorSupport,AbsTol=1e-12);
                end
            end
        end
        function aLateralStripCrossingTheCircleCenterIsRejected(testCase)
            curve = localCurve();
            testCase.verifyError(@() laneGeometry.referenceFrame(curve,30,3,401), ...
                "collisionAvoidanceController:invalidReferenceCurve");
        end
        function arcCoordinatesRoundTripAcrossSampleVertices(testCase)
            curve = localCurve();s = [1,20.03,20.11,55];d = [0,2,-4,1];
            [position,~] = laneGeometry.referencePose(s,d,curve);
            projection = laneGeometry.projectReferenceCurve(position,curve);
            testCase.verifyEqual(projection.station,s,AbsTol=1e-11);
            testCase.verifyEqual(projection.lateralPosition,d,AbsTol=1e-11);
        end
        function frameBoundContainsBothSidesOfTheLateralStrip(testCase)
            curve = localCurve();frame = laneGeometry.referenceFrame(curve,30,3,12);
            [s,d] = meshgrid(linspace(27,33,101),[-12,12]);
            [position,~] = laneGeometry.referencePose(s(:).',d(:).',curve);
            affine = frame.origin+frame.tangent*s(:).'+frame.lateral*d(:).';
            testCase.verifyLessThanOrEqual(max(abs(position-affine),[],2),frame.positionErrorBound+1e-12);
        end
        function uncertainArcPoseHasAValidContinuousChart(testCase)
            curve = localCurve();[position,heading] = laneGeometry.referencePose(30,2,curve);
            state = [position;heading;10;0;0.025];
            [radius,valid] = laneGeometry.referenceUncertainty(state,[0.04;0.04;0.01;0.1;0.1;0.001],curve);
            testCase.verifyTrue(valid);
            testCase.verifyLessThan(radius(1),0.06);
            testCase.verifyGreaterThan(radius(3),0.01);
        end
        function circleCenterCannotDefineAnUncertainChart(testCase)
            curve = localCurve();center = curve.origin+[-sin(curve.heading);cos(curve.heading)]/curve.curvature;
            [~,valid] = laneGeometry.referenceUncertainty([center;0;10;0;0],0.04*ones(6,1),curve);
            testCase.verifyFalse(valid);
        end
        function curvedCruiseIncludesConsistentHeadingAndYawRate(testCase)
            cfg = collisionAvoidanceControllerConfig();cfg.referenceSpeed = 10;
            [state,input] = ltvBicycleModel.cruiseEquilibrium(1/400,cfg);
            [a,b,c] = ltvBicycleModel.continuousMatrices(1/400,10,cfg,[],0, ...
                struct("state",state,"input",input));
            derivative = a*state+b*input+c;
            testCase.verifyEqual(derivative(2:6),zeros(5,1),AbsTol=1e-10);
            testCase.verifyEqual(state(6),hypot(state(4),state(5))/400,AbsTol=1e-12);
            testCase.verifyGreaterThan(input(2),0);
        end
    end
end

function curve = localCurve()
    curve = laneGeometry.validateReferenceCurve(struct("origin",[-20;3], ...
        "heading",-0.05,"curvature",1/400,"length",100));
end
