classdef smoothReferenceGeometryTest < matlab.unittest.TestCase
    properties (TestParameter)
        anchor = struct('insidePiece',30,'atCurvatureKnot',50,'atContinuation',100);
    end
    methods (TestClassSetup)
        function addPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function variableCurvatureRequiresAnExplicitContinuation(testCase)
            curve=rmfield(localCurve(),'continuation');
            testCase.verifyError(@() laneGeometry.validateReferenceCurve(curve), ...
                'collisionAvoidanceController:invalidReferenceCurve');
        end

        function aConstantProfileAgreesWithTheAnalyticCircle(testCase)
            curve=localCurve();curve.curvatureProfile(:,2)=.01;curve.curvature=.01;
            constant=rmfield(curve,{'curvatureProfile','continuation'});
            station=linspace(-10,120,201);
            [actual,heading,error]=laneGeometry.referencePose(station,0,curve);
            [expected,expectedHeading]=laneGeometry.referencePose(station,0,constant);
            testCase.verifyLessThanOrEqual(max(vecnorm(actual-expected)-error),0);
            testCase.verifyEqual(heading,expectedHeading,AbsTol=1e-12);
        end

        function aLinearCurvatureProfileMatchesAnIndependentIntegral(testCase)
            curve=localCurve();curve.curvatureProfile=[0,0;100,.01];
            [actual,~,error]=laneGeometry.referencePose(100,0,curve);
            expected=[integral(@(s) cos(.00005*s.^2),0,100,AbsTol=1e-12,RelTol=1e-13); ...
                integral(@(s) sin(.00005*s.^2),0,100,AbsTol=1e-12,RelTol=1e-13)];
            testCase.verifyLessThanOrEqual(norm(actual-expected),error);
        end

        function integratedHeadingHasTheDeclaredCurvature(testCase)
            curve=localCurve();station=[12,36,64,88];step=1e-4;
            [~,left]=laneGeometry.referencePose(station-step,0,curve);
            [~,right]=laneGeometry.referencePose(station+step,0,curve);
            [curvature,derivative]=laneGeometry.referenceCurvature(station,curve);
            testCase.verifyEqual((right-left)/(2*step),curvature,AbsTol=1e-11);
            testCase.verifyGreaterThan(max(abs(derivative)),0);
            testCase.verifyEqual(laneGeometry.referenceCurvature([-10,110],curve),[0,0],AbsTol=0);
        end

        function localMapsEncloseVariableCurvatureAndYaw(testCase,anchor)
            curve=localCurve();frame=laneGeometry.localPoseFrame(curve,[anchor;1;.1],[3;.7;.2]);
            [s,d,e]=ndgrid(linspace(anchor-3,anchor+3,21),linspace(.3,1.7,9),linspace(-.1,.3,7));
            state=[s(:).';d(:).';e(:).';zeros(3,numel(s))];
            [position,heading]=laneGeometry.referencePose(state(1,:),state(2,:),curve);
            affine=frame.positionOffset+frame.positionMap*state;
            testCase.verifyLessThanOrEqual(max(vecnorm(position-affine)),frame.positionRemainder);
            yawError=abs(heading+state(3,:)-frame.yawOffset-frame.yawRow*state);
            testCase.verifyLessThanOrEqual(max(yawError),frame.headingErrorBound);
        end

        function stationaryProjectionPreservesTheSelectedSmoothBranch(testCase)
            curve=localCurve();station=[-3,5,25,50,75,99,110];
            position=laneGeometry.referencePose(station,1.3,curve);
            projection=laneGeometry.projectReferenceCurve(position,curve,station+.1);
            testCase.verifyEqual(projection.station,station,AbsTol=1e-8);
            testCase.verifyEqual(projection.lateralPosition,1.3*ones(size(station)),AbsTol=1e-8);
        end

        function referenceOnlyGeometryIsIndependentOfTheEgoPose(testCase)
            curve=localCurve();road=struct('referenceCurve',curve);cfg=collisionAvoidanceControllerConfig();
            ego=struct('position',[0;0],'yaw',0,'speed',8);
            [~,first]=readPlanningInputs(ego,[],road,cfg);
            ego.position=[20;2];ego.yaw=.2;
            [~,second]=readPlanningInputs(ego,[],road,cfg);
            testCase.verifyEqual(first,second);
            testCase.verifyTrue(laneGeometry.isVaryingReference(first));
        end

        function uncertainInverseProjectionIsNotSilentlyUnderbounded(testCase)
            curve=localCurve();position=laneGeometry.referencePose(30,0,curve);
            state=[position;0;8;0;0];
            [radius,valid]=laneGeometry.referenceUncertainty(state,[.01;.01;.01;0;0;0],curve);
            testCase.verifyFalse(valid);
            testCase.verifyTrue(all(isinf(radius(1:3))));
            [exactRadius,exactValid]=laneGeometry.referenceUncertainty(state,zeros(6,1),curve);
            testCase.verifyTrue(exactValid);
            testCase.verifyEqual(exactRadius,zeros(6,1));
        end

        function sweptRowsChargeTheNonaffineYawRemainder(testCase)
            [maximumResidualGap,yawRemainder]=localPhysicalResidualGap();
            testCase.verifyGreaterThan(yawRemainder,0);
            testCase.verifyLessThanOrEqual(maximumResidualGap,1e-10);
        end
    end
end

function curve=localCurve()
    curve=struct('origin',[0;0],'heading',0,'curvature',0,'length',100, ...
        'curvatureProfile',[0,0;25,.01;50,0;75,-.01;100,0], ...
        'continuation','constantCurvature');
end

function [gap,yawRemainder]=localPhysicalResidualGap()
    curve=localCurve();frame=laneGeometry.localPoseFrame(curve,[36;1;.1],[3;.7;.2]);
    [pose,domain]=laneGeometry.poseData(frame);normal=[.6;.8];
    target=struct('center',[60;6;0;0;0;0;.2;0],'radius',zeros(8,1), ...
        'contract',struct('jerkBound',[0;0],'yawAccelerationBound',0),'halfLength',2.5,'halfWidth',1);
    boundary=struct('coefficients',[0,0,0],'safeSideSign',1,'longitudinalDirection',[1;0], ...
        'lateralDirection',[0;1],'origin',[0;0],'parameterRange',[-100;100],'normalDistanceErrorBound',0);
    data=struct('frame',[frame.origin;frame.tangent;frame.lateral;frame.heading;frame.positionErrorBound; ...
        frame.headingErrorBound;frame.stationLower;frame.stationUpper],'pose',pose,'domain',domain, ...
        'nominal',repmat([36;1;.1;8;0;0],1,8),'targets',target,'boundaries',repmat(boundary,0,1), ...
        'settings',[2.5;1;.8;4;.25],'duration',.1,'degree',7,'normals',normal);
    rows=avoidanceSafetyGeometry.cellRows(data);selected=rows.source==1;
    [s,d,e]=ndgrid(linspace(33,39,17),linspace(.3,1.7,7),linspace(-.1,.3,9));
    state=[s(:).';d(:).';e(:).';zeros(3,numel(s))];
    [position,heading]=laneGeometry.referencePose(state(1,:),state(2,:),curve);
    heading=heading+state(3,:);
    targetSupport=targetPrediction.rectangleSupport(2.5,1,normal,.2,0);
    egoSupport=2.5*abs(normal.'*[cos(heading);sin(heading)]) ...
        +abs(normal.'*[-sin(heading);cos(heading)]);
    physical=egoSupport+targetSupport+.25-normal.'*(position-target.center(1:2));
    conservative=max(rows.state(selected,:)*state-rows.bound(selected,1),[],1);
    gap=max(physical-conservative);yawRemainder=frame.headingErrorBound;
end
