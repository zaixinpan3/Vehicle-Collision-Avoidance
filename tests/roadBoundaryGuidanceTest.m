classdef roadBoundaryGuidanceTest < matlab.unittest.TestCase
    properties (TestParameter)
        egoSpeed={8,15};
    end
    methods (TestClassSetup)
        function preparePaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            for directory=["controller","config","scripts"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,directory)));
            end
        end
    end
    methods (Test)
        function aScanExposesOnlyVisibleChainsAndMarksRangeCuts(testCase)
            scan=perceiveRoadBoundaries(localRoad(),[0;0],20,3);
            testCase.verifyFalse(isfield(scan,'drivablePolygons'));
            testCase.verifyEmpty(scan.lateralClearance);
            testCase.verifyEqual(scan.boundaryObservation.time,3);
            chains=scan.boundaryObservation.chains;
            testCase.assertNumElements(chains,2);
            testCase.verifyTrue(all([chains.extendStart] & [chains.extendEnd]));
            points=vertcat(chains.points);
            testCase.verifyLessThanOrEqual(max(vecnorm(points,2,2)),20+1e-10);
        end
        function unknownRoadShapeCannotInfluenceTheCurrentField(testCase)
            straight=localRoad();junction=localRoad(70);
            a=perceiveRoadBoundaries(straight,[0;0],20);
            b=perceiveRoadBoundaries(junction,[0;0],20);
            testCase.verifyEqual(a.boundaryObservation,b.boundaryObservation,AbsTol=1e-10);
            model=localModel(a);x=[80;7.8;0;8;2;0];
            aa=predictiveSafetyGeometry.wallGuidance(x,[1;20],model);
            model.road=b;bb=predictiveSafetyGeometry.wallGuidance(x,[1;20],model);
            testCase.verifyEqual(aa,bb,AbsTol=1e-12);
        end
        function tangentExtensionHasNoPerceptionCircleCap(testCase)
            model=localModel(perceiveRoadBoundaries(localRoad(),[0;0],20));
            x=[80;0;0;8;0;0];
            [a,active]=predictiveSafetyGeometry.wallGuidance(x,[20;0],model);
            testCase.verifyFalse(active);testCase.verifyEqual(a,[20;0],AbsTol=0);
            x(2)=7.8;x(5)=2;
            [a,active]=predictiveSafetyGeometry.wallGuidance(x,[1;20],model);
            testCase.verifyTrue(active);testCase.verifyLessThan(a(2),0);
            testCase.verifyEqual(a(1),1,AbsTol=1e-12);
        end
        function anObservedIntersectionOpeningIsNotClosedByOffsetWalls(testCase)
            scan=perceiveRoadBoundaries(localRoad(0),[0;0],30);
            model=localModel(scan);x=[0;10;pi/2;4;0;0];
            [a,active]=predictiveSafetyGeometry.wallGuidance(x,[0;1],model);
            testCase.verifyFalse(active);testCase.verifyEqual(a,[0;1],AbsTol=0);
            testCase.verifyTrue(all(arrayfun(@(c)size(c.points,1)>=3,scan.boundaryObservation.chains)));
        end
        function actualJunctionCornersStillRepelOutwardMotion(testCase)
            model=localModel(perceiveRoadBoundaries(localRoad(0),[0;0],30));
            x=[7.5;7.5;pi/4;5;0;0];
            [a,active,info]=predictiveSafetyGeometry.roadBoundaryGuidance(x,[20;20],model);
            testCase.verifyTrue(active);testCase.verifyTrue(info.compatible);
            testCase.verifyLessThan(sum(a),0);
            testCase.verifyLessThanOrEqual(info.maximumDeficit,1e-8);
        end
        function anEmptyObservedScanDoesNotInventLegacyWalls(testCase)
            road=localRoad();road.drivablePolygons={[-100,-100;100,-100;100,100;-100,100]};
            model=localModel(perceiveRoadBoundaries(road,[0;0],20));
            testCase.verifyEmpty(model.road.boundaryObservation.chains);
            [a,active]=predictiveSafetyGeometry.wallGuidance([0;15;0;8;2;0],[0;100],model);
            testCase.verifyFalse(active);testCase.verifyEqual(a,[0;100],AbsTol=0);
        end
        function perceivedRoadsNeverCreateHardControllerRoadRows(testCase)
            scan=perceiveRoadBoundaries(localRoad(0),[0;0],30);
            scan.lateralClearance=[9;9];
            cfg=collisionAvoidanceControllerConfig(struct('controller',struct('enforceRoadBoundaries',true)));
            ego=struct('position',[0;0],'yaw',0,'speed',8,'lateralVelocity',0,'yawRate',0);
            [~,~,road]=readControllerInputs(ego,[],scan,cfg);
            testCase.verifyEmpty(road.lateralClearance);
            testCase.verifyEqual(road.boundaryObservation,scan.boundaryObservation);
        end
        function malformedVisibleChainsAreRejected(testCase)
            scan=perceiveRoadBoundaries(localRoad(),[0;0],20);
            scan.boundaryObservation.chains(1).points=[0,0;0,0];
            cfg=collisionAvoidanceControllerConfig();
            ego=struct('position',[0;0],'yaw',0,'speed',8,'lateralVelocity',0,'yawRate',0);
            testCase.verifyError(@()readControllerInputs(ego,[],scan,cfg), ...
                'collisionAvoidanceController:invalidBoundaryObservation');
        end
        function fullFootprintAuditCatchesAConcaveCornerCut(testCase)
            road=localRoad(0);shape=[2.4;.95;0;0];
            testCase.verifyGreaterThan(scenarioRoadFootprintMargin([0;20;pi/2],shape,road,9),0);
            testCase.verifyLessThan(scenarioRoadFootprintMargin([8.8;8.8;pi/4],shape,road,9),0);
        end
        function aJunctionTargetRemainsInItsCrossingRoad(testCase,egoSpeed)
            [x,q,road,cfg]=collisionThreatScenario("intersectionCrossing",struct('referenceSpeed',egoSpeed));
            future=predictiveSafetyGeometry.predictTarget(q,0:.1:8);
            margins=arrayfun(@(k)scenarioRoadFootprintMargin(future(1:3,k),future(8:11,k),road,9),1:size(future,2));
            testCase.verifyGreaterThan(min(margins),0);
            witness=bypassFeasibilityBaseline(x,q,road,cfg);
            testCase.verifyTrue(witness.qualified,witness.reason);
        end
        function jointControlKeepsTheInitializedJunctionHorizon(testCase,egoSpeed)
            cfg=struct('referenceSpeed',egoSpeed, ...
                'controller',struct('horizonSteps',8+8*(egoSpeed>=15)), ...
                'solver',struct('timeLimitSeconds',5,'maxIterations',200));
            report=runNonlinearPredictiveSafetyValidation(Scenarios="intersectionCrossing",Frames=11, ...
                ControllerConfiguration=cfg,EstimatorConfiguration=estimatorControllerIntegrationConfig());
            result=report.results;
            testCase.verifyEqual(result.failure,"");
            testCase.verifyEqual(result.executedFrames,11);
            testCase.verifyFalse(result.collisionDetected);
            testCase.verifyFalse(result.roadConstraintsEnforced);
            % The first encounter plan chooses a horizon longer than the
            % slack prefix; subsequent frames retain that same node count.
            testCase.assertNotEmpty(result.trace,'The controller must return a first plan before its horizon can be compared.');
            testCase.verifyGreaterThan(result.trace(1).horizonSteps,cfg.controller.horizonSteps);
            testCase.verifyEqual([result.trace.horizonSteps], ...
                repmat(result.trace(1).horizonSteps,1,numel(result.trace)));
        end
        function aBoundaryAdjustedSeedRetainsNonlinearDynamicConsistency(testCase)
            model=localModel(perceiveRoadBoundaries(localRoad(),[0;0],30));
            x=[0;8.9;0;8;0;0];
            [input,next,active]=predictiveSafetyGeometry.boundaryReferenceInput(x,[0;0],[0;0],model);
            testCase.verifyTrue(active);
            testCase.verifyEqual(next,nonlinearBicycleModel.sample(x,input,model.cfg),AbsTol=0);
            testCase.verifyTrue(all(isfinite([input;next])));
        end
        function aFullyVisibleTrafficIslandIsNotExtendedIntoInfiniteWalls(testCase)
            road=localRoad();road.drivablePolygons={[-100,-100;100,-100;100,100;-100,100; ...
                NaN,NaN;-2,-2;-2,2;2,2;2,-2]};
            scan=perceiveRoadBoundaries(road,[0;0],20);
            chains=scan.boundaryObservation.chains;
            testCase.assertNumElements(chains,1);
            testCase.verifyFalse(chains.extendStart || chains.extendEnd);
            testCase.verifyEqual(chains.points(1,:),chains.points(end,:),AbsTol=0);
            model=localModel(scan);x=[4;0;0;0;0;0];
            [a,active]=predictiveSafetyGeometry.wallGuidance(x,[-100;0],model);
            testCase.verifyTrue(active);testCase.verifyGreaterThan(a(1),-100);
            shape=[.5;.5;0;0];
            testCase.verifyLessThan(scenarioRoadFootprintMargin([0;0;0],shape,road,9),0);
            testCase.verifyGreaterThan(scenarioRoadFootprintMargin([4;0;0],shape,road,9),0);
        end
    end
end

function road=localRoad(junction)
    road=struct('centerline',[-1000,0;1000,0],'scenarioHalfWidthMeters',9);
    road.drivablePolygons={[-1000,-9;1000,-9;1000,9;-1000,9]};
    if nargin>0
        road.drivablePolygons{2}=[junction-9,-1000;junction+9,-1000;junction+9,1000;junction-9,1000];
    end
end

function model=localModel(road)
    model=struct('road',road,'cfg',collisionAvoidanceControllerConfig());
end
