classdef nodeCertificateTest < matlab.unittest.TestCase
    %nodeCertificateTest Hold-node safety certificate of the accepted plan.
    methods (TestClassSetup)
        function addPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'solver','bicycle')));
        end
    end
    methods (Test)
        function everyNodeOfTheAcceptedPlanIsPhysicallyClear(testCase)
            [ego,target,cfg,road]=localFixture();
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyTrue(problem.metadata.planCertified);
            [distance,nodes]=localNodeDistances(problem);
            testCase.verifyGreaterThanOrEqual(distance,cfg.collision.clearanceMargin-1e-9);
            testCase.verifyEqual(size(nodes,2),problem.prediction.stageCount);
        end

        function nodeStatesFollowTheExactSampledPlant(testCase)
            [ego,target,cfg,road]=localFixture();
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            prediction=problem.prediction;plan=problem.inputPlan;h=problem.model.sampleTime;
            state=problem.model.initialEgoState;
            for stage=1:prediction.stageCount
                flow=expm(h*[prediction.continuousA(:,:,stage),prediction.continuousB(:,:,stage), ...
                    prediction.continuousC(:,stage);zeros(3,9)]);
                state=flow(1:6,:)*[state;plan(:,stage);1];
                cell=prediction.cells(stage);
                node=cell.map*plan(:)+cell.offset;
                testCase.verifyEqual(node,state,AbsTol=1e-9);
                testCase.verifyEqual(cell.time,stage*h,AbsTol=1e-12);
                testCase.verifySize(cell.offset,[6,1]);
            end
        end

        function collisionRowsUseTheTargetSetAtEachNodeTime(testCase)
            [ego,target,cfg,road]=localFixture();
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            geometry=problem.program.geometry;encounter=problem.model.encounters(1);
            for index=1:numel(problem.prediction.cells)
                cell=problem.prediction.cells(index);data=geometry.cellData(index);
                [center,radius]=targetPrediction.finiteFlow(encounter,cell.time);
                testCase.verifyEqual(data.targets(1).center,center,AbsTol=0);
                testCase.verifyEqual(data.targets(1).radius,radius,AbsTol=0);
                testCase.verifyEqual(data.duration,0,AbsTol=0);
                testCase.verifyEqual(data.degree,0,AbsTol=0);
            end
            testCase.verifyEqual(numel(unique(geometry.cellIndex)),numel(problem.prediction.cells));
        end

        function theCarriedNodesShiftByOneHold(testCase)
            [ego,target,cfg,road]=localFixture();
            [~,~,first,stored]=collisionAvoidanceController(ego,target,road,cfg,[]);
            x=stored.predictedState(:,2);
            [ego.position,ego.yaw]=laneGeometry.fromFrenet(x,first.model.lane);
            ego.speed=x(4);ego.lateralVelocity=x(5);ego.yawRate=x(6);
            ego.stateTime=.1;ego.perception.time=.1;ego.heldActuatorInput=stored.appliedInput;
            target.targetPositionInertial=target.targetPositionInertial+.1*target.targetVelocityInertial;
            [~,~,next]=collisionAvoidanceController(ego,target,road,cfg,stored);
            testCase.verifyTrue(next.program.inheritedPredictionFamily);
            testCase.verifyEqual(next.metadata.inheritedFeasibleFamily,next.metadata.shiftedWitnessContained);
            testCase.verifyEqual([next.prediction.cells.stage],1:first.prediction.stageCount-1);
            testCase.verifyEqual([next.prediction.cells.time],.1*(1:first.prediction.stageCount-1),AbsTol=1e-12);
            distance=localNodeDistances(next);
            testCase.verifyGreaterThanOrEqual(distance,cfg.collision.clearanceMargin-1e-9);
        end

        function metadataDeclaresTheNodeSampling(testCase)
            [ego,~,cfg,road]=localFixture();
            [~,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            testCase.verifyFalse(problem.metadata.wholeHoldCertificate);
            testCase.verifyEqual(problem.metadata.certificateSampling,"holdNodes");
        end
    end
end

function [ego,target,cfg,road]=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'controller',struct('sampleTime',.1),'solver',struct('frameDeadlineSeconds',30)));
    ego=struct('position',[0;0],'yaw',0,'speed',8,'stateTime',0, ...
        'perception',struct('time',0,'range',16,'completeWithinRange',true));
    target=struct('trackId',1,'targetPositionInertial',[15;0], ...
        'targetVelocityInertial',[0;0],'targetAccelerationInertial',[0;0], ...
        'targetHeadingInertial',0,'targetYawRate',0, ...
        'predictionMotion',struct('kind',"finite-sensing-motion-v1", ...
        'jerkBound',[0;0],'yawAccelerationBound',0));
    road=[-100,0;2000,0];
end

function [distance,nodes]=localNodeDistances(problem)
% Exact rectangle distance between the ego and the target at every certified node.
    prediction=problem.prediction;plan=problem.inputPlan(:);cfg=problem.model.cfg;
    encounter=problem.model.encounters(1);count=prediction.stageCount;
    nodes=zeros(6,count);distance=zeros(1,count);
    for stage=1:count
        cell=prediction.cells(stage);
        nodes(:,stage)=cell.map*plan+cell.offset;
        [position,heading]=laneGeometry.fromFrenet(nodes(:,stage),problem.model.lane);
        center=targetPrediction.finiteFlow(encounter,cell.time);
        distance(stage)=avoidanceSafetyGeometry.rectangleDistance(position,heading,center(1:2),center(7), ...
            [cfg.vehicle.length/2;cfg.vehicle.width/2;encounter.halfLength;encounter.halfWidth]);
    end
end
