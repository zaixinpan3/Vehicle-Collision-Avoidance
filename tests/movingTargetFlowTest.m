classdef movingTargetFlowTest < matlab.unittest.TestCase
    % Moving-boundary identities and actual nonlinear initialization behavior.
    properties (TestParameter)
        angle = {0,.7,1.9};
        speed = {8,15};
        scenario = {"brakingLead","crossing","turningCrossing"};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
        end
    end
    methods (Test)
        function movingEllipseHasNoRelativeNormalPenetration(testCase,angle)
            rotation=[cos(.4),-sin(.4);sin(.4),cos(.4)];j=[0,-1;1,0];
            map=rotation*diag([5,3]);rate=.3*j*map+rotation*diag([.1,-.2]);
            center=[2;4];translation=[3;-2];q=[cos(angle);sin(angle)];
            v=predictiveSafetyGeometry.movingFlowVelocity(center+map*q,[8;1], ...
                center,translation,map,rate,.5);
            relative=map\(v-translation-rate*q);
            testCase.verifyEqual(q.'*relative,0,AbsTol=2e-14);
        end
        function changingWorldVelocityTransportsTheSameRelativeFlow(testCase)
            p=[-9;2];nominal=[8;0];center=[2;0];targetVelocity=[-3;1];boost=[5;-7];
            first=predictiveSafetyGeometry.movingFlowVelocity(p,nominal,center,targetVelocity,5*eye(2),zeros(2),.7);
            second=predictiveSafetyGeometry.movingFlowVelocity(p,nominal+boost,center,targetVelocity+boost,5*eye(2),zeros(2),.7);
            testCase.verifyEqual(second-first,boost,AbsTol=1e-14);
        end
        function rotatingTheWorldRotatesTheGuidance(testCase,angle)
            rotation=[cos(angle),-sin(angle);sin(angle),cos(angle)];
            p=[-9;2];nominal=[8;0];center=[2;0];translation=[-3;1];map=diag([5,3]);rate=[0,-.6;1,0];
            first=predictiveSafetyGeometry.movingFlowVelocity(p,nominal,center,translation,map,rate,.7);
            second=predictiveSafetyGeometry.movingFlowVelocity(rotation*p,rotation*nominal,rotation*center, ...
                rotation*translation,rotation*map,rotation*rate,.7);
            testCase.verifyEqual(second,rotation*first,AbsTol=2e-14);
        end
        function anInteriorSearchPointDoesNotMakeTheGuidanceSingular(testCase)
            velocity=predictiveSafetyGeometry.movingFlowVelocity([2;3],[8;0],[2;3],[1;0],eye(2),zeros(2),.7);
            testCase.verifyTrue(all(isfinite(velocity)));
        end
        function knownMovingThreatProducesAnAdmissibleNonlinearContinuation(testCase,speed,scenario)
            [ego,target,road,cfg]=localFixture(speed,scenario);
            [command,inputs,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            endpoint=[problem.solution.states(:,end);inputs(:,end)];
            membership=terminalContinuation.membership(endpoint,size(inputs,2),problem.model.terminal);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1));
            testCase.verifyEqual(problem.solution.hard,0);
            testCase.verifyEqual(problem.solution.safety,0);
            testCase.verifyLessThanOrEqual(membership,0);
            testCase.verifyGreaterThanOrEqual(problem.solution.terminalSeparationMargin,0);
            testCase.verifyFalse(problem.metadata.drivingModeSwitching);
            testCase.verifyFalse(problem.metadata.roadConstraintsEnforced);
            testCase.verifyNotEmpty(problem.metadata.search.initializationCandidates);
        end
        function finiteActuatorRatesAreRespectedByAnAcceptedSeed(testCase)
            [ego,target,road,cfg]=localFixture(8,"brakingLead");
            cfg.model.frontWheelSteeringRateMaximum=.8;cfg.model.brakingRatioRateMaximum=2;
            [~,inputs,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            differences=abs(diff([ego.heldActuatorInput,inputs],1,2));
            limits=cfg.controller.sampleTime*[.8;2];
            testCase.verifyLessThanOrEqual(max(differences,[],2),limits+1e-12);
            testCase.verifyEqual(problem.solution.hard,0);
            testCase.verifyEqual(problem.solution.safety,0);
        end
    end
end

function [ego,target,road,cfg]=localFixture(speed,name)
    [x,q,road,cfg]=collisionThreatScenario(name,struct('referenceSpeed',speed, ...
        'controller',struct('horizonSteps',speed+mod(speed,2))));
    cfg.solver.timeLimitSeconds=30;
    ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5), ...
        'yawRate',x(6),'stateTime',0,'heldActuatorInput',[0;0]);
    target=struct('targetPositionInertial',q(1:2), ...
        'targetVelocityInertial',q(4)*[cos(q(3)+q(6));sin(q(3)+q(6))], ...
        'targetYawInertial',q(3),'targetSideslip',q(6),'targetScalarAcceleration',q(5), ...
        'targetRearAxleDistance',q(7),'targetLength',2*q(8),'targetWidth',2*q(9));
end
