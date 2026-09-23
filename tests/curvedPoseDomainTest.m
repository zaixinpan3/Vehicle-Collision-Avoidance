classdef curvedPoseDomainTest < matlab.unittest.TestCase
    properties (TestParameter)
        curvature = struct('left',.02,'right',-.02);
    end
    methods (TestClassSetup)
        function addPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'solver','bicycle')));
        end
    end
    methods (Test)
        function aLocalMapEnclosesCircularPositionsAndPreservesYaw(testCase,curvature)
            curve=struct('origin',[2;-1],'heading',.3,'curvature',curvature,'length',100);
            frame=laneGeometry.localPoseFrame(curve,[30;2;.1],[2;.5;.2]);
            [s,d,e]=ndgrid(linspace(28,32,9),linspace(1.5,2.5,9),linspace(-.1,.3,7));
            states=[s(:).';d(:).';e(:).';zeros(3,numel(s))];
            [position,heading]=laneGeometry.referencePose(states(1,:),states(2,:),curve);
            affine=frame.positionOffset+frame.positionMap*states;
            testCase.verifyLessThanOrEqual(max(vecnorm(position-affine)),frame.positionRemainder);
            testCase.verifyEqual(frame.yawOffset+frame.yawRow*states,heading+states(3,:),AbsTol=2e-14);
            testCase.verifyEqual(norm(frame.tangent),1,AbsTol=1e-14);
        end

        function nativeCircularRowsMatchTheInterpretedCertificate(testCase)
            [~,~,problem]=localAdmission(.02);
            data=problem.program.geometry.cellData;
            testCase.verifyEqual(avoidanceCellRowsKernelMex(data),avoidanceSafetyGeometry.cellRows(data),AbsTol=1e-10);
        end

        function jointSupportUpperBoundsThePhysicalRectangleResidual(testCase,curvature)
            [~,~,problem]=localAdmission(curvature);
            discrepancy=localPhysicalResidualGap(problem);
            testCase.verifyLessThanOrEqual(discrepancy,1e-10);
        end

        function sparseRealizationPreservesCircularDomainRows(testCase)
            [~,~,problem]=localAdmission(.02);
            [rowError,costError]=localRealizationErrors(problem.program);
            testCase.verifyLessThan(rowError,problem.model.cfg.solver.constraintTolerance);
            testCase.verifyLessThan(costError,1e-7);
        end

        function anOverlappingCircularSeedFindsAHardScalarCertificate(testCase,curvature)
            [~,~,problem]=localAdmission(curvature,0);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyEqual(problem.metadata.restorationSolverCallCount,0);
            testCase.verifyLessThanOrEqual(max(problem.program.physicalMatrix*problem.decision ...
                -problem.program.physicalBound),0);
        end

        function shiftingRetainsACompleteDomainAndExitWitness(testCase,curvature)
            [ego,target,first,stored,road,cfg]=localAdmission(curvature);
            [next,witness]=localSuccessor(ego,target,first,stored,road,cfg);
            testCase.verifyTrue(next.metadata.planCertified);
            testCase.verifyTrue(next.metadata.inheritedFeasibleFamily);
            testCase.verifyEqual(next.program.completion.deadline,first.program.completion.deadline,AbsTol=0);
            testCase.verifyLessThanOrEqual(max(next.program.physicalMatrix*witness-next.program.physicalBound),0);
            testCase.verifyEqual(next.program.geometry.frames(1).domainCenter, ...
                first.program.geometry.frames(2).domainCenter,AbsTol=0);
        end
    end
end

function [rowError,costError]=localRealizationErrors(program)
    lifted=avoidanceStageQp.build(program);n=program.prediction.planCount;
    cost=zeros(2,2);rowError=0;
    for k=1:2
        decision=zeros(numel(program.q),1);
        decision(1:numel(program.anchorPlan))=program.anchorPlan;
        decision=decision+.01*k*sin((1:numel(program.q)).');
        prediction=program.prediction;
        states=prediction.egoStateOffset+reshape(pagemtimes(prediction.egoStateMatrix,decision(1:n)),6,[]);
        delta=states(:,2:end)-lifted.stateCenter(:,2:end);augmented=[decision;delta(:)];
        residual=lifted.b-lifted.A*augmented;
        expected=program.b(lifted.retainedRows)-program.A(lifted.retainedRows,:)*decision;
        difference=[residual(1:lifted.cones(1));residual(lifted.cones(1)+1:end)-expected];
        scale=1+abs(lifted.b)+abs(lifted.A)*abs(augmented);
        scale(lifted.cones(1)+1:end)=scale(lifted.cones(1)+1:end) ...
            +abs(program.b(lifted.retainedRows))+abs(program.A(lifted.retainedRows,:))*abs(decision);
        rowError=max(rowError,max(abs(difference)./scale));
        cost(k,:)=[.5*decision.'*program.P*decision+program.q.'*decision, ...
            .5*augmented.'*lifted.P*augmented+lifted.q.'*augmented];
    end
    costError=abs(diff(cost(:,1))-diff(cost(:,2)));
end

function [ego,target,problem,stored,road,cfg]=localAdmission(curvature,lateral)
    if nargin<2,lateral=-5*sign(curvature);end
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8,'controller',struct('sampleTime',.1,'minimumHorizonSteps',1), ...
        'solver',struct('frameDeadlineSeconds',30,'certificateSearchTimeLimit',30)));
    curve=struct('origin',[0;0],'heading',0,'curvature',curvature,'length',150);
    road=struct('referenceCurve',curve,'centerline',laneGeometry.referencePose(0:2:150,0,curve).');
    z=ltvBicycleModel.cruiseEquilibrium(curvature,cfg);
    ego=struct('position',[0;0],'yaw',z(3),'speed',z(4),'lateralVelocity',z(5),'yawRate',z(6),'stateTime',0, ...
        'perception',struct('time',0,'range',16,'completeWithinRange',true));
    [position,heading]=laneGeometry.referencePose(15,lateral,curve);
    target=struct('trackId',1,'targetPositionInertial',position,'targetVelocityInertial',[0;0], ...
        'targetAccelerationInertial',[0;0],'targetHeadingInertial',heading,'targetYawRate',0, ...
        'predictionMotion',struct('kind',"finite-sensing-motion-v1",'jerkBound',[0;0],'yawAccelerationBound',0));
    [~,~,problem,stored]=collisionAvoidanceController(ego,target,road,cfg,[]);
end

function gap=localPhysicalResidualGap(problem)
    index=20;data=problem.program.geometry.cellData(index);frame=problem.program.geometry.frames(index);
    cfg=problem.model.cfg;normal=problem.program.geometry.normals{index}(:,1);
    [s,d,e]=ndgrid(linspace(-1,1,9),linspace(-1,1,9),linspace(-1,1,9));
    points=frame.domainCenter+frame.domainRadius.*[s(:).';d(:).';e(:).'];
    states=[points;zeros(3,size(points,2))];
    [position,heading]=laneGeometry.fromFrenet(states,problem.model.lane);
    target=data.target;certificate=problem.program.jointCertificate;
    selected=find([certificate.records.stage]==index & ~[certificate.records.isExit],1);
    record=certificate.records(selected);angle=certificate.angles(selected);
    conservative=zeros(1,size(points,2));
    targetSupport=targetPrediction.rectangleSupport(target.halfLength,target.halfWidth,normal,target.center(7),0);
    physical=zeros(1,size(points,2));
    for k=1:numel(physical)
        conservative(k)=avoidanceSafetyGeometry.jointValue(record,states(:,k),angle);
        egoSupport=targetPrediction.rectangleSupport(cfg.vehicle.length/2,cfg.vehicle.width/2,normal,heading(k),0);
        physical(k)=egoSupport+targetSupport-normal.'*(position(:,k)-target.center(1:2));
    end
    gap=max(physical-conservative);
end

function [next,witness]=localSuccessor(ego,target,first,stored,road,cfg)
    x=stored.predictedState(:,2);[ego.position,ego.yaw]=laneGeometry.fromFrenet(x,first.model.lane);
    ego.speed=x(4);ego.lateralVelocity=x(5);ego.yawRate=x(6);ego.stateTime=.1;
    ego.perception.time=.1;ego.heldActuatorInput=stored.appliedInput;
    [~,~,next]=collisionAvoidanceController(ego,target,road,cfg,stored);
    witness=stored.plan(:,2:end);witness=[witness(:);0];
end
