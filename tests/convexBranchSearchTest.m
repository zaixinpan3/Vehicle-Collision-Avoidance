classdef convexBranchSearchTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function incompatibleFirstChoicesDoNotExcludeAnotherFeasibleBranch(testCase)
            [program,cfg]=localProgram();
            positive=localOption(program,-1,-.03);
            negative=localOption(program,1,-.03);
            compatible=localOption(program,1,-.04);
            impossible=localOption(program,-1,-2);
            program.branchFamily=localFamily({[positive,negative],[compatible,impossible]});
            [accepted,result,search]=solveHardCbfClf.branches(program,cfg);
            testCase.assertTrue(result.feasible);
            certified=solveHardCbfClf.certify(accepted,result.decision);
            testCase.verifyLessThanOrEqual(result.decision(1),-.04+1e-7);
            testCase.verifyGreaterThanOrEqual(min(certified.physicalBound ...
                -certified.physicalMatrix*result.decision),0);
            testCase.verifyGreaterThan(search.integerCalls,0);
            testCase.verifyEqual(search.status,"feasible");
        end
        function exhaustedFiniteFamilyIsDistinguishedFromSearchTimeout(testCase)
            [program,cfg]=localProgram();
            positive=localOption(program,-1,-.03);negative=localOption(program,1,-.03);
            middle=localOption(program,1,.01);
            middle.matrix=[middle.matrix;-middle.matrix];
            middle.bound=[.01;.01];middle.physicalBound=middle.bound+1e-6;
            program.branchFamily=localFamily({[positive,negative],middle});
            [~,result,search]=solveHardCbfClf.branches(program,cfg);
            testCase.verifyFalse(result.feasible);
            testCase.verifyTrue(search.exhausted);
            testCase.verifyEqual(search.status,"finiteFamilyInfeasible");
            cfg.solver.workTimer=tic;cfg.solver.workTimeLimit=realmin;
            [~,timed,limited]=solveHardCbfClf.branches(program,cfg);
            testCase.verifyFalse(timed.feasible);
            testCase.verifyFalse(limited.exhausted);
            testCase.verifyEqual(limited.status,"searchIncomplete");
        end
        function aDirectionGridIsConservativeAtRoundedRectangleCorners(testCase)
            clearance=.25;halfSize=[4.8;1.9];
            position=halfSize+clearance*[cos(pi/8);sin(pi/8)];
            exact=norm(max(abs(position)-halfSize,0));
            angles=(0:7)*pi/4;normals=[cos(angles);sin(angles)];
            residual=normals.'*position-abs(normals).'*halfSize-clearance;
            testCase.verifyEqual(exact,clearance,AbsTol=1e-14);
            testCase.verifyLessThan(max(residual),0);
            % An accepted finite branch still implies Euclidean clearance.
            accepted=halfSize+[.3;0];
            testCase.verifyGreaterThanOrEqual(norm(max(abs(accepted)-halfSize,0)),clearance);
            testCase.verifyGreaterThan(max(normals.'*accepted-abs(normals).'*halfSize-clearance),0);
        end
        function oncomingAdmissionProducesACarriedConvexWitness(testCase)
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
                'controller',struct('sampleTime',.1),'model',struct('lateralDomainRadius',4), ...
                'solver',struct('frameDeadlineSeconds',60)));
            ego=struct('position',[0;0],'yaw',0,'speed',8,'stateTime',0, ...
                'perception',struct('time',0,'range',16,'completeWithinRange',true));
            target=struct('trackId',1,'targetPositionInertial',[18.4;0], ...
                'targetVelocityInertial',[-8;0],'targetAccelerationInertial',[0;0], ...
                'targetHeadingInertial',pi,'targetYawRate',0, ...
                'predictionMotion',struct('kind',"finite-sensing-motion-v1", ...
                'jerkBound',[0;0],'yawAccelerationBound',0));
            road=[-100,0;2000,0];
            [~,~,first,state]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyGreaterThan(first.metadata.integerSolverCallCount,0);
            testCase.verifyTrue(first.metadata.postSolveCertificationPerformed);
            x=state.predictedState(:,2);
            [ego.position,ego.yaw]=laneGeometry.fromFrenet(x,first.model.lane);
            ego.speed=x(4);ego.lateralVelocity=x(5);ego.yawRate=x(6);
            ego.stateTime=.1;ego.perception.time=.1;ego.heldActuatorInput=state.appliedInput;
            target.targetPositionInertial=target.targetPositionInertial+[-.8;0];
            [~,~,next]=collisionAvoidanceController(ego,target,road,cfg,state);
            witness=state.plan(:,2:end);witness=witness(:);
            testCase.verifyTrue(next.metadata.inheritedFeasibleFamily);
            testCase.verifyEqual(next.metadata.integerSolverCallCount,0);
            testCase.verifyLessThanOrEqual(max(next.program.physicalMatrix*[witness;0] ...
                -next.program.physicalBound),0);
            testCase.verifyEqual(next.program.completion.deadline,first.program.completion.deadline);
        end
    end
end

function [program,cfg]=localProgram()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'controller',struct('sampleTime',.1),'model',struct('lateralDomainRadius',4)));
    ego=struct('position',[0;0],'yaw',0,'speed',8,'stateTime',0);
    [~,~,problem]=collisionAvoidanceController(ego,[],[-100,0;2000,0],cfg,[]);
    program=problem.program;
end

function option=localOption(program,coefficient,limit)
    row=zeros(1,program.layout.planCount);row(1)=coefficient;
    option=struct('matrix',row,'physicalBound',limit+1e-6,'bound',limit, ...
        'label',"collision:synthetic",'stage',1,'cellIndex',1,'targetIndex',1, ...
        'direction',[1;0],'stateRow',[],'stateBound',[]);
end

function family=localFamily(groups)
    family=struct('groups',{groups},'directions',[],'log10AssignmentCount', ...
        sum(log10(cellfun(@numel,groups))));
end
