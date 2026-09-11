classdef sparseControllerIntegrationTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            for name=["controller","config","tests"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,name)));
            end
        end
    end
    methods (Test)
        function largeResidualClfProblemSolvesBothNativeStages(testCase)
            [ego,~,route,cfg]=encounterTestFixture.crossing();
            % Use the diagnosed straight vehicle rather than attaching its
            % residual allowance to a different, infeasible crossing model.
            ego.speed=10;cfg.referenceSpeed=10;
            cfg.model.lateralDomainRadius=12;cfg.model.linearizationPolicy="trajectory";
            cfg.vehicle.m=1181;cfg.vehicle.Iz=2066;cfg.vehicle.lf=1.515;cfg.vehicle.lr=1.504;
            cfg.tire.corneringStiffness=[134958.69931334612;158363.4557764245];
            cfg.tire.frictionCoefficient=[1.1270986189302326;1.1102947429302326];
            cfg.model.plantModelResidualRateBound=[.2;.06;.02;2.5;5;4];
            cfg.model.frontWheelSteeringRateMaximum=.5;
            cfg.model.brakingRatioRateMaximum=2;
            cfg.solver.optimalityTolerance=1e-4;
            statuses=[];cfg.solver.jointFunction=@capture;
            [~,~,problem]=collisionAvoidanceController(ego,[],route,cfg,[]);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyEqual(statuses,[1,1]);
            function result=capture(~,program)
                result=program.defaultSolver();statuses(end+1)=result.output.status;
            end
        end
        function bothLexicographicStagesRetainDynamicsEqualities(testCase)
            [ego,target,route,cfg]=encounterTestFixture.crossing();
            equalityCounts=[];cfg.solver.jointFunction=@capture;
            [~,~,problem]=collisionAvoidanceController(ego,target,route,cfg,[]);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyNumElements(equalityCounts,2);
            testCase.verifyGreaterThan(equalityCounts,0);
            function result=capture(~,program)
                equalityCounts(end+1)=program.cones(1);
                result=program.defaultSolver();
            end
        end
        function changingMarginPreservesEveryDynamicsEquality(testCase)
            [ego,target,route,cfg]=encounterTestFixture.crossing();
            [~,~,problem]=collisionAvoidanceController(ego,target,route,cfg,[]);
            qp=problem.qp;prior=qp.stageProgram;
            qp.inequalityBound=qp.barrier.baseBound-.25*qp.barrier.scale;
            updated=updateAvoidanceStageBounds(qp);
            testCase.verifyEqual(updated.b(prior.rowMap.equality),prior.b(prior.rowMap.equality),AbsTol=0);
            testCase.verifyEqual(updated.b(prior.rowMap.inequality), ...
                prior.inequalityOffset+qp.inequalityBound(prior.inequalityIndices),AbsTol=0);
        end
        function continuousNormalProposalsNeedCompleteCertification(testCase)
            [ego,target,route,cfg]=encounterTestFixture.crossing();
            [~,~,problem]=collisionAvoidanceController(ego,target,route,cfg,[]);
            model=problem.qp.stageProgram.context.model;prediction=problem.prediction;plan=problem.inputPlan(:);
            [normals,information]=optimizeSeparationNormals(model,prediction,plan);
            testCase.verifyTrue(all(isfinite(information.proposalMargins),'all'));
            testCase.verifyEqual(vecnorm(cat(2,normals{:})),ones(1,numel(normals)),AbsTol=1e-12);
            prediction.separationNormals=normals;
            qp=formulateAvoidanceProblem(model,prediction,model.anchorPlan);
            [solve,qp]=solveHardCbfClf(qp,cfg);
            check=certifyAvoidancePlan(qp,prediction,model,solve.decision);
            testCase.verifyTrue(solve.feasible && check.accepted);
            testCase.verifyEqual(qp.geometry.normals,normals);
        end
        function invalidNormalCannotBecomeAnExecutableCertificate(testCase)
            [ego,target,route,cfg]=encounterTestFixture.crossing();
            [~,~,problem]=collisionAvoidanceController(ego,target,route,cfg,[]);
            prediction=problem.prediction;
            prediction.separationNormals=repmat({[0;0]},numel(prediction.cells),1);
            testCase.verifyError(@() formulateAvoidanceProblem(problem.model,prediction,problem.qp.stageProgram.context.model.anchorPlan), ...
                'collisionAvoidanceController:invalidSeparationNormal');
        end
        function worldFialaDynamicsMatchTheAuthoritativeStraightKernel(testCase)
            cfg=collisionAvoidanceControllerConfig();state=[1;-.2;-.1;8;.1;.02];input=[-.04;.02];h=1e-4;
            expected=ltvBicycleModel.fialaWorldDynamics(state,input,cfg);
            parameters=modifiedFialaTire.parameters(cfg);
            next=ltvBicycleModel.nominalKernel(state,input,h,0,0,cfg,parameters,0,false);
            testCase.verifyEqual((next(:,2)-state)/h,expected,AbsTol=.02);
            testCase.verifyTrue(all(isfinite(expected)));
        end
    end
end
