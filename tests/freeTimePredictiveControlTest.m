classdef freeTimePredictiveControlTest < matlab.unittest.TestCase
    % Physical-time behavior of the offline scalar-duration experiment.
    properties (Access=private)
        model
        previous
        result
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            cfg=collisionAvoidanceControllerConfig();cfg.referenceSpeed=8;
            cfg.controller.horizonSteps=8;cfg.solver.timeLimitSeconds=30;
            ego=struct('position',[0;.1],'yaw',0,'speed',8,'lateralVelocity',0,'yawRate',0);
            [~,~,prediction]=collisionAvoidanceController(ego,[],[],cfg,[]);
            testCase.model=prediction.model;
            testCase.previous=struct('inputTrajectory',repmat(prediction.model.nominalReference.input,1,33));
            testCase.result=exploreFreeTimePredictiveControl(testCase.model,testCase.previous);
            testCase.assertEqual(testCase.result.status,"completed");
        end
    end
    methods (Test)
        function durationInducedMotionAlsoCountsAsReferenceDeviation(testCase)
            r=testCase.result;
            states=r.free.solution.states-r.initialAnchor.states;
            expected=mean(sum((states(:,2:end)./[.5;.5;.1;.25;.5;.2]).^2,1));
            testCase.verifyEqual(r.free.solution.referenceStateDeviation,expected,AbsTol=1e-10);
        end
        function durationMovesTheTailWithoutChangingThePhysicalPrefix(testCase)
            r=testCase.result;prefix=r.model.cfg.controller.horizonSteps;
            sensitivity=r.freeProblem.durationStateDerivative;
            testCase.verifyEqual(sensitivity(:,1:prefix+1),zeros(6,prefix+1),AbsTol=1e-9);
            testCase.verifyEqual(sensitivity(1,end),8,AbsTol=1e-5);
            testCase.verifyEqual(sensitivity(2:6,end),zeros(5,1),AbsTol=1e-7);
            testCase.verifyEqual(r.free.durations(1:prefix),repmat(.05,1,prefix),AbsTol=0);
            testCase.verifyEqual(sum(r.free.durations),r.free.durationSeconds,AbsTol=1e-12);
            testCase.verifyLessThan(r.free.derivativeError,1e-4);
            testCase.verifyTrue(r.free.audit.rolloutAvailable);
        end
        function lockingDurationReproducesTheFixedProblem(testCase)
            r=exploreFreeTimePredictiveControl(testCase.model,testCase.previous,DurationIncreaseFraction=0);
            testCase.assertTrue(r.fixed.returned && r.free.returned);
            testCase.verifyEqual(r.free.durationSeconds,r.initialDurationSeconds,AbsTol=1e-9);
            testCase.verifyEqual(r.free.solution.inputs,r.fixed.solution.inputs,AbsTol=1e-4);
            testCase.verifyEqual(r.free.solution.safety,r.fixed.solution.safety,AbsTol=1e-5);
            testCase.verifyEqual(r.free.solution.clfSlack,r.fixed.solution.clfSlack,AbsTol=1e-4);
        end
        function durationAlsoMovesTheTargetPrediction(testCase)
            model=testCase.model;cfg=model.cfg;
            model.target=[1000;0;0;4;0;0;cfg.vehicle.lr;cfg.vehicle.length/2;cfg.vehicle.width/2;0;0];
            model.targetEpoch=model.target;model.targetEpochIndex=model.sampleIndex;
            model.targetOnset=false;
            r=exploreFreeTimePredictiveControl(model,testCase.previous);
            testCase.assertEqual(r.status,"completed");
            % The two straight vehicles close at 8-4 m/s. Time moves both
            % footprints, rather than moving only the ego at 8 m/s.
            testCase.verifyEqual(r.free.constraintTimeDerivativeNorm,4,AbsTol=1e-5);
            testCase.verifyGreaterThan(r.free.audit.minimumRectangleGapMeters,900);
        end
        function aLongerHoldPropagatesTheCurrentSpeedEnclosureLonger(testCase)
            r=testCase.result;model=r.model;anchor=r.initialAnchor;
            model.uncertainty.egoGenerator=diag([0,0,0,.1,0,0]);
            model.uncertainty.collisionGenerator=model.uncertainty.egoGenerator;
            model.uncertainty.relativeFrame=false;model.uncertainty.specified=true;
            model.predictionDurations(end)=.08;
            request=struct('operation',"formulate",'model',model,'anchor',anchor,'inherited',true);
            [~,~,checked]=solvePredictiveControl("probe",request);
            cfg=model.cfg;cfg.controller.sampleTime=.08;
            [~,~,~,~,jacobian]=nonlinearBicycleModel.hold(anchor.states(:,end-1),anchor.inputs(:,end),cfg);
            testCase.verifyEqual(checked.uncertaintyPrediction.time(end),r.initialDurationSeconds+.03,AbsTol=1e-10);
            testCase.verifyEqual(checked.uncertaintyPrediction.egoStateRadius(1,end),abs(jacobian(1,4))*.1,AbsTol=1e-12);
            testCase.verifyGreaterThan(checked.uncertaintyPrediction.egoStateRadius(1,end),.007);
        end
        function aReferenceWithoutAFreeTailIsReported(testCase)
            testCase.verifyError(@() exploreFreeTimePredictiveControl(testCase.model), ...
                'exploreFreeTimePredictiveControl:noFreeTail');
        end
        function anUnreachableTerminalBrakeTransitionIsRejected(testCase)
            model=testCase.model;model.cfg.model.brakingRatioRateMaximum=.01;
            r=exploreFreeTimePredictiveControl(model,testCase.previous,ReplanOnFailure=false);
            testCase.verifyEqual(r.status,"freeTimeSolveFailed");
            testCase.verifyFalse(r.fixed.returned);
            testCase.verifyFalse(r.free.returned);
            testCase.verifyEqual(r.free.search.stages(1).exitFlag,-2);
        end
        function anInfeasibleShiftedPathReplansAndRebuildsAtTheSameHorizon(testCase)
            model=testCase.model;cfg=model.cfg;model.cfg.controller.maximumHorizonSteps=32;
            % Physical road limits, not an artificial trust box, make this
            % sharply turning reference's affine subproblem infeasible.
            model.cfg.controller.enforceRoadBoundaries=true;
            model.road.lateralClearance=[3;3];
            model.target=[1000;0;0;4;0;0;cfg.vehicle.lr;cfg.vehicle.length/2;cfg.vehicle.width/2;0;0];
            model.targetEpoch=model.target;model.targetEpochIndex=model.sampleIndex;model.targetOnset=false;
            previous=struct('inputTrajectory',repmat([.35;0],1,33), ...
                'collisionEscapeNormals',repmat([0;-1],1,67),'collisionSide',-1);
            r=exploreFreeTimePredictiveControl(model,previous);
            testCase.assertTrue(r.returned);
            testCase.assertNumElements(r.attempts,2);
            failed=r.attempts{1};fresh=r.attempts{2};
            testCase.verifyEqual(failed.initialization,"shiftedInputRollout");
            testCase.verifyEqual(failed.free.search.stages(end).exitFlag,-2);
            testCase.verifyEqual(fresh.initialization,"plannedAvoidanceRollout");
            testCase.verifyTrue(fresh.plannerCompleted);
            testCase.verifyEqual(fresh.stageCount,failed.stageCount);
            testCase.verifyEqual(r.completedPlannerCalls,1);
            testCase.verifyEqual(r.horizonExtensions,0);
            testCase.verifyGreaterThan(norm(fresh.initialAnchor.states-failed.initialAnchor.states,Inf),1);
            % Both rebuilt references may share a valid far-target normal;
            % neither may inherit the deliberately stale supplied normals.
            stale=previous.collisionEscapeNormals(:,1:size(fresh.model.collisionEscapeNormals,2));
            testCase.verifyGreaterThan(norm(fresh.model.collisionEscapeNormals-stale,Inf),.1);
            testCase.verifyEqual(fresh.initialAnchor.states(:,1),model.initialState,AbsTol=0);
            testCase.verifyEqual(fresh.model.uncertainty,model.uncertainty);
            testCase.verifyEqual(fresh.model.solveTimer,failed.model.solveTimer);
        end
        function aFailedFreshPathIsReplannedWithMorePredictionSteps(testCase)
            model=testCase.model;cfg=model.cfg;
            model.cfg.model.brakingRatioRateMaximum=.01;model.cfg.controller.maximumHorizonSteps=48;
            model.target=[1000;0;0;4;0;0;cfg.vehicle.lr;cfg.vehicle.length/2;cfg.vehicle.width/2;0;0];
            model.targetEpoch=model.target;model.targetEpochIndex=model.sampleIndex;model.targetOnset=false;
            r=exploreFreeTimePredictiveControl(model,testCase.previous);
            testCase.verifyFalse(r.returned);
            testCase.verifyEqual(r.stopReason,"horizonLimit");
            testCase.assertNumElements(r.attempts,3);
            testCase.verifyEqual(cellfun(@(a) a.stageCount,r.attempts),[32,32,48]);
            testCase.verifyTrue(r.attempts{2}.plannerCompleted && r.attempts{3}.plannerCompleted);
            testCase.verifyEqual(r.completedPlannerCalls,2);
            testCase.verifyEqual(r.horizonExtensions,1);
            testCase.verifyEqual(r.attempts{3}.model.solveTimer,r.attempts{1}.model.solveTimer);
        end
        function anExpiredSharedDeadlineDoesNotStartAnotherPlan(testCase)
            model=testCase.model;model.cfg.solver.timeLimitSeconds=1e-12;
            r=exploreFreeTimePredictiveControl(model,testCase.previous);
            testCase.verifyFalse(r.returned);
            testCase.verifyEqual(r.stopReason,"timeLimit");
            testCase.verifyNumElements(r.attempts,1);
            testCase.verifyEqual(r.replanCount,0);
            testCase.verifyEqual(r.completedPlannerCalls,0);
        end
    end
end
