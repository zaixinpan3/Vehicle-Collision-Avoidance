classdef predictiveVelocityGeometryTest < matlab.unittest.TestCase
    % Conditional geometry checks, not a certificate for future innovations.
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function slowingChangesTheYawColumnWithoutScalingNoiseOrResidual(testCase)
            u=localFixture();fast=[zeros(3,1);8;0;0];slow=fast;slow(4)=.3;
            a=predictiveSafetyGeometry.egoErrorGenerator(fast,u);
            b=predictiveSafetyGeometry.egoErrorGenerator(slow,u);
            testCase.verifyEqual(a(4:5,4:5),b(4:5,4:5));
            testCase.verifyLessThan(abs(b(5,3)),abs(a(5,3))/20);
            testCase.verifyGreaterThanOrEqual(diag(b(4:5,6:7)),abs(u.velocityGeometry.measurementOffset));
            testCase.verifyGreaterThanOrEqual(sum(abs(b(4:5,:)),2),repmat(u.velocityGeometry.noiseRadius,2,1));
            testCase.verifyEqual(a([1:3,6],:),b([1:3,6],:));
        end
        function accelerationAndLateralMotionChangeTheRotationSensitivity(testCase)
            u=localFixture();x=[0;0;0;14;2;0];
            g=predictiveSafetyGeometry.egoErrorGenerator(x,u);
            v=x(4:5);
            testCase.verifyEqual(g(4:5,3),[v(2);-v(1)]*u.velocityGeometry.yawRadius);
            testCase.verifyGreaterThan(sum(abs(g(5,:))),.8);
        end
        function theConditionalRotatedSensorSetIsContainedAtEverySpeed(testCase)
            u=localFixture();geometry=u.velocityGeometry;
            for speed=[0,.3,2,8,15]
                x=[0;0;.4;speed;.03;0];g=predictiveSafetyGeometry.egoErrorGenerator(x,u);
                for angle=linspace(-geometry.yawRadius,geometry.yawRadius,15)
                    rotation=[cos(angle),sin(angle);-sin(angle),cos(angle)];
                    for direction=linspace(-pi,pi,15)
                        noise=geometry.noiseRadius*[cos(direction);sin(direction)];
                        for sign=[1,1,-1,-1;1,-1,1,-1]
                            v=x(4:5)+abs(geometry.measurementOffset).*sign;
                            error=rotation*(v+noise)-x(4:5);
                            residual=error-g(4:5,3)*angle/geometry.yawRadius;
                            testCase.verifyLessThanOrEqual(abs(residual),sum(abs(g(4:5,4:end)),2)+1e-12);
                        end
                    end
                end
            end
        end
        function unfactoredUserEnclosuresRemainUnchanged(testCase)
            u=localFixture();u=rmfield(u,'velocityGeometry');
            testCase.verifyEqual(predictiveSafetyGeometry.egoErrorGenerator(zeros(6,1),u),u.egoGenerator);
        end
        function predictionRetainsTheCurrentHoldAndSchedulesTheSlowerTail(testCase)
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
                'controller',struct('horizonSteps',40,'sampleTime',.05), ...
                'solver',struct('timeLimitSeconds',10)));
            ego=struct('position',[0;0],'yaw',0,'speed',8,'lateralVelocity',0,'yawRate',0);
            [~,~,prediction]=collisionAvoidanceController(ego,[],[],cfg,[]);
            model=prediction.model;anchor=model.linearization;
            anchor.inputs=repmat([0;-.4],1,size(anchor.inputs,2));
            for k=1:size(anchor.inputs,2)
                anchor.states(:,k+1)=nonlinearBicycleModel.sample(anchor.states(:,k),anchor.inputs(:,k),cfg);
            end
            factors=localFixture();
            model.uncertainty.egoGenerator=predictiveSafetyGeometry.egoErrorGenerator(model.initialState,factors);
            model.uncertainty.collisionGenerator=model.uncertainty.egoGenerator;
            model.uncertainty.velocityGeometry=factors.velocityGeometry;
            [~,~,scheduled]=solvePredictiveControl('probe',struct('operation','formulate', ...
                'model',model,'anchor',anchor,'inherited',false));
            model.uncertainty=rmfield(model.uncertainty,'velocityGeometry');
            [~,~,fixed]=solvePredictiveControl('probe',struct('operation','formulate', ...
                'model',model,'anchor',anchor,'inherited',false));
            testCase.verifyEqual(scheduled.uncertaintyPrediction.egoStateRadius(:,1:3), ...
                fixed.uncertaintyPrediction.egoStateRadius(:,1:3),AbsTol=1e-12);
            testCase.verifyLessThan(scheduled.uncertaintyPrediction.egoStateRadius(5,end), ...
                fixed.uncertaintyPrediction.egoStateRadius(5,end)/2);
            testCase.verifyTrue(scheduled.uncertaintyPrediction.futurePrimitiveBudgetsAssumed);
            testCase.verifyFalse(scheduled.uncertaintyPrediction.futureObserverDynamicsIncluded);
        end
    end
end
function u=localFixture()
    g=zeros(6,8);g(1:3,1:3)=diag([.04,.04,.06]);g(6,8)=.0015;
    geometry=struct('kind',"rotatedGnssVelocity-v1",'referenceVelocity',[8;0], ...
        'measurementOffset',[.012;-.004],'noiseRadius',.05,'yawRadius',.06);
    u=struct('egoGenerator',g,'velocityGeometry',geometry);
end
