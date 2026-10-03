classdef nominalClfTest < matlab.unittest.TestCase
    % One analytic CLF, with a local (not global) nonlinear decrease scope.
    properties (TestParameter)
        referenceSpeed={8,15};
        curvature={0,.005};
        coordinate={1,2,3,4,5};
        direction={-1,1};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function trimHasZeroValueAndTheInitializationGuidancePreservesIt(testCase,referenceSpeed,curvature)
            [cfg,lane,reference]=localSetup(referenceSpeed,curvature);
            parameters=nonlinearBicycleModel.nominalGuidanceParameters(cfg,curvature);
            trim=[0;0;reference.state(3:6)];
            input=nonlinearBicycleModel.nominalFeedback(trim,reference.input,lane,reference,cfg,parameters);
            testCase.verifyEqual(input,reference.input,AbsTol=1e-12);
            testCase.verifyEqual(nonlinearBicycleModel.nominalValue(trim,lane,reference),0,AbsTol=1e-20);
            testCase.verifyGreaterThan(min(eig(reference.matrix)),0);
        end
        function quadraticHasStrictNonlinearDecreaseNearCruise(testCase,referenceSpeed,curvature,coordinate,direction)
            [cfg,lane,reference]=localSetup(referenceSpeed,curvature);
            scales=[cfg.clf.lateralPositionErrorScale;cfg.clf.headingErrorScale;cfg.clf.speedErrorScale; ...
                cfg.clf.lateralVelocityErrorScale;cfg.clf.yawRateErrorScale];
            e=zeros(5,1);e(coordinate)=direction*1e-3*scales(coordinate);
            x=[0;e(1);reference.state(3)+e(2);reference.state(4:6)+e(3:5)];
            input=reference.input+reference.gain*e;
            value=nonlinearBicycleModel.nominalValue(x,lane,reference);
            next=nonlinearBicycleModel.nominalValue(nonlinearBicycleModel.sample(x,input,cfg),lane,reference);
            testCase.verifyLessThanOrEqual(next-value,-cfg.nominalClf.decreaseFraction*sum((e./scales).^2)+1e-12);
        end
        function longitudinalPathPhaseDoesNotChangeTheValue(testCase,referenceSpeed,curvature)
            [~,lane,reference]=localSetup(referenceSpeed,curvature);
            values=zeros(1,3);stations=[0,47,-23];
            for index=1:numel(stations)
                [position,heading]=laneGeometry.referencePose(stations(index),.8,lane.referenceCurve);
                x=[position;heading+reference.state(3)+.2;reference.state(4:6)+[-.7;.1;.05]];
                values(index)=nonlinearBicycleModel.nominalValue(x,lane,reference);
            end
            testCase.verifyEqual(values,repmat(values(1),size(values)),RelTol=1e-10);
        end
        function retiredCostToGoAndExtraIterationSettingsAreRejected(testCase)
            for field=["evaluationSeconds","tailLevel"]
                testCase.verifyError(@()collisionAvoidanceControllerConfig(struct('nominalClf',struct(field,1))), ...
                    'collisionAvoidanceController:invalidConfiguration');
            end
            testCase.verifyError(@()collisionAvoidanceControllerConfig(struct('nonlinear',struct('maximumLinearizations',3))), ...
                'collisionAvoidanceController:invalidConfiguration');
        end
        function nominalFractionsMustLieInsideTheUnitInterval(testCase)
            for name=["decreaseFraction","lateralAccelerationFraction","frontForceFraction","brakingRatioLimit"]
                testCase.verifyError(@()collisionAvoidanceControllerConfig(struct('nominalClf',struct(name,1))), ...
                    'collisionAvoidanceController:invalidConfiguration');
            end
            testCase.verifyError(@()collisionAvoidanceControllerConfig(struct('nominalClf',struct('courseGain',0))), ...
                'collisionAvoidanceController:invalidConfiguration');
        end
    end
end

function [cfg,lane,reference]=localSetup(speed,curvature)
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed));
    lane=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200));
    reference=nonlinearBicycleModel.cruise(cfg,curvature);
end
