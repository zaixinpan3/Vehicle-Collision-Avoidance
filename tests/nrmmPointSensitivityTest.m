classdef nrmmPointSensitivityTest < matlab.unittest.TestCase
    properties (SetAccess=private)
        design
    end
    properties (TestParameter)
        operatingPoint=struct('gentle',[8;.1;.06;-.1;.3], ...
            'fast',[25;.5;.2;-.6;3], ...
            'strongSteering',[8;0;.6;-.8;0], ...
            'lowerClip',[8;.1;.1;-.2;100], ...
            'upperClip',[8;.1;.1;-.2;-100])
        endpoint={-1,1}
        adhesionOffset={-1e-7,0,1e-7}
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'estimator')));
            cfg=collisionAvoidanceControllerConfig();
            setup=estimatorConfigurationFromController(estimatorControllerIntegrationConfig(),cfg);
            testCase.design=synthesizeNrmmObserverGains(setup.observer);
        end
    end
    methods (Test)
        function virtualInverseToleranceBoundsItsPointError(testCase,operatingPoint)
            input=localInput(operatingPoint);
            exact=nrmmEgoCourseGeometry('pointCourse',input,testCase.design);
            input.predictionInverseTolerance=.005;
            coarse=nrmmEgoCourseGeometry('pointCourse',input,testCase.design);
            testCase.verifyLessThanOrEqual(abs(coarse.lateralMeasurement.center ...
                -exact.lateralMeasurement.center),input.predictionInverseTolerance);
            % A prediction-only option must not change the actual observer.
            actual=nrmmEgoCourseGeometry('course',input,testCase.design);
            testCase.verifyEqual(actual.lateralMeasurement.center,exact.lateralMeasurement.center);
        end
        function pointValuesAreUnchanged(testCase,operatingPoint)
            input=localInput(operatingPoint);
            [value,jac,info]=nrmmEgoCourseGeometry('pointMeasurement',input,testCase.design);
            testCase.verifyEqual(value,localReference(input,testCase.design),AbsTol=0);
            testCase.verifyTrue(all(isfinite(jac),'all'));
            testCase.verifyFalse(any(info.secantColumns));
            testCase.verifyFalse(info.nonlinearCertificate);
        end
        function smoothJacobiansMatchIndependentPerturbations(testCase,operatingPoint)
            input=localInput(operatingPoint);
            [~,jac]=nrmmEgoCourseGeometry('pointMeasurement',input,testCase.design);
            numerical=localDifference(input,testCase.design);
            testCase.verifyLessThan(norm(jac-numerical,'fro')/max(1,norm(numerical,'fro')),2e-4);
            testCase.verifyEqual(jac(:,[1,2,5]),zeros(4,3),AbsTol=1e-12);
        end
        function fullBrakingUsesAReportedInwardSecant(testCase,endpoint)
            input=localInput([8;.1;.2;endpoint;0]);m=testCase.design.egoModel;
            load=m.mass*m.gravity*m.lr/(m.lf+m.lr);
            input.bodyAcceleration(2)=endpoint*m.frictionCoefficient(1)*load*sin(.2)/m.mass;
            [value,jac,info]=nrmmEgoCourseGeometry('pointMeasurement',input,testCase.design);
            shifted=input;shifted.heldInput(2)=endpoint*(1-1e-4);
            expected=(localReference(shifted,testCase.design)-value)/(-endpoint*1e-4);
            testCase.verifyTrue(info.secantColumns(9));
            testCase.verifyEqual(jac(:,9),expected,RelTol=1e-10,AbsTol=1e-10);
            testCase.verifyTrue(all(isfinite(jac),'all'));
        end
        function standstillDoesNotPretendToHaveASmoothNormDerivative(testCase)
            input=localInput([0;0;0;0;0]);
            [value,jac,info]=nrmmEgoCourseGeometry('pointMeasurement',input,testCase.design);
            testCase.verifyEqual(value,zeros(4,1),AbsTol=1e-12);
            testCase.verifyTrue(all(info.secantColumns(3:4)));
            testCase.verifyTrue(all(isfinite(jac),'all'));
        end
        function kinematicFallbackHasTheSamePointAndDerivative(testCase)
            input=localInput([8;.1;.06;-.1;.3]);input.heldInput=[];
            [value,jac,info]=nrmmEgoCourseGeometry('pointMeasurement',input,testCase.design);
            numerical=localDifference(input,testCase.design);
            testCase.verifyEqual(value,localReference(input,testCase.design),AbsTol=0);
            testCase.verifyLessThan(norm(jac-numerical,'fro'),1e-6);
            testCase.verifyEqual(info.branch,"kinematic");
        end
        function saturatedTiresRetainTheRegularizedInverseSlope(testCase)
            [input,regularization]=localSaturatedInput(testCase.design);
            [~,~,info]=nrmmEgoCourseGeometry('pointMeasurement',input,testCase.design);
            testCase.verifyEqual(info.branch,"implicitInterior");
            testCase.verifyEqual(info.inverseSlope,-regularization,AbsTol=1e-9);
        end
        function adhesionTransitionHasConsistentSensitivities(testCase,adhesionOffset)
            input=localTransitionInput(testCase.design,adhesionOffset);
            [~,jac,info]=nrmmEgoCourseGeometry('pointMeasurement',input,testCase.design);
            numerical=localDifference(input,testCase.design);
            testCase.verifyFalse(any(info.secantColumns));
            testCase.verifyLessThan(norm(jac-numerical,'fro')/max(1,norm(numerical,'fro')),2e-4);
        end
    end
end

function input=localInput(point)
    input=struct('gnssPosition',[0;0],'gnssVelocity',point(1)*[cos(.13);sin(.13)], ...
        'yawRate',point(2),'heldInput',point(3:4),'bodyAcceleration',[0;point(5)]);
end

function value=localReference(input,design)
    course=nrmmEgoCourseGeometry('pointCourse',input,design);
    body=nrmmObserverVectorField('velocityMeasurement',input.gnssVelocity,input.yawRate, ...
        design,course.lateralMeasurement.center);
    value=[body;course.correspondence.rotationVector];
end

function jac=localDifference(input,design)
    jac=zeros(4,9);step=1e-5;
    for k=3:9
        plus=input;minus=input;
        if k<=4
            plus.gnssVelocity(k-2)=plus.gnssVelocity(k-2)+step;
            minus.gnssVelocity(k-2)=minus.gnssVelocity(k-2)-step;
        elseif k<=6
            plus.bodyAcceleration(k-4)=plus.bodyAcceleration(k-4)+step;
            minus.bodyAcceleration(k-4)=minus.bodyAcceleration(k-4)-step;
        elseif k==7
            plus.yawRate=plus.yawRate+step;minus.yawRate=minus.yawRate-step;
        elseif ~isempty(input.heldInput)
            plus.heldInput(k-7)=plus.heldInput(k-7)+step;
            minus.heldInput(k-7)=minus.heldInput(k-7)-step;
        end
        jac(:,k)=(localReference(plus,design)-localReference(minus,design))/(2*step);
    end
end

function [input,rho]=localSaturatedInput(design)
    input=localInput([8;0;0;.8;0]);m=design.egoModel;
    load=m.mass*m.gravity/(m.lf+m.lr)*[m.lr;m.lf];
    v=8*.9*sin(design.yaw.courseModel.sideslipDomainMaximum);
    rho=(m.mass*design.sensors.accelerometerNoiseMaximum ...
        +m.parameterUncertainty*sum(m.frictionCoefficient(:).*load)) ...
        /(8*sin(design.yaw.courseModel.sideslipDomainMaximum));
    force=-sum(m.frictionCoefficient(:).*load)*sqrt(1-.8^2);
    input.bodyAcceleration(2)=(force-rho*v)/m.mass;
end

function input=localTransitionInput(design,offset)
    input=localInput([8;0;0;.5;0]);m=design.egoModel;
    load=m.mass*m.gravity/(m.lf+m.lr)*[m.lr;m.lf];
    capacity=m.frictionCoefficient(:).*load*sqrt(1-.5^2);
    tangent=3*capacity(1)/m.corneringStiffness(1)*(1+offset);
    v=8*tangent/sqrt(1+tangent^2);
    q=m.corneringStiffness(:)*abs(tangent)./(3*capacity);
    forces=-capacity*sign(tangent);active=q<1;
    forces(active)=-m.corneringStiffness(active)*tangent.*(1-q(active)+q(active).^2/3);
    rho=(m.mass*design.sensors.accelerometerNoiseMaximum ...
        +m.parameterUncertainty*sum(m.frictionCoefficient(:).*load)) ...
        /(8*sin(design.yaw.courseModel.sideslipDomainMaximum));
    input.bodyAcceleration(2)=(sum(forces)-rho*v)/m.mass;
end
