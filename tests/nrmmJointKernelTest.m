classdef nrmmJointKernelTest < matlab.unittest.TestCase
    properties (TestParameter)
        targetPoint=struct('rest',[0;0;0;0],'belowFloor',[.2;.1;.1;.05], ...
            'regular',[8;1;.3;.1],'speedSaturation',[30;4;.4;.1], ...
            'forceSaturation',[8;1;20;10],'yawSaturation',[2;0;0;3])
        observerSize={7,15}
        radarAvailable={false,true}
        columnCount={1,56,80}
        nearlyFixed={0,1e-14}
        speed={0,.3,1.5,8,25}
        braking={-1,-.2,0,.1}
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'estimator')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            native=fullfile(root,'solver','nrmm');
            if isfolder(native)
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(native));
            end
        end
    end
    methods (Test)
        function sharedTargetFactorsPreserveTheObserverVectorField(testCase,targetPoint)
            domain=localDesign().target.domain;
            domain.rearAxleDistance=1.6;domain.sideslipMaximum=.2;
            ego=struct('bodyVelocity',[6;.1],'yawRate',.03);
            state=[20;2;targetPoint];
            [~,phi]=nrmmObserverVectorField("targetJacobian", 'evaluate',targetPoint(1:2),targetPoint(3:4),domain);
            reference=nrmmObserverVectorField("targetDerivative", state,ego,domain);
            testCase.verifyEqual(phi-ego.yawRate*[-state(6);state(5)],reference(5:6),AbsTol=1e-12);
        end
        function compiledIntegrationPreservesTheJointStateAndAllSharedTangents(testCase,observerSize,radarAvailable,columnCount)
            testCase.assumeEqual(exist('nrmmJointObserverKernelMex','file'),3, ...
                'Run scripts/buildNrmmJointKernel to exercise the optional compiled core.');
            args=localArguments(observerSize,radarAvailable,columnCount);
            expected=nrmmJointPrediction('observerRk4',args{:});
            actual=nrmmJointObserverKernelMex(args{:});
            testCase.verifyEqual(actual.point,expected.point,AbsTol=5e-12,RelTol=5e-12);
            testCase.verifyEqual(actual.gradient,expected.gradient,AbsTol=5e-12,RelTol=5e-12);
        end
        function posteriorProjectionRetainsCorrelationsAndEqualityResiduals(testCase,nearlyFixed)
            p=struct('base',nrmmSet([2;-1;.5],[1,1,nearlyFixed;1,0,0;0,0,1]), ...
                'currentGenerator',zeros(3,0),'equal',[1,1,0],'rhs',.25, ...
                'a',[0,0,1],'b',.2,'primitiveLower',-ones(3,1),'primitiveUpper',ones(3,1));
            projected=nrmmJointPrediction('posteriorOuter',p);
            [lower,upper]=bounds(projected);
            testCase.verifyEqual(lower,[2.25;-1.75;-.5],AbsTol=1e-12);
            testCase.verifyEqual(upper,[2.25;0;.7],AbsTol=1e-12);
            % An almost equality-fixed coordinate is not silently made exact.
            testCase.verifyLessThanOrEqual(lower(1),2.25-nearlyFixed);
            testCase.verifyGreaterThanOrEqual(upper(1),2.25+.2*nearlyFixed);
        end
        function compiledPlantPreservesModelAndParameterTangentsAcrossSpeedRegimes(testCase,speed,braking)
            testCase.assumeEqual(exist('nrmmJointPlantKernelMex','file'),3, ...
                'Run scripts/buildNrmmJointKernel to exercise the optional compiled core.');
            cfg=collisionAvoidanceControllerConfig();parameters=[.1;-.05;.05;-.02];
            plant=nrmmJointPrediction('plantConfiguration',cfg,parameters);
            ratio=1./(1+parameters);x=[1;2;.2;speed;.04;.01];u=[.1;braking];
            expected=nrmmJointPrediction('plantField',x,u,plant,ratio,eye(6),ones(2,6),ones(4,6));
            actual=nrmmJointPlantKernelMex(x,u,plant,ratio,eye(6),ones(2,6),ones(4,6));
            testCase.verifyEqual(actual.point,expected.point,AbsTol=1e-11,RelTol=1e-11);
            testCase.verifyEqual(actual.gradient,expected.gradient,AbsTol=1e-10,RelTol=1e-11);
        end
    end
end

function design=localDesign()
    domain=struct('speedMinimum',.5,'speedMaximum',20, ...
        'scalarAccelerationMaximum',2,'yawRateMaximum',.2, ...
        'accelerationNormBound',4,'regularizationSpeedMinimum',.5);
    design=struct('velocity',struct('gain',4),'position',struct('gain',3), ...
        'yaw',struct('correctionBandwidth',2), ...
        'target',struct('innovationGains',[3;3;1],'domain',domain));
end

function args=localArguments(count,radar,columns)
    random=RandStream('mt19937ar','Seed',41);
    state=[8;.1;1;2;1.05;1.99;.1;20;3;5;.2;.1;-.1;20.02;2.98];
    gradient=randn(random,count,columns);
    input=struct('bodyAcceleration',[-.3;.2],'yawRate',.02, ...
        'gnssVelocity',[7.98;.9],'radarDetectionAvailable',radar);
    rows=9;if count==15,rows=11;end
    dm=randn(random,rows,columns);pseudo=[8;.1;cos(.1);sin(.1)];
    args={state(1:count),gradient,input,dm,pseudo,randn(random,4,columns),localDesign(),.0025,5};
end
