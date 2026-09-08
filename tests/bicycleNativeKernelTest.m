classdef bicycleNativeKernelTest < matlab.unittest.TestCase
    properties (TestParameter)
        curvature = struct('straight',0,'left',0.01,'right',-0.01);
    end
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"solver","bicycle")));
        end
    end
    methods (Test)
        function nativeRolloutPreservesStatesAndSensitivities(testCase,curvature)
            [cfg,tire,state,input] = localData();
            [expected,a,b] = ltvBicycleModel.nominalKernel(state,input,.05,0,curvature,cfg,tire,0,true);
            [actual,nativeA,nativeB] = bicycleNominalKernelMex(state,input,.05,0,curvature,cfg,tire,0,true);
            testCase.verifyEqual(actual,expected,AbsTol=1e-10);
            testCase.verifyEqual(nativeA,a,AbsTol=1e-7);
            testCase.verifyEqual(nativeB,b,AbsTol=1e-7);
        end
        function nativeStageMatricesPreserveDisturbanceEnclosures(testCase,curvature)
            [cfg,~,state,input] = localData();
            full = collisionAvoidanceControllerConfig();cfg.tire = full.tire;
            states = repmat(state,1,size(input,2));
            curvatures = repmat(curvature,1,size(input,2));
            rate = [.2;.06;.02;2.5;5;4];
            [a,b,c,tire,reserve] = ltvBicycleModel.linearizationKernel(states,input,curvatures,cfg,0,rate,.05);
            [nativeA,nativeB,nativeC,nativeTire,nativeReserve] = ...
                bicycleLinearizationKernelMex(states,input,curvatures,cfg,0,rate,.05);
            testCase.verifyEqual(nativeA,a,AbsTol=1e-10);
            testCase.verifyEqual(nativeB,b,AbsTol=1e-10);
            testCase.verifyEqual(nativeC,c,AbsTol=1e-10);
            testCase.verifyEqual(nativeTire,tire,AbsTol=1e-9);
            testCase.verifyEqual(nativeReserve,reserve,AbsTol=1e-11);
        end

        function nativeHeldIntervalPreservesEverySweptAndEndpointEnclosure(testCase,curvature)
            [cfg,~,state,input] = localData();
            full = collisionAvoidanceControllerConfig();cfg.tire = full.tire;
            [a,b,c] = ltvBicycleModel.linearizationKernel(state,input(:,1),curvature,cfg,0,zeros(6,1),.05);
            map = zeros(6,24);held = map;held(:,1:2) = b;
            radius = [.02;.03;.001;.04;.02;.01];rate = [.2;.06;.02;2.5;5;4];
            stateLimit = [100;6;.4;18;3;1];inputLimit = ones(24,1);
            count = max(1,ceil(2*norm(a,inf)*.05));
            for order = [3,6]
                expected = stateUncertainty.heldInterval(a,held,c,map,state,radius,rate,.05, ...
                    order,stateLimit,inputLimit,zeros(6,1),count);
                actual = bicycleHeldIntervalKernelMex(a,held,c,map,state,radius,rate,.05, ...
                    order,stateLimit,inputLimit,zeros(6,1),count);
                testCase.verifyEqual(actual,expected,AbsTol=1e-10,RelTol=1e-12);
            end
        end
    end
end

function [cfg,tire,state,input] = localData()
    full = collisionAvoidanceControllerConfig();
    cfg = struct("vehicle",struct("m",full.vehicle.m,"Iz",full.vehicle.Iz, ...
        "lf",full.vehicle.lf,"lr",full.vehicle.lr,"gravity",full.vehicle.gravity), ...
        "model",struct("scheduleSpeedFloor",full.model.scheduleSpeedFloor),"roadLoad",full.roadLoad);
    parameters = modifiedFialaTire.parameters(full);
    tire = struct("corneringStiffness",parameters.corneringStiffness, ...
        "longitudinalForceScale",parameters.longitudinalForceScale);
    state = [12;.2;.05;10;.1;.02];
    input = [.02*sin((1:12)/3);.1*cos((1:12)/4)];
end
