classdef fialaIntervalKernelTest < matlab.unittest.TestCase
    properties (TestParameter)
        steering = {0,.04,.4,-.12};
    end
    methods (TestClassSetup)
        function buildKernelAdapter(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            output=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            clear fialaIntervalKernelMex
            try
                mex('-R2018a','CXXFLAGS=$CXXFLAGS -std=c++17', ...
                    fullfile(root,'tests','fialaIntervalKernelMex.cpp'),'-lmpfr','-lgmp','-outdir',output.Folder);
            catch exception
                if string(exception.identifier)~="MATLAB:mex:Error" || ~contains(exception.message,"is not a MEX file")
                    rethrow(exception);
                end
            end
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(output.Folder));
        end
    end
    methods (Test)
        function directedProductsAgreeWithExhaustiveExtrema(testCase)
            [~,~,products]=localDifferentials(0);
            testCase.verifyEqual(products,100);
        end
        function directionalCurvatureContainsIndependentSecants(testCase,steering)
            [bounds,values]=localDifferentials(steering);
            testCase.verifyLessThanOrEqual(max(values-bounds(:,2),[],'all'),1e-4);
            testCase.verifyLessThanOrEqual(max(bounds(:,1)-values,[],'all'),1e-4);
        end
    end
end

function [bounds,values,products]=localDifferentials(steering)
    cfg=collisionAvoidanceControllerConfig();
    center=[0;0;0;10;0;0;steering;.1];
    direction=[0;.02;.03;.1;-.2;.15;.03;.02];radius=.02*abs(direction);
    [first,second,products]=fialaIntervalKernelMex(center-radius,center+radius,direction,fialaCertificate.parameters(cfg));
    points=center+.5*radius.*cos((1:8).'*(1:32)*sqrt(2));step=1e-3;
    middle=ltvBicycleModel.fialaWorldDynamics(points(1:6,:),points(7:8,:),cfg);
    plus=points+step*direction;minus=points-step*direction;
    positive=ltvBicycleModel.fialaWorldDynamics(plus(1:6,:),plus(7:8,:),cfg);
    negative=ltvBicycleModel.fialaWorldDynamics(minus(1:6,:),minus(7:8,:),cfg);
    bounds=[first;second];values=[(positive-negative)/(2*step);(positive-2*middle+negative)/step^2];
end
