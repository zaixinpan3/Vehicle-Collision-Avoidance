classdef nrmmYawSetTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addEstimator(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"estimator")));
        end
    end
    methods (Test)
        function wrappingRetainsOnePhysicalArc(testCase)
            set = nrmmYawSet("initialize",pi-0.01,0.1);
            set = nrmmYawSet("propagate",set,0.03,0.02);
            testCase.verifyEqual(set.heading,-pi+0.02,AbsTol=1e-13);
            testCase.verifyEqual(set.radius,0.12,AbsTol=1e-13);
        end

        function uninformativeMeasurementPreservesThePrior(testCase)
            prior = nrmmYawSet("initialize",-0.4,0.2);
            full = nrmmYawSet("initialize",1.1,Inf);
            next = nrmmYawSet("intersect",prior,full);
            testCase.verifyEqual(next,prior,AbsTol=1e-13);
            testCase.verifyEqual(full.radius,pi,AbsTol=0);
        end

        function disjointMeasurementsRemainInvalidThroughPropagation(testCase)
            first = nrmmYawSet("initialize",0,0.1);
            second = nrmmYawSet("initialize",pi,0.1);
            empty = nrmmYawSet("intersect",first,second);
            propagated = nrmmYawSet("propagate",empty,0.4,1);
            retried = nrmmYawSet("intersect",propagated,first);
            testCase.verifyFalse(retried.valid);
            testCase.verifyEmpty(retried.intervals);
            testCase.verifyTrue(isinf(retried.radius));
        end

        function largeArcIntersectionRetainsDisconnectedComponents(testCase)
            first = nrmmYawSet("initialize",0,2.5);
            second = nrmmYawSet("initialize",pi,2.5);
            result = nrmmYawSet("intersect",first,second);
            testCase.verifySize(result.intervals,[2,2]);
            testCase.verifyEqual(result.intervals, ...
                [pi-2.5,2.5;2*pi-2.5,pi+2.5],AbsTol=1e-13);
            testCase.verifyEqual(result.radius,2.5,AbsTol=1e-13);
        end

        function sharedEndpointsAreTheSamePointAcrossTheCircleCut(testCase)
            first = nrmmYawSet("initialize",0.1,0.1);
            second = nrmmYawSet("initialize",-0.1,0.1);
            result = nrmmYawSet("intersect",first,second);
            testCase.verifyTrue(result.valid);
            testCase.verifyEqual(result.heading,0,AbsTol=1e-13);
            testCase.verifyEqual(result.radius,0,AbsTol=1e-13);
        end

        function uncertaintyGrowthCanReachTheWholeCircle(testCase)
            first = nrmmYawSet("initialize",0.8,0.2);
            result = nrmmYawSet("propagate",first,0.3,pi);
            testCase.verifyEqual(result.intervals,[0,2*pi],AbsTol=0);
            testCase.verifyEqual(result.radius,pi,AbsTol=0);
        end
    end
end
