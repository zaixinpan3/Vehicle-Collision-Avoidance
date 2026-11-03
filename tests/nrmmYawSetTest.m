classdef nrmmYawSetTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addEstimator(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"estimator")));
        end
    end
    methods (Test)
        function radiusAboutAnObserverIncludesItsOffsetFromTheArcCenter(testCase)
            set = nrmmSet.yaw("initialize",0.2,0.1);
            radius = nrmmSet.yaw("radiusAbout",set,-0.4);
            testCase.verifyEqual(radius,0.7,AbsTol=1e-12);
        end

        function interiorAntipodeSetsTheMaximumCircularDistance(testCase)
            set = nrmmSet.yaw("initialize",pi,0.2);
            radius = nrmmSet.yaw("radiusAbout",set,0);
            testCase.verifyEqual(radius,pi,AbsTol=0);
        end

        function disconnectedComponentsAreAllIncludedAboutTheObserver(testCase)
            first = nrmmSet.yaw("initialize",0,2.5);
            second = nrmmSet.yaw("initialize",pi,2.5);
            set = nrmmSet.yaw("intersect",first,second);
            radius = nrmmSet.yaw("radiusAbout",set,0);
            testCase.verifyEqual(radius,2.5,AbsTol=1e-12);
        end

        function invalidAndFullSetsHaveTheDeclaredObserverRadii(testCase)
            first = nrmmSet.yaw("initialize",0,0.1);
            second = nrmmSet.yaw("initialize",pi,0.1);
            empty = nrmmSet.yaw("intersect",first,second);
            full = nrmmSet.yaw("initialize",0,pi);
            testCase.verifyTrue(isinf(nrmmSet.yaw("radiusAbout",empty,0.8)));
            testCase.verifyEqual(nrmmSet.yaw("radiusAbout",full,0.8),pi,AbsTol=0);
        end

        function observerRadiusRespectsTheCircleCut(testCase)
            set = nrmmSet.yaw("initialize",pi-0.02,0.1);
            radius = nrmmSet.yaw("radiusAbout",set,-pi+0.03);
            testCase.verifyEqual(radius,0.15,AbsTol=1e-12);
        end

        function wrappingRetainsOnePhysicalArc(testCase)
            set = nrmmSet.yaw("initialize",pi-0.01,0.1);
            set = nrmmSet.yaw("propagate",set,0.03,0.02);
            testCase.verifyEqual(set.heading,-pi+0.02,AbsTol=1e-13);
            testCase.verifyEqual(set.radius,0.12,AbsTol=1e-13);
        end

        function uninformativeMeasurementPreservesThePrior(testCase)
            prior = nrmmSet.yaw("initialize",-0.4,0.2);
            full = nrmmSet.yaw("initialize",1.1,Inf);
            next = nrmmSet.yaw("intersect",prior,full);
            testCase.verifyEqual(next,prior,AbsTol=1e-13);
            testCase.verifyEqual(full.radius,pi,AbsTol=0);
        end

        function disjointMeasurementsRemainInvalidThroughPropagation(testCase)
            first = nrmmSet.yaw("initialize",0,0.1);
            second = nrmmSet.yaw("initialize",pi,0.1);
            empty = nrmmSet.yaw("intersect",first,second);
            propagated = nrmmSet.yaw("propagate",empty,0.4,1);
            retried = nrmmSet.yaw("intersect",propagated,first);
            testCase.verifyFalse(retried.valid);
            testCase.verifyEmpty(retried.intervals);
            testCase.verifyTrue(isinf(retried.radius));
        end

        function largeArcIntersectionRetainsDisconnectedComponents(testCase)
            first = nrmmSet.yaw("initialize",0,2.5);
            second = nrmmSet.yaw("initialize",pi,2.5);
            result = nrmmSet.yaw("intersect",first,second);
            testCase.verifySize(result.intervals,[2,2]);
            testCase.verifyEqual(result.intervals, ...
                [pi-2.5,2.5;2*pi-2.5,pi+2.5],AbsTol=1e-13);
            testCase.verifyEqual(result.radius,2.5,AbsTol=1e-13);
        end

        function sharedEndpointsAreTheSamePointAcrossTheCircleCut(testCase)
            first = nrmmSet.yaw("initialize",0.1,0.1);
            second = nrmmSet.yaw("initialize",-0.1,0.1);
            result = nrmmSet.yaw("intersect",first,second);
            testCase.verifyTrue(result.valid);
            testCase.verifyEqual(result.heading,0,AbsTol=1e-13);
            testCase.verifyEqual(result.radius,0,AbsTol=1e-13);
        end

        function uncertaintyGrowthCanReachTheWholeCircle(testCase)
            first = nrmmSet.yaw("initialize",0.8,0.2);
            result = nrmmSet.yaw("propagate",first,0.3,pi);
            testCase.verifyEqual(result.intervals,[0,2*pi],AbsTol=0);
            testCase.verifyEqual(result.radius,pi,AbsTol=0);
        end
    end
end
