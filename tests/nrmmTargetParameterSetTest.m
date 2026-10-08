classdef nrmmTargetParameterSetTest < matlab.unittest.TestCase
    % Certified constant-parameter set of an NRMM target (nrmmTargetParameterSet).
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"estimator")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"controller")));
        end
    end
    methods (Test)
        function containsTheTruthAndNarrowsTheObserverSet(testCase)
            fixture = localEncounter(7);
            previous = [];
            for time = [0.5,1.0,1.5]
                [history,center,truth] = localWindow(fixture,time);
                set = nrmmTargetParameterSet(history,time,center,fixture.domain,fixture.prior(truth), ...
                    previous,fixture.options);
                testCase.verifyTrue(set.available,set.reason);
                testCase.verifyGreaterThanOrEqual(truth,set.center+set.lower-1e-9);
                testCase.verifyLessThanOrEqual(truth,set.center+set.upper+1e-9);
                previous = set;
            end
            % Course and speed are well inside the observer's prior radii.
            testCase.verifyLessThan(max(abs([set.lower(3),set.upper(3)])),0.6*0.43);
            testCase.verifyLessThan((set.upper(4)-set.lower(4))/2,0.6*3.3);
        end
        function constantParametersAreNestedAcrossSamples(testCase)
            fixture = localEncounter(11);
            [history,center,truth] = localWindow(fixture,1.0);
            first = nrmmTargetParameterSet(history,1.0,center,fixture.domain,fixture.prior(truth),[],fixture.options);
            [history,center,truth] = localWindow(fixture,1.05);
            second = nrmmTargetParameterSet(history,1.05,center,fixture.domain,fixture.prior(truth),first,fixture.options);
            testCase.assertTrue(first.available && second.available);
            testCase.verifyGreaterThanOrEqual(second.accelerationInterval(1),first.accelerationInterval(1));
            testCase.verifyLessThanOrEqual(second.accelerationInterval(2),first.accelerationInterval(2));
            testCase.verifyGreaterThanOrEqual(second.curvatureInterval(1),first.curvatureInterval(1));
            testCase.verifyLessThanOrEqual(second.curvatureInterval(2),first.curvatureInterval(2));
        end
        function requiresACurrentDetectionAndATransportBound(testCase)
            fixture = localEncounter(3);
            [history,center,truth] = localWindow(fixture,0.5);
            stale = nrmmTargetParameterSet(history,0.51,center,fixture.domain,fixture.prior(truth),[],fixture.options);
            testCase.verifyFalse(stale.available);
            testCase.verifyEqual(stale.reason,"noCurrentDetection");
            history.yawAccelerationMaximum = Inf;
            unbounded = nrmmTargetParameterSet(history,0.5,center,fixture.domain,fixture.prior(truth),[],fixture.options);
            testCase.verifyFalse(unbounded.available);
            testCase.verifyEqual(unbounded.reason,"noTransportBound");
        end
        function detectionsOutsideTheContractAreInconsistent(testCase)
            fixture = localEncounter(5);
            [history,center,truth] = localWindow(fixture,0.6);
            % A lateral jump of 3 m half-way through the window is not a
            % constant-parameter path.
            for index = 1:floor(numel(history.records)/2)
                history.records(index).relativePosition = history.records(index).relativePosition+[0;3];
            end
            set = nrmmTargetParameterSet(history,0.6,center,fixture.domain,fixture.prior(truth),[],fixture.options);
            testCase.verifyFalse(set.available);
            testCase.verifyEqual(set.reason,"inconsistentMeasurements");
        end
    end
end

function fixture = localEncounter(seed)
% Ego on a gentle constant-rate turn at 15 m/s; an oncoming NRMM target with
% A = -0.5 m/s^2 and beta = 0.03 rad. Bounded uniform sensor errors at 80 Hz,
% and a slowly varying heading error inside its certified radius.
    stream = RandStream("mt19937ar","Seed",seed);
    h = 1/80;lr = 1.6;radar = 0.04;gnss = 0.04;gyroNoise = 0.0015;b = 0.0075;
    fixture.domain = struct('speedMinimum',1,'speedMaximum',21.67,'scalarAccelerationMaximum',1.1, ...
        'sideslipMaximum',0.055,'rearAxleDistance',lr);
    fixture.options = struct('radarNoise',radar,'gnssNoise',gnss,'maximumMeasurements',32, ...
        'iterations',3,'courseSliceWidth',0.1,'curvatureSliceWidth',0.008,'maximumSlices',64);
    fixture.egoSpeed = 15;fixture.egoRate = 0.1;fixture.lr = lr;
    fixture.target = [45;3;pi;12;-0.5;0.03;lr;2.4;.95;0;0];
    fixture.times = 0:h:2;
    records = struct('time',{},'relativePosition',{},'egoPosition',{},'yawRate',{},'heading',{},'headingRadius',{});
    for time = fixture.times
        [position,yaw] = localEgo(fixture,time);
        q = predictiveSafetyGeometry.predictTarget(fixture.target,time);
        rotation = [cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
        records(end+1) = struct('time',time, ...
            'relativePosition',rotation.'*(q(1:2)-position)+radar*localDisk(stream), ...
            'egoPosition',position+gnss*localDisk(stream), ...
            'yawRate',fixture.egoRate+gyroNoise*(2*rand(stream)-1), ...
            'heading',yaw+0.6*b*cos(3*time),'headingRadius',b); %#ok<AGROW>
    end
    fixture.records = records;
    fixture.history = struct('records',records,'gyroscopeNoise',gyroNoise,'yawAccelerationMaximum',7.3);
    fixture.prior = @(truth) struct('positionRadius',0.07,'courseRadius',0.43, ...
        'speedInterval',truth(4)+[-3.3;3.3],'accelerationInterval',[-1.1;1.1], ...
        'curvatureInterval',[-0.034;0.034]);
end

function [history,center,truth] = localWindow(fixture,time)
    last = find(abs(fixture.times-time)<1e-9,1);
    history = fixture.history;history.records = fixture.records(max(1,last-160):last);
    [position,yaw] = localEgo(fixture,time);
    q = predictiveSafetyGeometry.predictTarget(fixture.target,time);
    rotation = [cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
    course = q(3)+q(6)-yaw;
    truth = [rotation.'*(q(1:2)-position);atan2(sin(course),cos(course));q(4);q(5);sin(q(6))/fixture.lr];
    % A tracker estimate with ordinary errors.
    center = struct('relativePosition',truth(1:2)+[0.03;-0.02],'course',truth(3)+0.02, ...
        'speed',truth(4)+0.3,'acceleration',truth(5)+0.4,'curvature',truth(6)-0.01);
end

function [position,yaw] = localEgo(fixture,time)
    yaw = fixture.egoRate*time;
    position = fixture.egoSpeed/fixture.egoRate*[sin(yaw);1-cos(yaw)];
end

function value = localDisk(stream)
    radius = sqrt(rand(stream));angle = 2*pi*rand(stream);
    value = radius*[cos(angle);sin(angle)];
end
