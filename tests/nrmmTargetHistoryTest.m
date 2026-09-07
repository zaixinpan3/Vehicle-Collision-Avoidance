classdef nrmmTargetHistoryTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"estimator")));
        end
    end
    methods (Test)
        function aCircularChordBoundsHeadingDespiteTangentialAcceleration(testCase)
            history = localHistory();curvature = 0.004;
            for time = 0:0.02:1
                heading = curvature*(10*time+time^2);
                position = [sin(heading);1-cos(heading)]/curvature;
                history = nrmmTargetHistory("measure",history,time,position,[0.04;0.04]);
            end
            enclosure = nrmmTargetHistory("enclose",history,1);
            testCase.verifyTrue(enclosure.heading.available);
            testCase.verifyLessThanOrEqual(abs(enclosure.heading.center-0.044),enclosure.heading.radius);
            testCase.verifyLessThan(enclosure.heading.radius,0.1);
        end
        function noisyPositionHistoryContainsConstantAcceleration(testCase)
            history = localHistory();times = 0:0.02:1;
            for time = times
                truth = [20-8*time+0.2*time^2;1+time-0.1*time^2];
                noise = 0.04*[sin(11*time);cos(17*time)];
                history = nrmmTargetHistory("measure",history,time,truth+noise,[0.04;0.04]);
            end
            enclosure = nrmmTargetHistory("enclose",history,1);
            truth = [12.2;1.9;-7.6;0.8;0.4;-0.2];
            testCase.verifyTrue(enclosure.available);
            testCase.verifyLessThanOrEqual(enclosure.lower,truth);
            testCase.verifyGreaterThanOrEqual(enclosure.upper,truth);
            testCase.verifyLessThan(max(enclosure.upper(3:4)-enclosure.lower(3:4)),1.5);
        end
        function singlePositionDoesNotInventVelocityInformation(testCase)
            history = nrmmTargetHistory("measure",localHistory(),0,[20;1],[0.04;0.04]);
            enclosure = nrmmTargetHistory("enclose",history,0);
            testCase.verifyEqual(enclosure.lower(3:4),[-20;-20],AbsTol=2e-12);
            testCase.verifyEqual(enclosure.upper(3:4),[20;20],AbsTol=2e-12);
            testCase.verifyFalse(enclosure.heading.available);
        end
        function sharedGyroTransportContainsMotionWithBoundedSensorNoise(testCase)
            history = localHistory();history.yawAccelerationMaximum = 0.6;
            design.sensors = struct("velocityNoiseMaximum",0.05,"positionNoiseMaximum",0.04, ...
                "radarNoiseMaximum",0.04,"gyroscopeNoiseMaximum",0.0015);
            design.yaw.courseModel = struct("rearAxleDistance",1.5, ...
                "singleTrackYawRateMismatchMaximum",0.1,"sideslipDomainMaximum",0.12);
            bodyVelocity = [10;0.3];rate = 0.2;
            for time = 0:0.02:1
                heading = rate*time;
                rotation = [cos(heading),-sin(heading);sin(heading),cos(heading)];
                egoPosition = [sin(heading),cos(heading)-1;1-cos(heading),sin(heading)]*bodyVelocity/rate;
                targetPosition = [20-8*time;1+time];
                input = struct("time",time,"gnssVelocity",rotation*bodyVelocity ...
                    +0.05*[sin(17*time);cos(17*time)], ...
                    "yawRate",rate+0.0015*sin(7*time), ...
                    "gnssPosition",egoPosition+0.04*[cos(11*time);sin(11*time)], ...
                    "radarRelativePosition",(rotation.'*(targetPosition-egoPosition) ...
                    +0.04*[sin(13*time);cos(13*time)]).');
                history = nrmmTargetHistory("sensor",history,input,design,1);
            end
            enclosure = nrmmTargetHistory("enclose",history,1);
            truth = [12;2;-8;1;0;0];
            testCase.verifyTrue(enclosure.available);
            testCase.verifyLessThanOrEqual(enclosure.lower,truth);
            testCase.verifyGreaterThanOrEqual(enclosure.upper,truth);
            history.yawAccelerationMaximum = Inf;
            independent = nrmmTargetHistory("enclose",history,1);
            testCase.verifyLessThanOrEqual(enclosure.upper(3:4)-enclosure.lower(3:4), ...
                independent.upper(3:4)-independent.lower(3:4)+1e-10);
        end
        function inconsistentMeasurementsDoNotProduceAnAvailableSet(testCase)
            history = nrmmTargetHistory("measure",localHistory(),0,[0;0],[0;0]);
            history = nrmmTargetHistory("measure",history,0.01,[10;0],[0;0]);
            enclosure = nrmmTargetHistory("enclose",history,0.01);
            testCase.verifyFalse(enclosure.available);
        end
        function futureMeasurementsAreRejected(testCase)
            history = nrmmTargetHistory("measure",localHistory(),1,[0;0],[0;0]);
            testCase.verifyError(@() nrmmTargetHistory("enclose",history,0),"nrmmTargetHistory:futureMeasurement");
        end
        function oldHistoryIsRemovedWithoutChangingTheDeclaredBounds(testCase)
            history = localHistory();history.duration = 0.1;
            history = nrmmTargetHistory("measure",history,0,[0;0],[0.04;0.04]);
            history = nrmmTargetHistory("measure",history,1,[0;0],[0.04;0.04]);
            testCase.verifyEqual(history.time,1,AbsTol=0);
            testCase.verifyEqual(history.speedMaximum,20,AbsTol=0);
        end
    end
end

function history = localHistory()
    domain = struct("speedMaximum",20,"accelerationNormBound",3, ...
        "yawRateMaximum",0.1,"scalarAccelerationMaximum",2);
    history = nrmmTargetHistory("initialize",domain,0,2);
end
