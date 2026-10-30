classdef nrmmTargetHistoryTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"estimator")));
        end
    end
    methods (Test)
        function stationaryChordsBoundSpeedWithoutAnObservableHeading(testCase)
            history=localHistory();
            for time=0:.025:.25
                position=[30;2]+.04*[sin(17*time);cos(11*time)];
                history=nrmmTargetHistory('measure',history,time,position,[.04;.04]);
            end
            interval=nrmmTargetHistory('speed',history,.25,2);
            testCase.verifyEqual(interval(1),0);
            testCase.verifyLessThan(interval(2),1.2);
            testCase.verifyFalse(nrmmTargetHistory('enclose',history,.25).heading.available);
        end
        function chordSpeedBoundsContainAcceleratingAndStoppingArcs(testCase)
            for curvature=[-.02,0,.02]
                for acceleration=[-2,0,2]
                    for initialSpeed=[0,1,6]
                        history=localHistory();history.yawRateMaximum=.5;
                        for time=0:.025:1
                            duration=time;
                            if acceleration<0,duration=min(time,-initialSpeed/acceleration);end
                            arc=initialSpeed*duration+acceleration*duration^2/2;
                            turn=curvature*arc/2;factor=1;if turn~=0,factor=sin(turn)/turn;end
                            position=arc*factor*[cos(.7+turn);sin(.7+turn)] ...
                                +.04*[sin(13*time);cos(7*time)];
                            history=nrmmTargetHistory('measure',history,time,position,[.04;.04]);
                        end
                        for age=[0,.2]
                            interval=nrmmTargetHistory('speed',history,1+age,2);
                            truth=max(0,initialSpeed+acceleration*(1+age));
                            testCase.verifyLessThanOrEqual(interval(1),truth);
                            testCase.verifyGreaterThanOrEqual(interval(2),truth);
                        end
                    end
                end
            end
        end
        function motionContractionKeepsBoundedNoiseTruthAndRejectsUnsupportedTurns(testCase)
            [result,truth,original]=localMotionContraction();
            testCase.verifyTrue(result.available);
            testCase.verifyLessThanOrEqual(result.intervals(1,:).',truth);
            testCase.verifyGreaterThanOrEqual(result.intervals(2,:).',truth);
            testCase.verifyLessThan(diff(result.intervals(:,3)),.8*diff(original(:,3)));
            testCase.verifyLessThan(diff(result.intervals(:,4)),diff(original(:,4))/2);
        end
        function aStationaryHistoryDoesNotInventAPreStopMotionLaw(testCase)
            history=localHistory();history=nrmmTargetHistory('measure',history,0,[0;0],[.04;.04]);
            history=nrmmTargetHistory('measure',history,.5,[0;0],[.04;.04]);
            history=nrmmTargetHistory('measure',history,1,[0;0],[.04;.04]);
            intervals=[0,-2,-.03,-.3;1,2,.03,.3];
            result=nrmmTargetHistory('contractMotion',history,1,intervals);
            testCase.verifyEqual(result.intervals,intervals);
            testCase.verifyEqual(result.checks,0);
        end
        function holdSpecificGyroBoundsTightenWithoutChangingTheSensorNoise(testCase)
            [tight,domain,truth]=localHoldHistory();
            testCase.verifyTrue(tight.available);
            testCase.verifyLessThanOrEqual(tight.lower,truth);
            testCase.verifyGreaterThanOrEqual(tight.upper,truth);
            testCase.verifyLessThanOrEqual(tight.upper(3:4)-tight.lower(3:4), ...
                domain.upper(3:4)-domain.lower(3:4)+1e-10);
        end
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
                history = nrmmTargetHistory("sensor",history,input,design);
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

function [result,truth,original]=localMotionContraction()
    history=localHistory();truth=[8;.4;.006;.1];
    for dt=-1:.025:0
        arc=truth(1)*dt+truth(2)*dt^2/2;turn=truth(3)*arc/2;
        factor=1;if turn~=0,factor=sin(turn)/turn;end
        position=arc*factor*[cos(truth(4)+turn);sin(truth(4)+turn)];
        noise=.04*[sin(13*dt);cos(17*dt)];
        history=nrmmTargetHistory('measure',history,dt,position+noise,[.04;.04]);
    end
    original=[6,-1.65,-.035,-.3;10,1.65,.035,.5];
    result=nrmmTargetHistory('contractMotion',history,0,original);
end

function [tight,domain,truth]=localHoldHistory()
    history=localHistory();history.yawAccelerationMaximum=.6;
    design.sensors=struct('velocityNoiseMaximum',.05,'positionNoiseMaximum',.04, ...
        'radarNoiseMaximum',.04,'gyroscopeNoiseMaximum',.0015);
    design.yaw.courseModel=struct('rearAxleDistance',1.5, ...
        'singleTrackYawRateMismatchMaximum',.1,'sideslipDomainMaximum',.12);
    body=[10;.3];rate=.2;
    for time=0:.02:1
        yaw=rate*time;rotation=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
        position=[sin(yaw),cos(yaw)-1;1-cos(yaw),sin(yaw)]*body/rate;
        target=[20-8*time;1+time];
        input=struct('time',time,'gnssVelocity',rotation*body+.05*[sin(17*time);cos(17*time)], ...
            'yawRate',rate+.0015*sin(7*time),'gnssPosition',position+.04*[cos(11*time);sin(11*time)], ...
            'radarRelativePosition',rotation'*(target-position)+.04*[sin(13*time);cos(13*time)], ...
            'yawAccelerationMaximum',0);
        history=nrmmTargetHistory('sensor',history,input,design);
    end
    tight=nrmmTargetHistory('enclose',history,1);
    domain=nrmmTargetHistory('enclose',rmfield(history,'holdYawAcceleration'),1);
    truth=[12;2;-8;1;0;0];
end

function history = localHistory()
    domain = struct("speedMaximum",20,"accelerationNormBound",3, ...
        "yawRateMaximum",0.1,"scalarAccelerationMaximum",2);
    history = nrmmTargetHistory("initialize",domain,0,2);
end
