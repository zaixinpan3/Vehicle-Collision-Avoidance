classdef perceptionRangeTest < matlab.unittest.TestCase
    % Only positions inside the physical sensor disk reach the estimator.
    properties (TestParameter)
        rangeMeters={49.9,50,50.1};
        azimuth={0,pi/2,pi};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'estimator')));
        end
    end
    methods (Test)
        function aDiscardedStartupProbePreservesTheFirstPublicationAndRandomStream(testCase)
            cfg=estimatorControllerIntegrationConfig();
            ego=struct('position',[0;0],'yaw',0,'longitudinalVelocity',8, ...
                'lateralVelocity',0,'yawRate',0);
            target=@(time,unused)struct('targetPositionInertial',[20+8*time;3]);
            context=nrmmEstimatorControllerAdapter("initialize",cfg,ego,target);
            randomState=context.randomStream.State;runtime=context.runtime;
            [~,probeEgo,probeTarget,probeFrame]=nrmmEstimatorControllerAdapter( ...
                "sample",context,0,ego,target(0,ego));
            testCase.verifyTrue(context.initialFramePending);
            testCase.verifyEqual(context.randomStream.State,randomState);
            testCase.verifyEqual(context.runtime,runtime);
            [~,actualEgo,actualTarget,actualFrame]=nrmmEstimatorControllerAdapter( ...
                "sample",context,0,ego,target(0,ego));
            testCase.verifyEqual(actualEgo,probeEgo);
            testCase.verifyEqual(actualTarget,probeTarget);
            testCase.verifyEqual(actualFrame,probeFrame);
        end
        function positionsArePublishedOnlyWithinFiftyMeters(testCase,rangeMeters,azimuth)
            cfg=estimatorControllerIntegrationConfig();
            testCase.verifyEqual(cfg.sensor.radar.rangeMaximum,50);
            ego=struct('position',[0;0],'yaw',0,'longitudinalVelocity',8, ...
                'lateralVelocity',0,'yawRate',0);
            offset=rangeMeters*[cos(azimuth);sin(azimuth)];
            target=@(time,unused)struct('targetPositionInertial',offset+[8*time;0]);
            context=nrmmEstimatorControllerAdapter("initialize",cfg,ego,target);
            [~,estimate,published,frame]=nrmmEstimatorControllerAdapter("sample",context,0,ego,target(0,ego));
            visible=rangeMeters<=50;
            testCase.verifyEqual(frame.radarDetectionAvailable,visible);
            testCase.verifyEqual(~isempty(published),visible);
            testCase.verifyTrue(estimate.perception.completeWithinRange);
            testCase.verifyEqual(estimate.perception.range,50);
            if visible
                testCase.verifyTrue(all(isfinite(frame.radarRelativePosition)));
            else
                testCase.verifyTrue(all(isnan(frame.radarRelativePosition)));
            end
        end
    end
end
