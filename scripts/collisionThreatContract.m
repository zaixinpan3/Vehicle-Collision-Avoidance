function contract = collisionThreatContract(names, speeds, options)
%collisionThreatContract Physical bounds over a declared family of experiments.
% Constant tangential acceleration makes the maximum speed occur at an
% endpoint. Absorption at zero can only reduce the yaw and acceleration bounds.
    arguments
        names (1,:) string = strings(1,0)
        speeds (1,:) double {mustBePositive,mustBeFinite} = zeros(1,0)
        options.WindowSeconds (1,1) double {mustBePositive,mustBeFinite} = 20
        options.ControllerConfiguration (1,1) struct = struct()
    end
    benchmarkSpeeds=[8,15,25];
    if isempty(names)
        % The full benchmark contract also covers the published campaign speeds.
        speeds=unique([speeds,benchmarkSpeeds]);
        names=["headOn","acceleratingHeadOn","brakingLead","crossing", ...
            "turningCrossing","curvedHeadOn","curvedCrossing","trafficHeadOn", ...
            "trafficCrossing","trafficBrakingLead","stoppedLead","hardBrakingLead","intersectionCrossing"];
    elseif isempty(speeds)
        speeds=benchmarkSpeeds;
    end
    upper=zeros(1,5);
    for speed=speeds
        cfg=collisionAvoidanceControllerConfig(options.ControllerConfiguration);
        cfg.referenceSpeed=speed;
        for name=names
            [~,target]=collisionThreatScenario(name,cfg);
            acceleration=abs(target(5));
            speedUpper=max(target(4),target(4)+options.WindowSeconds*target(5));
            slip=abs(target(6));
            yawRate=speedUpper*sin(slip)/target(7);
            upper=max(upper,[speedUpper,acceleration,slip,yawRate,hypot(acceleration,speedUpper*yawRate)]);
        end
    end
    contract=struct('speedMinimum',0,'speedMaximum',upper(1),'scalarAccelerationMaximum',upper(2), ...
        'sideslipMaximum',upper(3),'yawRateMaximum',upper(4),'accelerationNormBound',upper(5));
end
