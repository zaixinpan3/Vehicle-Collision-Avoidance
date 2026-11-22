function campaign = runCollisionThreatValidation(options)
%runCollisionThreatValidation Run physically qualified raw-sensor encounters.
    arguments
        options.Frames (1,1) double {mustBeInteger,mustBePositive} = 100
        options.Speeds (1,:) double {mustBePositive,mustBeFinite} = 15
        options.Scenarios (1,:) string = ["stoppedLead","trafficBrakingLead","trafficHeadOn","trafficCrossing"]
        options.OutputDirectory (1,1) string = string(tempname)
        options.ControllerConfiguration (1,1) struct = struct()
    end
    campaign=runJointOutputFeedbackScenario(Scenarios=options.Scenarios,Speeds=options.Speeds, ...
        Frames=options.Frames,ControllerConfiguration=options.ControllerConfiguration, ...
        OutputDirectory=options.OutputDirectory);
end
