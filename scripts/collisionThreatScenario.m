function [state,target,road,cfg,impactTime] = collisionThreatScenario(name,controllerConfiguration)
%collisionThreatScenario Construct conflicts with ideal given-path cruise.
% Target initial conditions are obtained by reversing its prescribed motion from
% a shared future road position. Controller behavior is not used for selection.
    arguments
        name (1,1) string
        controllerConfiguration (1,1) struct = struct()
    end
    cfg = collisionAvoidanceControllerConfig(controllerConfiguration);
    % US four-lane road with 12 ft (3.6576 m) lanes and 10 ft (3.048 m) paved
    % shoulders. The ego follows the centre of the second lane from the right,
    % so lateralClearance = [right; left] = [1.5 lanes + shoulder; 2.5 lanes +
    % shoulder] from the given path. laneOffsets are the four lane centres.
    lanes = 3.6576*[-1,0,1,2];
    road = struct('centerline',[-100,0;1000,0],'lateralClearance',[8.5344;12.192],'laneOffsets',lanes);
    state = [0;0;0;cfg.referenceSpeed;0;0];
    lr = cfg.target.rearAxleDistance;
    impactTime = 1.6;curvature = 0;
    switch name
        case "headOn"
            target = [24;0;pi;8;0;0;lr;2.4;.95;0;0];
            impactTime = 24/(cfg.referenceSpeed+8);
            return;
        case "acceleratingHeadOn"
            target = [24;0;pi;8;1;0;lr;2.4;.95;0;0];
            impactTime = 48/(cfg.referenceSpeed+8+sqrt((cfg.referenceSpeed+8)^2+48));
            return;
        case "brakingLead"
            impactTime = 5;
            targetSpeed = cfg.referenceSpeed-.5*impactTime;
            targetAcceleration = -.5;beta = 0;relativeCourse = 0;
        case "crossing"
            targetSpeed = 8;targetAcceleration = 0;beta = 0;relativeCourse = -pi/2;
        case "turningCrossing"
            targetSpeed = 10;targetAcceleration = 1;beta = .05;relativeCourse = -pi/2;
        case "curvedHeadOn"
            curvature = .005;
            targetSpeed = 8;targetAcceleration = 0;beta = -asin(curvature*lr);relativeCourse = pi;
        case "curvedCrossing"
            curvature = .005;
            targetSpeed = 10;targetAcceleration = .5;beta = .04;relativeCourse = -pi/2;
        otherwise
            error('collisionThreatScenario:unknownScenario','Unknown collision threat %s.',name);
    end
    curve = struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200);
    if curvature~=0
        road = struct('referenceCurve',curve,'lateralClearance',[8.5344;12.192],'laneOffsets',lanes);
        trim = nonlinearBicycleModel.cruise(cfg,curvature);
        state = trim.state;state(1:2) = 0;
    end
    [position,heading] = laneGeometry.referencePose(cfg.referenceSpeed*impactTime,0,curve);
    impactTarget = [position;heading+relativeCourse-beta;targetSpeed;targetAcceleration;beta;lr;2.4;.95;0;0];
    target = predictiveSafetyGeometry.predictTarget(impactTarget,-impactTime);
end
