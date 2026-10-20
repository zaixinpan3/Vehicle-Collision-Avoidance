function [state,target,road,cfg,impactTime] = collisionThreatScenario(name,controllerConfiguration)
%collisionThreatScenario Construct conflicts with ideal given-path cruise.
% Target initial conditions are obtained by reversing its prescribed motion from
% a shared future road position. Controller behavior is not used for selection.
    arguments
        name (1,1) string
        controllerConfiguration (1,1) struct = struct()
    end
    cfg = collisionAvoidanceControllerConfig(controllerConfiguration);
    % Scenario geometry is independent of online road-constraint enforcement.
    % Both complete footprints must fit inside this road during the encounter.
    road = struct('centerline',[-100,0;1000,0],'lateralClearance',[9;9], ...
        'scenarioHalfWidthMeters',9);
    if ~cfg.controller.enforceRoadBoundaries,road.lateralClearance=[];end
    state = [0;0;0;cfg.referenceSpeed;0;0];
    lr = cfg.target.rearAxleDistance;
    impactTime = 1.6;curvature = 0;
    switch name
        case "headOn"
            distance = 1.6*(cfg.referenceSpeed+8);
            target = [distance;0;pi;8;0;0;lr;2.4;.95;0;0];
            impactTime = 1.6;
            return;
        case "acceleratingHeadOn"
            distance = 1.6*(cfg.referenceSpeed+8)+.5*1.6^2;
            target = [distance;0;pi;8;1;0;lr;2.4;.95;0;0];
            impactTime = 1.6;
            return;
        case "stoppedLead"
            % Fixed target: increasing ego speed creates a closing-speed test.
            target = [34;0;0;0;0;0;lr;2.4;.95;0;0];
            impactTime = 34/cfg.referenceSpeed;
            return;
        case "hardBrakingLead"
            % Ego and target speeds are independent: braking alone is
            % insufficient at the 25 m/s campaign operating point.
            target = [30;0;0;5;-2;0;lr;2.4;.95;0;0];
            closing = cfg.referenceSpeed-5;
            impactTime = (-closing+sqrt(closing^2+120))/2;
            if impactTime>2.5,impactTime=36.25/cfg.referenceSpeed;end
            return;
        case "brakingLead"
            impactTime = 5;
            targetSpeed = cfg.referenceSpeed-.5*impactTime;
            targetAcceleration = -.5;beta = 0;relativeCourse = 0;
        case "crossing"
            targetSpeed = 2.5;targetAcceleration = -1.5;beta = 0;relativeCourse = -pi/2;
        case "intersectionCrossing"
            targetSpeed=8;targetAcceleration=0;beta=0;relativeCourse=-pi/2;
            % Unlike the stopping in-strip targets, this vehicle crosses
            % continuously. Allow a physically witnessed bypass at both
            % campaign ego speeds without changing either motion online.
            impactTime=2.2;
        case "turningCrossing"
            targetSpeed = 2.5;targetAcceleration = -1.5;beta = .05;relativeCourse = -pi/2;
        case "curvedHeadOn"
            curvature = .005;
            targetSpeed = 8;targetAcceleration = 0;beta = -asin(curvature*lr);relativeCourse = pi;
        case "curvedCrossing"
            curvature = .005;
            targetSpeed = 2.5;targetAcceleration = -1.5;beta = .04;relativeCourse = -pi/2;
        otherwise
            error('collisionThreatScenario:unknownScenario','Unknown collision threat %s.',name);
    end
    curve = struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200);
    if curvature~=0
        road = struct('referenceCurve',curve,'lateralClearance',road.lateralClearance, ...
            'scenarioHalfWidthMeters',road.scenarioHalfWidthMeters);
        trim = nonlinearBicycleModel.cruise(cfg,curvature);
        state = trim.state;state(1:2) = 0;
    end
    [position,heading] = laneGeometry.referencePose(cfg.referenceSpeed*impactTime,0,curve);
    if name=="intersectionCrossing"
        center=position(1);width=road.scenarioHalfWidthMeters;reach=1000;
        road.drivablePolygons={[-reach,-width;reach,-width;reach,width;-reach,width], ...
            [center-width,-reach;center+width,-reach;center+width,reach;center-width,reach]};
    end
    impactTarget = [position;heading+relativeCourse-beta;targetSpeed;targetAcceleration;beta;lr;2.4;.95;0;0];
    % Crossing targets decelerate to rest inside the road. Constant A/beta
    % apply until rest, followed by the existing stationary NRMM extension.
    % Merely moving the old high-speed crossings inside the initial road
    % would let them leave its opposite edge during the same encounter.
    target = predictiveSafetyGeometry.predictTarget(impactTarget,-impactTime);
end
