function contract = collisionThreatContract(names, speeds, options)
%collisionThreatContract Target operating domain implied by the scenario set.
% The estimator's target premises (speed interval, |A|, |beta|, yaw rate and
% acceleration norm) are a contract about other vehicles; they cannot be
% derived from the ego vehicle. For the collision-threat experiments they
% follow from the declared scenario targets: each target keeps its constant
% tangential acceleration A and sideslip beta, and is relevant while it is
% within the radar range of the ego's nominal cruise along the given path.
% Over that window the speed V(t) = V0 + A t, the yaw rate V sin(beta)/lr
% and the acceleration norm hypot(A, V^2 sin(beta)/lr) are enumerated, and
% the hull is widened by a relative margin. The speed floor is kept positive.
    arguments
        names (1,:) string = ["headOn","acceleratingHeadOn","brakingLead","crossing", ...
            "turningCrossing","curvedHeadOn","curvedCrossing"]
        speeds (1,:) double = [8 15]
        options.WindowSeconds (1,1) double {mustBePositive} = 20
        options.Range (1,1) double {mustBePositive} = 50
        options.Margin (1,1) double {mustBeNonnegative} = 0.1
        options.SpeedFloor (1,1) double {mustBePositive} = 1
    end
    speedLow = Inf;speedHigh = 0;acceleration = 0;sideslip = 0;yawRate = 0;norm2 = 0;
    for speed = speeds
        for name = names
            cfg = collisionAvoidanceControllerConfig(struct('referenceSpeed',speed));
            [x0,q0,road,cfg] = collisionThreatScenario(name,cfg);
            curve = struct('origin',[0;0],'heading',0,'curvature',0,'length',1e4);
            if isfield(road,'referenceCurve'),curve = road.referenceCurve;curve.length = 1e4;end
            for t = 0:0.05:options.WindowSeconds
                q = predictiveSafetyGeometry.predictTarget(q0,t);
                if q(4) <= 0,break;end
                position = laneGeometry.referencePose(x0(4)*t,0,curve);
                if norm(q(1:2)-position(:)) > options.Range,continue;end
                omega = q(4)*sin(q(6))/q(7);
                speedLow = min(speedLow,q(4));speedHigh = max(speedHigh,q(4));
                acceleration = max(acceleration,abs(q(5)));sideslip = max(sideslip,abs(q(6)));
                yawRate = max(yawRate,abs(omega));norm2 = max(norm2,hypot(q(5),q(4)*omega));
            end
        end
    end
    widen = 1+options.Margin;
    contract = struct('speedMinimum',max(options.SpeedFloor,speedLow/widen), ...
        'speedMaximum',speedHigh*widen,'scalarAccelerationMaximum',max(0.1,acceleration*widen), ...
        'sideslipMaximum',max(1e-3,sideslip*widen),'yawRateMaximum',max(1e-3,yawRate*widen), ...
        'accelerationNormBound',max(0.1,norm2*widen));
end
