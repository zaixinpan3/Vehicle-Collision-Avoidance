function state = nrmmTargetTruth(truth, time)
%nrmmTargetTruth Exact NRMM truth state of a declared-plant scenario target.
% truth.center is the state [p; v; a; yaw; yaw rate] at time 0; truth.speedRate
% and truth.curvature are its constant speed-rate and path curvature. The
% target follows the constant-curvature path through its initial velocity
% direction (its initial yaw when at rest), stops and holds at zero speed. The
% returned state is [p; v; a; yaw; yaw rate] at the given time.
    arguments
        truth (1,1) struct
        time (1,1) double {mustBeNonnegative}
    end
    x = truth.center;
    speed = norm(x(3:4));
    course = x(7);
    if speed>0
        course = atan2(x(4),x(3));
    end
    rate = truth.speedRate;
    curvature = truth.curvature;
    moving = time;
    if rate<0
        moving = min(time,speed/-rate);
    end
    arc = speed*moving+rate*moving^2/2;
    speedNow = max(0,speed+rate*moving);
    heading = course+curvature*arc;
    tangent = [cos(heading);sin(heading)];
    normal = [-tangent(2);tangent(1)];
    half = curvature*arc/2;
    scale = 1;
    if abs(half)>1e-8
        scale = sin(half)/half;
    end
    acceleration = rate*tangent+curvature*speedNow^2*normal;
    if speedNow==0
        acceleration = zeros(2,1);
    end
    state = [x(1:2)+arc*scale*[cos(course+half);sin(course+half)];speedNow*tangent;acceleration; ...
        x(7)+curvature*arc;curvature*speedNow];
end
