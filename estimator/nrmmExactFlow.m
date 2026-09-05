function [next, physical] = nrmmExactFlow(state, duration, yawIncrement, translation)
% nrmmExactFlow Exact constant-A, constant-curvature target flow.
% state = [rhoX; rhoY; courseRelativeToEgo; speed; acceleration; curvature].
% translation is the ego displacement in the START ego frame (m).
% yawIncrement is an integrated ego yaw increment (rad), not a yaw angle.
% sinc below is unnormalized: sin(z)/z. Speed must stay positive throughout.

    arguments
        state (6, 1) double {mustBeFinite}
        duration (1, 1) double {mustBeFinite, mustBeNonnegative}
        yawIncrement (1, 1) double {mustBeFinite} = 0
        translation (2, 1) double {mustBeFinite} = [0; 0]
    end
    speed = state(4)+state(5)*duration;
    if min(state(4), speed) <= 0
        error("nrmmExactFlow:nonpositiveSpeed", ...
            "The exact coordinate chart requires positive speed on the interval.");
    end
    distance = state(4)*duration+0.5*state(5)*duration^2;
    turn = state(6)*distance;
    halfTurn = turn/2;
    if abs(halfTurn) < 1.0e-4
        scale = 1-halfTurn^2/6+halfTurn^4/120-halfTurn^6/5040;
    else
        scale = sin(halfTurn)/halfTurn;
    end
    direction = state(3)+halfTurn;
    displacement = distance*scale*[cos(direction); sin(direction)];
    rotation = [cos(yawIncrement), sin(yawIncrement); ...
        -sin(yawIncrement), cos(yawIncrement)];
    next = [rotation*(state(1:2)+displacement-translation); ...
        state(3)+turn-yawIncrement; speed; state(5:6)];
    course = [cos(next(3)); sin(next(3))];
    normal = [-course(2); course(1)];
    physical = [next(1:2); speed*course; ...
        next(5)*course+next(6)*speed^2*normal];
end
