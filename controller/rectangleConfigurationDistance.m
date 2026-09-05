function [signedDistance, normal, supportValue, outside] = ...
        rectangleConfigurationDistance( ...
        egoPosition, egoYaw, targetPosition, targetYaw, halfDimensions)
% rectangleConfigurationDistance The configuration obstacle of two rectangles.
%
% THE geometry of the collision problem, and the whole of stage 1. The
% Minkowski sum of the target rectangle and the ego rectangle rotated
% to the ego yaw is the exact configuration obstacle of the ego CENTRE:
% the two rectangles intersect if and only if the ego centre lies in
% it. Both being centrally symmetric, the sum is the zonotope of their
% four half-edge generators - a convex polygon of at most eight
% vertices whose edge normals are the two rectangles' own edge
% normals. No bounding box is taken of either rectangle anywhere.
%
% halfDimensions is [egoHalfLength; egoHalfWidth; targetHalfLength;
% targetHalfWidth]. Returns the SIGNED distance from the ego position
% to that polygon (positive outside by the separation, negative inside
% by the least penetration), the unit NORMAL of the supporting
% half-plane the position is separated by - the direction from the
% closest boundary point to the position outside, and the outward
% normal of the least-penetrated edge inside, which is the unified
% minimum-penetration answer stage 1 needs for a probe that has run
% into the obstacle - the SUPPORT VALUE of the polygon in that
% direction (so that normal'*z <= supportValue for every z of the
% obstacle, with equality at the closest point), and whether the
% position was outside.
%
% Called in PATH COORDINATES by stage 1 (formulateAvoidanceProblem), with the
% heading errors as the yaws and the target centre as the origin, and
% in Cartesian coordinates by the harness for the physical clearance
% readout.
%
    [vertices, faceNormal, faceBound] = localConfigurationObstacle( ...
        egoYaw, targetPosition, targetYaw, halfDimensions);
    [signedDistance, normal, outside] = localPointPolygonSignedDistance( ...
        egoPosition, vertices, faceNormal, faceBound);
    supportValue = max(normal.' * vertices);
end

function [vertices, faceNormal, faceBound] = ...
        localConfigurationObstacle( ...
        egoYaw, targetPosition, targetYaw, halfDimensions)
% The Minkowski sum of the two oriented rectangles, as the zonotope of
% their four half-edge generators: sort the generators by direction and
% walk them, which traces the eight vertices in order. Written without
% loops - stage 1 evaluates this at every node of every program, and
% the loop form measured 15 ms of a sample against 2 ms here.
    egoCosine = cos(egoYaw);
    egoSine = sin(egoYaw);
    targetCosine = cos(targetYaw);
    targetSine = sin(targetYaw);
    generator = [ ...
        halfDimensions(1)*egoCosine, -halfDimensions(2)*egoSine, ...
        halfDimensions(3)*targetCosine, -halfDimensions(4)*targetSine; ...
        halfDimensions(1)*egoSine, halfDimensions(2)*egoCosine, ...
        halfDimensions(3)*targetSine, halfDimensions(4)*targetCosine];
    % Canonical half-plane, then sorted by direction.
    generatorAngle = atan2(generator(2, :), generator(1, :));
    reverse = generatorAngle < 0.0 | generatorAngle >= pi;
    generator(:, reverse) = -generator(:, reverse);
    generatorAngle(reverse) = mod(generatorAngle(reverse), pi);
    [~, generatorOrder] = sort(generatorAngle);
    step = 2.0 * generator(:, generatorOrder);

    walk = [step, -step(:, 1:3)];
    vertices = targetPosition - sum(generator, 2) ...
        + cumsum([zeros(2, 1), walk], 2);
    edge = vertices(:, [2:8, 1]) - vertices;
    edgeLength = max(sqrt(sum(edge.^2, 1)), realmin);
    faceNormal = [edge(2, :); -edge(1, :)] ./ edgeLength;
    faceBound = sum(faceNormal .* vertices, 1).';
    faceNormal = faceNormal.';
end

function [signedDistance, normal, outside] = ...
        localPointPolygonSignedDistance( ...
        point, vertices, faceNormal, faceBound)
    violation = faceNormal * point - faceBound;
    tolerance = 100.0 * eps(max( ...
        [1.0; abs(faceBound); abs(point)]));
    outside = any(violation > tolerance);
    if ~outside
        % Inside: the least-penetrated edge is the minimum-norm way
        % out, and its outward normal is the supporting direction.
        [signedDistance, faceIdx] = max(violation);
        normal = faceNormal(faceIdx, :).';
        if abs(signedDistance) <= tolerance
            signedDistance = 0.0;
        end
        return;
    end
    edge = vertices(:, [2:8, 1]) - vertices;
    offset = point - vertices;
    parameter = sum(offset .* edge, 1) ./ max(sum(edge.^2, 1), realmin);
    parameter = min(max(parameter, 0.0), 1.0);
    difference = offset - parameter .* edge;
    distanceSquared = sum(difference.^2, 1);
    [minimumSquared, edgeIdx] = min(distanceSquared);
    signedDistance = sqrt(max(minimumSquared, 0.0));
    if signedDistance > tolerance
        normal = difference(:, edgeIdx) / signedDistance;
    else
        [~, faceIdx] = max(violation);
        normal = faceNormal(faceIdx, :).';
    end
end
