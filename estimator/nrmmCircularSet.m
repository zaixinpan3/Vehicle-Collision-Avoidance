function result = nrmmCircularSet(action, first, second, radius)
% nrmmCircularSet Closed circular arcs as a union of rows in [-pi,pi].
% 'arc': first is centre, second is radius. 'predict': second is increment
% and radius bounds its error. 'intersect': intersection of two arc unions.
% Empty stays empty; no implicit reset or choice of a physical branch.

    switch string(action)
        case "arc"
            result = localArc(first, second);
        case "predict"
            result = zeros(0, 2);
            for index = 1:size(first, 1)
                result = [result; localArc(mean(first(index, :))+second, ...
                    diff(first(index, :))/2+radius)]; %#ok<AGROW>
            end
            result = localMerge(result);
        case "intersect"
            result = zeros(0, 2);
            for index = 1:size(first, 1)
                for other = 1:size(second, 1)
                    pair = [max(first(index, 1), second(other, 1)), ...
                        min(first(index, 2), second(other, 2))];
                    if pair(1) <= pair(2)
                        result(end+1, :) = pair; %#ok<AGROW>
                    end
                end
            end
            % -pi and pi denote the same point on the circle.
            if ~isempty(first) && ~isempty(second) ...
                    && ((any(first(:, 1) == -pi) && any(second(:, 2) == pi)) ...
                    || (any(second(:, 1) == -pi) && any(first(:, 2) == pi)))
                result = [result; -pi, -pi; pi, pi];
            end
            result = localMerge(result);
        otherwise
            error("nrmmCircularSet:invalidAction", "Unknown circular-set action.");
    end
end

function arcs = localArc(centre, radius)
    if ~isscalar(centre) || ~isfinite(centre) || ~isscalar(radius) ...
            || ~isfinite(radius) || radius < 0
        error("nrmmCircularSet:invalidArc", "Arc parameters must be finite.");
    end
    if radius >= pi
        arcs = [-pi, pi];
        return
    end
    centre = mod(centre+pi, 2*pi)-pi;
    lower = centre-radius;
    upper = centre+radius;
    if lower < -pi
        arcs = [-pi, upper; lower+2*pi, pi];
    elseif upper > pi
        arcs = [-pi, upper-2*pi; lower, pi];
    else
        arcs = [lower, upper];
    end
end

function result = localMerge(arcs)
    if isempty(arcs)
        result = zeros(0, 2);
        return
    end
    arcs = sortrows(arcs, 1);
    result = arcs(1, :);
    for index = 2:size(arcs, 1)
        if arcs(index, 1) <= result(end, 2)
            result(end, 2) = max(result(end, 2), arcs(index, 2));
        else
            result(end+1, :) = arcs(index, :); %#ok<AGROW>
        end
    end
end
