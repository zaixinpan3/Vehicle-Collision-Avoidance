function set = nrmmYawSet(action, varargin)
% nrmmYawSet Propagate and intersect closed orientation sets on the circle.
% initialize(center,radius), propagate(set,integralMeasuredRate,errorRadius),
% intersect(set,measurementSet). Intervals are a union in [0,2*pi]. The
% representative is the center of a smallest covering arc (largest gap's
% complement). An empty intersection stays empty and has infinite radius.
% Propagation requires a certified integral-error radius, including hold error
% when the measured rate is sampled. There is no yaw correction state or gain.

    switch string(action)
        case "initialize"
            center = varargin{1};
            radius = varargin{2};
            validateattributes(center, {'double'}, {'real','finite','scalar'});
            validateattributes(radius, {'double'}, {'real','scalar','nonnegative','nonnan'});
            set = localSet(localArc(center-radius,center+radius),center);
        case "propagate"
            previous = varargin{1};
            increment = varargin{2};
            radius = varargin{3};
            validateattributes(increment, {'double'}, {'real','finite','scalar'});
            validateattributes(radius, {'double'}, {'real','scalar','nonnegative','nonnan'});
            intervals = zeros(0,2);
            for index = 1:size(previous.intervals,1)
                arc = previous.intervals(index,:)+increment+[-radius,radius];
                intervals = [intervals;localArc(arc(1),arc(2))]; %#ok<AGROW>
            end
            set = localSet(intervals,previous.heading+increment);
        case "intersect"
            previous = varargin{1};
            measurement = varargin{2};
            % Include shifted copies so 0 and 2*pi denote the same point.
            intervals = zeros(0,2);
            for first = 1:size(previous.intervals,1)
                for second = 1:size(measurement.intervals,1)
                    for shift = [-2*pi,0,2*pi]
                        lower = max(previous.intervals(first,1),measurement.intervals(second,1)+shift);
                        upper = min(previous.intervals(first,2),measurement.intervals(second,2)+shift);
                        tolerance = 256*eps(2*pi);
                        if lower <= upper+tolerance
                            intervals = [intervals;localArc(min(lower,upper),max(lower,upper))]; %#ok<AGROW>
                        end
                    end
                end
            end
            set = localSet(intervals,previous.heading);
        otherwise
            error("nrmmYawSet:unknownAction","Unknown orientation-set action.");
    end
end

function intervals = localArc(lower,upper)
    width = upper-lower;
    if width >= 2*pi || ~isfinite(width)
        intervals = [0,2*pi];
        return
    end
    lower = mod(lower,2*pi);
    upper = lower+width;
    if upper <= 2*pi
        intervals = [lower,upper];
    else
        intervals = [0,upper-2*pi;lower,2*pi];
    end
end

function set = localSet(intervals,fallback)
    intervals = sortrows(intervals,1);
    merged = zeros(0,2);
    for index = 1:size(intervals,1)
        if isempty(merged) || intervals(index,1) > merged(end,2)+256*eps(2*pi)
            merged(end+1,:) = intervals(index,:); %#ok<AGROW>
        else
            merged(end,2) = max(merged(end,2),intervals(index,2));
        end
    end
    heading = fallback;
    radius = Inf;
    if ~isempty(merged)
        gaps = [merged(2:end,1);merged(1,1)+2*pi]-merged(:,2);
        [gap,index] = max(gaps);
        radius = min(pi,max(0,(2*pi-gap)/2));
        if gap > 0
            heading = merged(index,2)+gap+radius;
        end
    end
    set = struct("intervals",merged,"valid",~isempty(merged), ...
        "heading",mod(heading+pi,2*pi)-pi,"radius",radius);
end
