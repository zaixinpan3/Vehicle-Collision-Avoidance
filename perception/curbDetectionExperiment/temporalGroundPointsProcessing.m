function [processingResult, state, diagnostics] = ...
        temporalGroundPointsProcessing( ...
        frame, egoState, sampleTime, state, cfg)
%TEMPORALGROUNDPOINTSPROCESSING Detect and track curb quadratic models.
%   This explicit-state wrapper tracks one quadratic model per curb using
%   trackCurbQuadraticModels and reports, for each side, the curve and the
%   current-frame ring-feature candidates that support it. The single-frame
%   detector is the measurement source only while no track exists.
%
%   EGOSTATE is
%       [x, xDot, xDDot, y, yDot, yDDot, psi, psiDot]
%   in a fixed inertial frame and SI units. STATE is supplied and returned
%   explicitly; pass [] to reset at a scenario boundary.
%
%   While a track exists the measurement comes from candidates associated
%   to it rather than from the unguided detector. Association is a bounded
%   fixed-point iteration: the curve refitted inside the corridor becomes
%   the reference for the next corridor, so a stale track walks onto the
%   curb one corridor at a time while a correct one converges immediately.
%   Two limits keep that a validation gate rather than a free search, and
%   both bind: the iteration budget, and a leash on total displacement from
%   the prediction.
%
%   A reported curb model is a claim about where a curb is, so it is only
%   ever asserted over road the current sweep resolved. Each returned curve
%   is cut back to the longest run of current candidates that follows it,
%   split wherever consecutive supporting candidates are further apart than
%   the permitted gap, and a curve that no candidate follows is dropped.
%   The tracker keeps its own extent unchanged: predicting over road that
%   was not observed is exactly what coasting is for, and only what is
%   reported has to be supported.
%
%   Relevant CFG fields and defaults are:
%       curbAssociationCorridorWidth              0.55 m
%       curbAssociationMaximumIterations             3
%       curbAssociationMaximumDisplacement        0.70 m
%       curbAssociationConvergenceTolerance       0.02 m
%       curbAssociationCorridorGrowth             0.10 m per metre
%       curbAssociationMaximumCorridorWidth       1.50 m
%       curbModelSupportCorridorWidth             1.00 m (ceiling)
%       curbModelSupportScaleMultiple             4.685
%       curbModelSupportMinimumWidth              0.20 m
%       curbModelSupportMaximumGap                6.00 m
%       curbFitDensityBinWidth                    1.00 m
%       curbFitDensityRatioLimit                  2.00
%
%   The permitted gap is set from how densely the sensor actually samples a
%   curb, not from how continuous a curb is. Measured along the labelled
%   curbs of the nine development frames, consecutive candidates are a
%   median 0.18 to 0.31 m apart but reach 1.27 to 9.26 m at the worst, so a
%   limit near the median would cut real curbs into fragments at ranges
%   where the rings simply do not land often enough. Tightening it to
%   2.00 m costs 0.156 of the mean positive rate and drops whole frames to
%   a quarter of their feature-layer ceiling.
%
%   Measured over the nine development-label frames, the iteration budget
%   and the leash trade off against each other along a ridge of roughly
%   constant reach: (1, 0.80 m) scores 0.761, (2, 0.70 m) 0.787 and
%   (3, 0.70 m) 0.794, while (4, 0.55 m) and (5, 0.45 m) both collapse to
%   0.705 as the association walks off the curb. Hard-negative selection is
%   zero throughout.

    if nargin < 4
        state = [];
    end
    if nargin < 5 || isempty(cfg)
        cfg = struct();
    end
    isInitializationFrame = isempty(state);

    [detectorCfg, roadHeadingPriorApplied, ...
        roadHeadingPriorDegrees] = localDetectorConfigWithHeadingPrior( ...
            cfg, state, egoState, sampleTime);
    % Association is attempted whenever a track exists. Re-acquiring from
    % the unguided detector on a fixed cadence was measured and rejected:
    % where the detector cannot resolve a curb it does not merely fail, it
    % fits a neighbouring structure, so a periodic re-anchor is a periodic
    % chance to corrupt a healthy track.
    associated = false;
    candidateMask = [];
    if roadHeadingPriorApplied
        [measurementResult, associated, candidateMask] = ...
            localAssociatedMeasurement( ...
                frame, state, egoState, roadHeadingPriorDegrees, cfg);
    end
    if ~associated
        measurementResult = groundPointsProcessing(frame, detectorCfg);
        % An association attempt that bailed after computing candidates
        % already holds the exact same (frame, cfg) candidate mask; only a
        % pre-candidate bail leaves it empty.
        if isempty(candidateMask)
            candidateMask = extractRingFeatureCurbCandidates(frame, cfg);
        end
    end
    [trackedModels, state, diagnostics] = ...
        trackCurbQuadraticModels( ...
            measurementResult.curbModels, ...
            egoState, sampleTime, state, cfg);
    % A curb model is a claim about road that this sweep actually saw, so
    % its longitudinal extent is cut back to the span over which current
    % candidates support it. Without this the extent is inherited from the
    % prediction and the curve is asserted over road the sensor never
    % resolved this frame.
    [trackedModels, sideIndices] = localBoundModelsToCandidateSupport( ...
        trackedModels, frame, candidateMask, cfg);
    if isInitializationFrame
        % Fail fast: a perception stack that cannot measure both curbs on
        % its very first frame must stop the experiment instead of
        % starting from a wrong or empty track.
        leftInitialized = localCombinedModel( ...
            trackedModels, "left").found;
        rightInitialized = localCombinedModel( ...
            trackedModels, "right").found;
        if ~leftInitialized || ~rightInitialized
            sideNames = ["left", "right"];
            missingSides = strjoin(sideNames( ...
                ~[leftInitialized, rightInitialized]), " and ");
            error( ...
                "temporalGroundPointsProcessing:" + ...
                "InitializationDetectionFailed", ...
                "The curb detector found no %s curb model on the " + ...
                "initialization frame; the experiment cannot start.", ...
                missingSides);
        end
    end
    diagnostics.roadHeadingPriorApplied = ...
        roadHeadingPriorApplied;
    diagnostics.roadHeadingPriorDegrees = ...
        roadHeadingPriorDegrees;
    if associated
        diagnostics.measurementSource = "associated-candidates";
    else
        diagnostics.measurementSource = "full-detector";
    end
    processingResult = measurementResult;
    processingResult.curbModels = trackedModels;
    processingResult.curbModels.temporal = diagnostics;

    % Reported curb points are the candidates that support the reported
    % curve, on both sides and in every update mode. Harvesting them from
    % the raw sweep along the curve instead was measured and rejected: only
    % 36.8 per cent of the points it emitted were candidates at all, so the
    % output asserted curb points the geometric feature layer had never
    % identified as such, and a coasted side kept emitting points long
    % after its own evidence was gone.
    processingResult.left = sideIndices.left;
    processingResult.right = sideIndices.right;
    processingResult.curbs = sortrows( ...
        unique([sideIndices.left; sideIndices.right], "rows"), [1, 2]);
end

function [detectorCfg, applied, headingDegrees] = ...
        localDetectorConfigWithHeadingPrior( ...
            cfg, state, egoState, sampleTime)
%LOCALDETECTORCONFIGWITHHEADINGPRIOR Propagate road heading by ego yaw.
    detectorCfg = cfg;
    applied = false;
    headingDegrees = NaN;
    if isfield(cfg, "curbRoadHeadingDegrees")
        configuredHeading = double(cfg.curbRoadHeadingDegrees);
        if isscalar(configuredHeading) && isfinite(configuredHeading)
            return
        end
    end
    requiredStateFields = [ ...
        "initialized", "previousEgoState", ...
        "roadHeadingDegrees"];
    stateIsUsable = isstruct(state) ...
        && all(isfield(state, requiredStateFields)) ...
        && isscalar(state.initialized) ...
        && logical(state.initialized) ...
        && isnumeric(state.previousEgoState) ...
        && numel(state.previousEgoState) == 8 ...
        && all(isfinite(state.previousEgoState), "all") ...
        && isscalar(state.roadHeadingDegrees) ...
        && isfinite(state.roadHeadingDegrees);
    if ~stateIsUsable
        return
    end
    egoState = reshape(double(egoState), 1, []);
    sampleTime = double(sampleTime);
    maximumSampleTime = localConfigValue( ...
        cfg, "curbTemporalMaximumSampleTime", 0.5);
    if numel(egoState) ~= 8 || any(~isfinite(egoState)) ...
            || ~isscalar(sampleTime) || ~isfinite(sampleTime) ...
            || sampleTime <= 0 || sampleTime > maximumSampleTime
        return
    end

    previousEgoState = reshape( ...
        double(state.previousEgoState), 1, 8);
    predictedHeading = localPredictedRoadHeading( ...
        state, previousEgoState, egoState);
    headingDegrees = localNormalizeLineHeading(predictedHeading);
    detectorCfg.curbRoadHeadingPriorDegrees = headingDegrees;
    applied = true;
end

function headingDegrees = localPredictedRoadHeading( ...
        state, previousEgoState, currentEgoState)
%LOCALPREDICTEDROADHEADING Propagate the tracked local curb tangent.
    previousRoadHeadingDegrees = ...
        double(state.roadHeadingDegrees);
    previousPsi = previousEgoState(7);
    currentPsi = currentEgoState(7);

    deltaWorldX = currentEgoState(1) - previousEgoState(1);
    deltaWorldY = currentEgoState(4) - previousEgoState(4);
    previousSensorX = ...
        cos(previousPsi) * deltaWorldX ...
        + sin(previousPsi) * deltaWorldY;
    previousSensorY = ...
        -sin(previousPsi) * deltaWorldX ...
        + cos(previousPsi) * deltaWorldY;
    previousRoadHeading = deg2rad( ...
        previousRoadHeadingDegrees);
    previousRoadX = ...
        cos(previousRoadHeading) * previousSensorX ...
        + sin(previousRoadHeading) * previousSensorY;

    tangentOffsets = zeros(0, 1);
    for fieldName = ["leftModel", "rightModel"]
        if ~isfield(state, fieldName)
            continue
        end
        model = state.(fieldName);
        usableModel = isstruct(model) ...
            && isfield(model, "found") ...
            && isscalar(model.found) ...
            && logical(model.found) ...
            && all(isfield(model, ["curvature", "slope"])) ...
            && all(isfinite([model.curvature, model.slope]));
        if usableModel
            tangentSlope = ...
                2.0 * double(model.curvature) ...
                    * previousRoadX ...
                + double(model.slope);
            tangentOffsets(end + 1, 1) = ...
                atan(tangentSlope); %#ok<AGROW>
        end
    end
    if isempty(tangentOffsets)
        tangentOffset = 0.0;
    else
        tangentOffset = 0.5 * atan2( ...
            mean(sin(2.0 * tangentOffsets)), ...
            mean(cos(2.0 * tangentOffsets)));
    end
    headingDegrees = previousRoadHeadingDegrees ...
        + rad2deg(tangentOffset) ...
        + rad2deg(previousPsi - currentPsi);
end

function headingDegrees = localNormalizeLineHeading(headingDegrees)
%LOCALNORMALIZELINEHEADING Map an unoriented line angle to [-90, 90).
    headingDegrees = mod(double(headingDegrees) + 90.0, 180.0) - 90.0;
end

function model = localCombinedModel(models, sideName)
    model = emptyCurbBoundaryModel();
    lineField = char(sideName);
    forwardField = [lineField, 'Forward'];
    if isfield(models, forwardField) ...
            && models.(forwardField).found
        model = models.(forwardField);
    elseif isfield(models, lineField) ...
            && models.(lineField).found
        model = models.(lineField);
    end
end

function [trackedModels, sideIndices] = ...
        localBoundModelsToCandidateSupport( ...
            trackedModels, frame, candidateMask, cfg)
%LOCALBOUNDMODELSTOCANDIDATESUPPORT Cut each curve back to seen road.
%   A reported curb model asserts where a curb is. That assertion is only
%   admissible over road the current sweep resolved, so each curve is
%   trimmed to the longest run of current candidates that follows it and is
%   dropped outright when no candidate does. Only the reported models are
%   trimmed; the tracker keeps its own extent, because prediction over
%   unobserved road is exactly what coasting is for.
    sideIndices = struct("left", zeros(0, 2), "right", zeros(0, 2));
    if isempty(candidateMask) || ~any(candidateMask, "all")
        return;
    end
    if ~isfield(trackedModels, "roadFrame") ...
            || ~isfield(trackedModels.roadFrame, "headingDegrees")
        return;
    end
    headingDegrees = double(trackedModels.roadFrame.headingDegrees);
    if ~isscalar(headingDegrees) || ~isfinite(headingDegrees)
        return;
    end

    sensorX = double(frame.x);
    sensorY = double(frame.y);
    supportMask = candidateMask & isfinite(sensorX) & isfinite(sensorY);
    if isfield(frame, "valid")
        supportMask = supportMask & logical(frame.valid);
    end
    if ~any(supportMask, "all")
        return;
    end
    [roadX, roadY] = localTransformToRoadFrame( ...
        sensorX, sensorY, headingDegrees);
    supportX = roadX(supportMask);
    supportY = roadY(supportMask);

    corridorParameters = struct( ...
        "fixedWidth", max(eps, localConfigValue( ...
            cfg, "curbModelSupportCorridorWidth", 1.00)), ...
        "scaleMultiple", max(eps, localConfigValue( ...
            cfg, "curbModelSupportScaleMultiple", 4.685)), ...
        "minimumWidth", max(eps, localConfigValue( ...
            cfg, "curbModelSupportMinimumWidth", 0.20)));
    maximumGap = max(eps, localConfigValue( ...
        cfg, "curbModelSupportMaximumGap", 6.00));
    modelFields = ["left", "leftForward", "right", "rightForward"];
    sideCorridor = struct("left", corridorParameters.fixedWidth, ...
        "right", corridorParameters.fixedWidth);
    for fieldIndex = 1:numel(modelFields)
        fieldName = modelFields(fieldIndex);
        if ~isfield(trackedModels, fieldName)
            continue;
        end
        [trackedModels.(fieldName), fieldCorridor] = ...
            localBoundModelToSupport( ...
                trackedModels.(fieldName), supportX, supportY, ...
                corridorParameters, maximumGap);
        if startsWith(fieldName, "left")
            sideCorridor.left = min(sideCorridor.left, fieldCorridor);
        else
            sideCorridor.right = min(sideCorridor.right, fieldCorridor);
        end
    end

    % The reported curb points are exactly the candidates that support the
    % reported curve. Anything else would report a point the geometric
    % feature layer never called a curb, or a point that has nothing to do
    % with the curve drawn through it.
    [supportRows, supportColumns] = find(supportMask);
    sides = ["left", "right"];
    for sideIndex = 1:numel(sides)
        sideName = sides(sideIndex);
        model = localCombinedModel(trackedModels, sideName);
        follows = localCandidatesFollowingModel( ...
            model, supportX, supportY, sideCorridor.(sideName));
        sideIndices.(sideName) = [ ...
            supportColumns(follows), supportRows(follows)];
    end
end

function follows = localCandidatesFollowingModel( ...
        model, supportX, supportY, corridorWidth)
%LOCALCANDIDATESFOLLOWINGMODEL Candidates lying along one reported curve.
    follows = false(size(supportX));
    if ~isstruct(model) || ~isfield(model, "found") || ~model.found
        return;
    end
    if ~isfinite(model.minimumX) || ~isfinite(model.maximumX)
        return;
    end
    curveY = model.curvature * supportX.^2 ...
        + model.slope * supportX + model.intercept;
    curveSlope = 2.0 * model.curvature * supportX + model.slope;
    perpendicular = abs(supportY - curveY) ./ hypot(1.0, curveSlope);
    follows = perpendicular <= corridorWidth ...
        & supportX >= model.minimumX ...
        & supportX <= model.maximumX;
end

function corridorWidth = localSupportCorridorWidth( ...
        perpendicular, withinFixed, parameters)
%LOCALSUPPORTCORRIDORWIDTH Corridor set by the curve's own residual spread.
%   A fixed corridor makes the reported points disagree with the curve they
%   are said to support. On frame 200 two candidates sit 0.56 and 0.92 m
%   off the right-hand curve: the robust fit gives them zero weight, so
%   removing them moves the curve by 0.0001 m, yet a 1.00 m corridor still
%   reports them as supporting it. Sizing the corridor from the same robust
%   scale the fit uses keeps the two statements consistent, and the fixed
%   width remains only as a ceiling.
    corridorWidth = parameters.fixedWidth;
    residual = perpendicular(withinFixed);
    if numel(residual) < 8
        return;
    end
    spread = 1.4826 * median(abs(residual - median(residual)));
    if ~isfinite(spread) || spread <= 0
        return;
    end
    corridorWidth = min(parameters.fixedWidth, ...
        max(parameters.minimumWidth, parameters.scaleMultiple * spread));
end

function [model, corridorWidth] = localBoundModelToSupport( ...
        model, supportX, supportY, corridorParameters, maximumGap)
%LOCALBOUNDMODELTOSUPPORT Trim one curve to the candidates following it.
    corridorWidth = corridorParameters.fixedWidth;
    if ~isstruct(model) || ~isfield(model, "found") || ~model.found
        return;
    end
    if ~isfinite(model.minimumX) || ~isfinite(model.maximumX)
        return;
    end
    curveY = model.curvature * supportX.^2 ...
        + model.slope * supportX + model.intercept;
    curveSlope = 2.0 * model.curvature * supportX + model.slope;
    perpendicular = abs(supportY - curveY) ./ hypot(1.0, curveSlope);
    inSpan = supportX >= model.minimumX & supportX <= model.maximumX;
    corridorWidth = localSupportCorridorWidth( ...
        perpendicular, inSpan & perpendicular <= corridorParameters.fixedWidth, ...
        corridorParameters);
    followsCurve = perpendicular <= corridorWidth & inSpan;
    if ~any(followsCurve)
        model = emptyCurbBoundaryModel();
        return;
    end

    [minimumX, maximumX] = localLongestSupportedRun( ...
        supportX(followsCurve), maximumGap);
    if ~isfinite(minimumX) || ~isfinite(maximumX) ...
            || maximumX <= minimumX
        model = emptyCurbBoundaryModel();
        return;
    end
    model.minimumX = minimumX;
    model.maximumX = maximumX;
    model.longitudinalSpan = maximumX - minimumX;
end

function [minimumX, maximumX] = localLongestSupportedRun( ...
        supportX, maximumGap)
%LOCALLONGESTSUPPORTEDRUN Longest gap-free span of supporting candidates.
%   Clamping the two endpoints alone would still let the curve bridge a
%   stretch with no candidates in it, so the span is split wherever
%   consecutive supporting candidates are further apart than MAXIMUMGAP and
%   the longest surviving segment is kept. Splitting on the gaps themselves
%   rather than on occupancy of a fixed grid is what makes the bound exact:
%   under a grid, two candidates in adjacent occupied cells can still be
%   almost two cell widths apart, so the reported curve would cross road
%   that nothing supports.
    minimumX = NaN;
    maximumX = NaN;
    if isempty(supportX)
        return;
    end
    sortedX = sort(reshape(supportX, [], 1));
    segmentStart = [1; find(diff(sortedX) > maximumGap) + 1];
    segmentEnd = [segmentStart(2:end) - 1; numel(sortedX)];
    [~, longestSegment] = max( ...
        sortedX(segmentEnd) - sortedX(segmentStart));
    minimumX = sortedX(segmentStart(longestSegment));
    maximumX = sortedX(segmentEnd(longestSegment));
end

function coefficients = localRobustQuadraticFit( ...
        sampleX, sampleY, parameters)
%LOCALROBUSTQUADRATICFIT Reweighted quadratic through the candidates.
%   Ordinary least squares gives every point the same say, so a candidate
%   far off the curb pulls the curve towards itself in proportion to its
%   distance. Weights come from each residual scaled by a robust estimate
%   of the residuals' own spread, so the rule adapts to how noisy a frame
%   is rather than assuming a fixed tolerance.
%
%   The weight function is redescending, which matters more than it might
%   seem. A Huber weight falls off as one over the scaled residual and
%   never reaches zero, so a point far outside the distribution keeps a
%   small but real vote: measured on frame 200, two candidates lying 16.5
%   and 26.7 robust standard deviations off the right-hand curve still
%   carried weights of 0.082 and 0.050 and moved it visibly. Tukey's
%   biweight reaches zero beyond its tuning constant and removes them
%   outright.
%
%   The first passes still use the Huber weight, because a redescending
%   weight applied to a badly contaminated starting fit can discard the
%   wrong points; the Huber passes buy a starting fit the biweight can
%   trust.
    % Points are weighted by the inverse of how many share their metre of
    % road, so a densely sampled near stretch does not outvote a sparse far
    % one merely for being closer. The imbalance is large: on frame 107 the
    % right boundary carries 15 candidates per metre inside 5 m and 3 per
    % metre beyond 12 m. Fitted by point count the curve misses a candidate
    % 19.6 m ahead by 1.63 m; weighted by road length it misses it by
    % 0.04 m. Without this the robust weight below makes matters worse
    % rather than better, since the sparse far points look like outliers
    % against a fit the dense near points already own.
    sampleX = sampleX(:);
    sampleY = sampleY(:);
    densityWeight = localRoadLengthWeight( ...
        sampleX, parameters.binWidth, parameters.densityRatioLimit);
    design = [sampleX.^2, sampleX, ones(numel(sampleX), 1)];
    coefficients = localWeightedFit(design, sampleY, densityWeight);
    if isempty(coefficients)
        coefficients = polyfit(sampleX, sampleY, 2);
        return;
    end
    if parameters.iterations <= 0
        return;
    end
    for iteration = 1:parameters.iterations
        residual = sampleY - design * coefficients(:);
        spread = 1.4826 * median(abs(residual - median(residual)));
        if ~isfinite(spread) || spread < 1.0e-6
            return;
        end
        if iteration <= parameters.huberIterations
            normalised = abs(residual) / (parameters.huberTuning * spread);
            weight = ones(size(normalised));
            damped = normalised > 1;
            weight(damped) = 1 ./ normalised(damped);
        else
            normalised = abs(residual) / (parameters.tukeyTuning * spread);
            weight = zeros(size(normalised));
            inside = normalised < 1;
            weight(inside) = (1 - normalised(inside).^2).^2;
        end
        if nnz(weight > 0) < 3
            return;
        end
        updated = localWeightedFit(design, sampleY, weight .* densityWeight);
        if isempty(updated)
            return;
        end
        coefficients = updated;
    end
end

function weight = localRoadLengthWeight(sampleX, binWidth, ratioLimit)
%LOCALROADLENGTHWEIGHT One over the candidates sharing a stretch of road.
%   The ratio between the lightest and heaviest point is capped. Weighting
%   purely by road length hands a thinly sampled far stretch the same say
%   as the whole near boundary, and where that far evidence is wrong it
%   wins outright: measured over the full lap, an uncapped weight drops the
%   worst frame from 396 curb points to 27 and pushes coasting up on both
%   sides. Capping keeps the far points audible without letting them decide
%   alone.
    lowest = min(sampleX);
    binIndex = floor((sampleX - lowest) / binWidth) + 1;
    occupancy = accumarray(binIndex, 1);
    perPoint = occupancy(binIndex);
    perPoint = max(perPoint, max(perPoint) / ratioLimit);
    weight = 1 ./ perPoint;
end

function coefficients = localWeightedFit(design, sampleY, weight)
%LOCALWEIGHTEDFIT Least squares under the given per-point weights.
    coefficients = [];
    if nnz(weight > 0) < 3
        return;
    end
    weighted = design .* weight;
    solved = (weighted.' * design) \ (weighted.' * sampleY);
    if any(~isfinite(solved))
        return;
    end
    coefficients = solved(:).';
end

function [roadX, roadY] = localTransformToRoadFrame( ...
        sensorX, sensorY, headingDegrees)
    cosine = cosd(headingDegrees);
    sine = sind(headingDegrees);
    roadX = cosine * sensorX + sine * sensorY;
    roadY = -sine * sensorX + cosine * sensorY;
end

function value = localConfigValue(cfg, fieldName, defaultValue)
    value = double(defaultValue);
    if ~isstruct(cfg) || ~isfield(cfg, fieldName)
        return;
    end
    candidate = double(cfg.(fieldName));
    if isscalar(candidate) && isfinite(candidate)
        value = candidate;
    end
end

function [processingResult, ready, candidateMask] = ...
        localAssociatedMeasurement( ...
            frame, state, egoState, headingDegrees, cfg)
%LOCALASSOCIATEDMEASUREMENT Associate ring-feature candidates to the track.
%   Data association is part of tracking, not a shortcut around it. The
%   single-frame detector searches the whole sweep with no prior, and where
%   a curb is far or sparsely sampled it fits the wrong structure or none at
%   all. Given a track, the candidates that matter are the ones lying along
%   the propagated curve, so each side is measured by refitting the
%   candidates inside its own predicted corridor.
    ready = false;
    processingResult = struct();
    candidateMask = [];
    if ~isstruct(state) || ~isfield(state, "leftModel") ...
            || ~isfield(state, "rightModel")
        return;
    end

    leftPrediction = localPropagateModel( ...
        state.leftModel, state, egoState, headingDegrees, -1, cfg);
    rightPrediction = localPropagateModel( ...
        state.rightModel, state, egoState, headingDegrees, 1, cfg);
    if ~leftPrediction.found && ~rightPrediction.found
        return;
    end

    candidateMask = extractRingFeatureCurbCandidates(frame, cfg);
    if ~any(candidateMask, "all")
        return;
    end

    x = double(frame.x);
    y = double(frame.y);
    z = double(frame.z);
    valid = isfinite(x) & isfinite(y) & isfinite(z);
    if isfield(frame, "valid")
        valid = valid & logical(frame.valid);
    end
    [roadX, roadY] = localTransformToRoadFrame(x, y, headingDegrees);

    [leftMask, leftModel] = localAssociateSide( ...
        leftPrediction, -1, candidateMask & valid, roadX, roadY, z, cfg);
    [rightMask, rightModel] = localAssociateSide( ...
        rightPrediction, 1, candidateMask & valid, roadX, roadY, z, cfg);
    if ~leftModel.found && ~rightModel.found
        return;
    end

    roadWidth = NaN;
    if leftModel.found && rightModel.found
        roadWidth = abs(rightModel.intercept - leftModel.intercept);
    end
    curbModels = struct( ...
        "left", emptyCurbBoundaryModel(), ...
        "right", emptyCurbBoundaryModel(), ...
        "leftForward", leftModel, ...
        "rightForward", rightModel, ...
        "roadFrame", struct( ...
            "found", true, ...
            "headingDegrees", double(headingDegrees), ...
            "source", "associated-candidates", ...
            "roadWidth", roadWidth, ...
            "leftSupport", double(leftModel.azimuthSupport), ...
            "rightSupport", double(rightModel.azimuthSupport), ...
            "pairScore", -inf, ...
            "scoreMargin", NaN), ...
        "ringFeatureFusion", struct( ...
            "enabled", true, ...
            "activationSource", "associated-candidates", ...
            "validCellCount", nnz(valid), ...
            "acceptedCellCount", nnz(candidateMask)));

    % These are the candidates this side was measured from. The caller
    % replaces them with the candidates supporting the tracked curve, which
    % is the same kind of set evaluated against the posterior geometry
    % rather than the prior.
    leftIndices = localMaskToIndices(leftMask);
    rightIndices = localMaskToIndices(rightMask);
    processingResult.left = leftIndices;
    processingResult.right = rightIndices;
    processingResult.curbs = sortrows( ...
        unique([leftIndices; rightIndices], "rows"), [1, 2]);
    processingResult.curbModels = curbModels;
    ready = true;
end

function predictedModel = localPropagateModel( ...
        model, state, egoState, headingDegrees, sideSign, cfg)
%LOCALPROPAGATEMODEL Move one tracked curve into the current road frame.
    predictedModel = emptyCurbBoundaryModel();
    if ~isstruct(model) || ~isfield(model, "found") ...
            || ~isscalar(model.found) || ~logical(model.found) ...
            || any(~isfinite([model.curvature, model.slope, ...
                model.intercept, model.minimumX, model.maximumX]))
        return;
    end
    minimumModelX = localConfigValue(cfg, "curbForwardCurveMinimumX", -45.0);
    maximumModelX = localConfigValue(cfg, "curbForwardCurveMaximumX", 45.0);
    sampleCount = max(9, floor(localConfigValue( ...
        cfg, "curbTemporalModelSampleCount", 81)));
    minimumLateral = max(0.0, localConfigValue( ...
        cfg, "curbMinLateralDistance", 2.5));

    minimumX = max(minimumModelX, model.minimumX);
    maximumX = min(maximumModelX, model.maximumX);
    if ~isfinite(minimumX) || ~isfinite(maximumX) || maximumX <= minimumX
        return;
    end
    previousRoadX = linspace(minimumX, maximumX, sampleCount).';
    previousRoadY = model.curvature * previousRoadX.^2 ...
        + model.slope * previousRoadX + model.intercept;

    previousHeading = deg2rad(double(state.roadHeadingDegrees));
    previousSensorX = cos(previousHeading) * previousRoadX ...
        - sin(previousHeading) * previousRoadY;
    previousSensorY = sin(previousHeading) * previousRoadX ...
        + cos(previousHeading) * previousRoadY;
    previousState = reshape(double(state.previousEgoState), 1, 8);
    previousPsi = previousState(7);
    worldX = previousState(1) + cos(previousPsi) * previousSensorX ...
        - sin(previousPsi) * previousSensorY;
    worldY = previousState(4) + sin(previousPsi) * previousSensorX ...
        + cos(previousPsi) * previousSensorY;

    egoState = reshape(double(egoState), 1, 8);
    currentPsi = egoState(7);
    deltaX = worldX - egoState(1);
    deltaY = worldY - egoState(4);
    currentSensorX = cos(currentPsi) * deltaX + sin(currentPsi) * deltaY;
    currentSensorY = -sin(currentPsi) * deltaX + cos(currentPsi) * deltaY;
    currentHeading = deg2rad(double(headingDegrees));
    currentRoadX = cos(currentHeading) * currentSensorX ...
        + sin(currentHeading) * currentSensorY;
    currentRoadY = -sin(currentHeading) * currentSensorX ...
        + cos(currentHeading) * currentSensorY;
    usable = isfinite(currentRoadX) & isfinite(currentRoadY) ...
        & currentRoadX >= minimumModelX & currentRoadX <= maximumModelX;
    currentRoadX = currentRoadX(usable);
    currentRoadY = currentRoadY(usable);
    if numel(currentRoadX) < 3 ...
            || max(currentRoadX) <= min(currentRoadX)
        return;
    end
    coefficients = polyfit(currentRoadX, currentRoadY, 2);
    if any(~isfinite(coefficients)) ...
            || sideSign * coefficients(3) < minimumLateral
        return;
    end
    predictedModel = model;
    predictedModel.found = true;
    predictedModel.curvature = coefficients(1);
    predictedModel.slope = coefficients(2);
    predictedModel.intercept = coefficients(3);
    predictedModel.minimumX = min(currentRoadX);
    predictedModel.maximumX = max(currentRoadX);
end

function [associatedMask, model] = localAssociateSide( ...
        prediction, sideSign, candidateMask, roadX, roadY, z, cfg)
%LOCALASSOCIATESIDE Refit one side from the candidates near its prediction.
%   Association is a fixed-point iteration rather than a single pass: the
%   curve refitted from the corridor becomes the reference for the next
%   corridor, up to a small iteration budget. This is what lets one width
%   serve every frame. Measured over the development labels a single pass
%   has no such width: the frames following a long coast improve
%   monotonically as the corridor widens while the freshly corrected ones
%   degrade monotonically, so any fixed choice sacrifices one group. Under
%   iteration a stale track walks onto the curb one corridor at a time
%   while a track that is already correct converges on the first pass and
%   never admits the far candidates that a wide corridor would.
    associatedMask = false(size(candidateMask));
    model = emptyCurbBoundaryModel();
    if ~prediction.found
        return;
    end
    % Sizing this gate from the tracked covariance was measured and
    % rejected: the process noise that the coasting budget requires keeps
    % the prior wide every frame, so a covariance-sized gate saturates at
    % its ceiling and behaves like the widest fixed corridor, which scores
    % worse. Until the prior reflects the real uncertainty the fixed width
    % is the honest choice.
    corridorWidth = max(eps, localConfigValue( ...
        cfg, "curbAssociationCorridorWidth", 0.55));
    maximumIterations = max(1, floor(localConfigValue( ...
        cfg, "curbAssociationMaximumIterations", 3)));
    convergenceTolerance = max(0.0, localConfigValue( ...
        cfg, "curbAssociationConvergenceTolerance", 0.02));
    maximumDisplacement = max(corridorWidth, localConfigValue( ...
        cfg, "curbAssociationMaximumDisplacement", 0.70));
    corridorGrowth = max(0.0, localConfigValue( ...
        cfg, "curbAssociationCorridorGrowth", 0.10));
    maximumCorridorWidth = max(corridorWidth, localConfigValue( ...
        cfg, "curbAssociationMaximumCorridorWidth", 1.50));
    parametersRobust = struct( ...
        "iterations", max(0, floor(localConfigValue( ...
            cfg, "curbFitRobustIterations", 5))), ...
        "huberIterations", max(0, floor(localConfigValue( ...
            cfg, "curbFitHuberIterations", 2))), ...
        "huberTuning", max(eps, localConfigValue( ...
            cfg, "curbFitHuberTuning", 1.345)), ...
        "tukeyTuning", max(eps, localConfigValue( ...
            cfg, "curbFitTukeyTuning", 4.685)), ...
        "binWidth", max(eps, localConfigValue( ...
            cfg, "curbFitDensityBinWidth", 1.0)), ...
        "densityRatioLimit", max(1.0, localConfigValue( ...
            cfg, "curbFitDensityRatioLimit", 2.0)));
    longitudinalMargin = max(0.0, localConfigValue( ...
        cfg, "curbTemporalExpansionLongitudinalMargin", 6.0));
    minimumSupport = max(2, floor(localConfigValue( ...
        cfg, "curbAssociationMinimumSupport", 20)));
    minimumSpan = max(0.0, localConfigValue( ...
        cfg, "curbAssociationMinimumSpan", 5.0));
    maximumCurvature = max(eps, localConfigValue( ...
        cfg, "curbForwardCurveMaxCurvature", 0.03));
    minimumLateral = max(0.0, localConfigValue( ...
        cfg, "curbMinLateralDistance", 2.5));
    spanScoreWeight = max(0.0, localConfigValue( ...
        cfg, "curbSpanScoreWeight", 2.0));
    spanScoreSaturation = max(eps, localConfigValue( ...
        cfg, "curbSpanScoreSaturation", 30.0));

    % The longitudinal window stays anchored on the prediction across all
    % iterations. Letting it follow each refit would let the association
    % extend itself as well as move, which is how a corridor walks off the
    % curb and onto whatever structure continues past its end.
    withinWindow = candidateMask ...
        & roadX >= prediction.minimumX - longitudinalMargin ...
        & roadX <= prediction.maximumX + longitudinalMargin;
    referenceCurvature = prediction.curvature;
    referenceSlope = prediction.slope;
    referenceIntercept = prediction.intercept;
    % An iterate is adopted only after it passes every check the single-pass
    % association applies, and the last adopted one survives whatever the
    % next iterate does. Abandoning the accepted result when a later
    % iterate fails was measured and is strictly harmful: it discards a
    % sound first pass and drops the whole side to the unguided detector.
    accepted = false;
    for iteration = 1:maximumIterations %#ok<FXUP>
        curveY = referenceCurvature * roadX.^2 ...
            + referenceSlope * roadX + referenceIntercept;
        curveSlope = 2.0 * referenceCurvature * roadX + referenceSlope;
        perpendicular = abs(roadY - curveY) ./ hypot(1.0, curveSlope);
        % The corridor widens beyond the span the model currently covers,
        % because that is where the prediction is an extrapolation rather
        % than a fit. Holding it at one width is self-limiting: the
        % association only offers points near the present curve, so a curve
        % that stops short can never be shown the evidence that would
        % extend it. Measured on frame 107, the right boundary's candidates
        % reach 11.8 m while the associated span stops at 8.9 m, and a
        % quadratic fitted over the full span lands within 0.33 m of a
        % point 19.6 m ahead that the short fit misses by 1.6 m.
        beyondSpan = max(0, max( ...
            prediction.minimumX - roadX, roadX - prediction.maximumX));
        localWidth = min(maximumCorridorWidth, ...
            corridorWidth + corridorGrowth * beyondSpan);
        iterationMask = withinWindow & perpendicular <= localWidth;
        if nnz(iterationMask) < minimumSupport
            break;
        end

        [~, azimuthIndices] = find(iterationMask);
        iterationSupport = numel(unique(azimuthIndices));
        iterationX = roadX(iterationMask);
        iterationSpan = max(iterationX) - min(iterationX);
        if iterationSupport < minimumSupport || iterationSpan < minimumSpan
            break;
        end
        iterationCoefficients = localRobustQuadraticFit( ...
            iterationX, roadY(iterationMask), parametersRobust);
        % The leash is what keeps iteration a validation gate rather than a
        % free search. Without a bound on the total displacement from the
        % prediction, successive corridors can walk the association onto a
        % neighbouring structure one width at a time and report it with
        % full confidence.
        displacement = abs( ...
            iterationCoefficients(3) - prediction.intercept);
        if any(~isfinite(iterationCoefficients)) ...
                || abs(iterationCoefficients(1)) > maximumCurvature ...
                || sideSign * iterationCoefficients(3) < minimumLateral ...
                || displacement > maximumDisplacement
            break;
        end

        interceptStep = abs(iterationCoefficients(3) - referenceIntercept);
        withinCorridor = iterationMask;
        coefficients = iterationCoefficients;
        support = iterationSupport;
        associatedX = iterationX;
        span = iterationSpan;
        accepted = true;
        referenceCurvature = coefficients(1);
        referenceSlope = coefficients(2);
        referenceIntercept = coefficients(3);
        if interceptStep <= convergenceTolerance
            break;
        end
    end
    if ~accepted
        return;
    end

    associatedZ = z(withinCorridor);
    model.found = true;
    model.curvature = coefficients(1);
    model.slope = coefficients(2);
    model.intercept = coefficients(3);
    model.minimumX = min(associatedX);
    model.maximumX = max(associatedX);
    model.minimumZ = min(associatedZ);
    model.maximumZ = max(associatedZ);
    model.azimuthSupport = support;
    model.longitudinalSpan = span;
    model.score = support ...
        + spanScoreWeight * min(span, spanScoreSaturation);
    associatedMask = withinCorridor;
end

function indices = localMaskToIndices(mask)
    [ringIndex, azimuthIndex] = find(mask);
    indices = sortrows([azimuthIndex, ringIndex], [1, 2]);
end
