function estimate = estimateRoadHeadingFromFeatureSeeds( ...
        seedX, seedY, seedAzimuth, seedWeight, cfg)
%ESTIMATEROADHEADINGFROMFEATURESEEDS Estimate an unoriented road tangent.
%   ESTIMATE = ESTIMATEROADHEADINGFROMFEATURESEEDS(X, Y, AZIMUTH, WEIGHT)
%   finds the longest, strongest near-linear structure among side-agnostic
%   LiDAR feature-consensus seeds. Unlike a curb-pair estimator, only one
%   reliable road-aligned structure is required. This permits an interior
%   pavement seam to stabilize road orientation without later classifying
%   that seam as a curb.
%
%   The estimator is invariant to permutation of seeds, processes positive
%   and negative lateral offsets identically, and treats heading as an
%   unoriented line in [-90, 90) degrees. Repeated seeds with the same
%   azimuth contribute only their maximum capped weight to a hypothesis.
%
%   CFG is optional and accepts:
%       searchStepDegrees          1.0 degree
%       minimumLateralDistanceM    2.5 m
%       maximumLateralDistanceM   18.0 m
%       interceptBinWidthM         0.08 m
%       seedDistanceToleranceM     0.30 m
%       minimumLineSupport        20 unique azimuths
%       minimumLineSpanM           5.0 m
%       spanSaturationM           30.0 m
%       maximumSeedWeight          4.0
%       marginSeparationDegrees    8.0 degrees

    if nargin < 5 || isempty(cfg)
        cfg = struct();
    end
    localValidateInputs(seedX, seedY, seedAzimuth, seedWeight, cfg);
    parameters = localParameters(cfg);
    seedX = double(seedX(:));
    seedY = double(seedY(:));
    seedAzimuth = double(seedAzimuth(:));
    seedWeight = double(seedWeight(:));
    estimate = localEmptyEstimate();
    if numel(seedX) < parameters.minimumLineSupport
        return
    end

    headingCandidates = -90.0:parameters.searchStepDegrees:(90.0 - eps);
    if ~any(abs(headingCandidates) <= eps)
        headingCandidates = sort([headingCandidates, 0.0]);
    end
    scoreProfile = -inf(size(headingCandidates));
    offsetProfileM = nan(size(headingCandidates));
    supportProfile = zeros(size(headingCandidates));
    spanProfileM = zeros(size(headingCandidates));
    for headingIndex = 1:numel(headingCandidates)
        headingDegrees = headingCandidates(headingIndex);
        cosine = cosd(headingDegrees);
        sine = sind(headingDegrees);
        longitudinal = cosine * seedX + sine * seedY;
        lateral = -sine * seedX + cosine * seedY;
        model = localBestOffsetModel( ...
            longitudinal, lateral, seedAzimuth, seedWeight, parameters);
        if ~model.found
            continue
        end
        scoreProfile(headingIndex) = model.score;
        offsetProfileM(headingIndex) = model.offsetM;
        supportProfile(headingIndex) = model.azimuthSupport;
        spanProfileM(headingIndex) = model.longitudinalSpanM;
    end

    finiteScore = isfinite(scoreProfile);
    if ~any(finiteScore)
        estimate.scoreProfile = localScoreTable( ...
            headingCandidates, scoreProfile, offsetProfileM, ...
            supportProfile, spanProfileM);
        return
    end
    [bestScore, bestIndex] = max(scoreProfile);
    estimate.found = true;
    estimate.headingDegrees = headingCandidates(bestIndex);
    estimate.offsetM = offsetProfileM(bestIndex);
    estimate.score = bestScore;
    estimate.azimuthSupport = supportProfile(bestIndex);
    estimate.longitudinalSpanM = spanProfileM(bestIndex);
    lineDistanceDegrees = abs(mod( ...
        headingCandidates - estimate.headingDegrees + 90.0, ...
        180.0) - 90.0);
    separatedScores = scoreProfile( ...
        finiteScore ...
        & lineDistanceDegrees >= parameters.marginSeparationDegrees);
    if ~isempty(separatedScores)
        estimate.scoreMargin = bestScore - max(separatedScores);
    end
    estimate.scoreProfile = localScoreTable( ...
        headingCandidates, scoreProfile, offsetProfileM, ...
        supportProfile, spanProfileM);
end


function localValidateInputs(x, y, azimuth, weight, cfg)
    values = {x, y, azimuth, weight};
    for valueIndex = 1:numel(values)
        value = values{valueIndex};
        if ~isnumeric(value) || ~isreal(value) || ~isvector(value)
            error( ...
                "estimateRoadHeadingFromFeatureSeeds:InvalidSeedInput", ...
                "Seed inputs must be real numeric vectors.");
        end
    end
    numberOfSeeds = numel(x);
    if numel(y) ~= numberOfSeeds ...
            || numel(azimuth) ~= numberOfSeeds ...
            || numel(weight) ~= numberOfSeeds
        error( ...
            "estimateRoadHeadingFromFeatureSeeds:InconsistentSeedCount", ...
            "All seed vectors must have equal length.");
    end
    if any(~isfinite([x(:); y(:); azimuth(:); weight(:)])) ...
            || any(weight(:) <= 0.0)
        error( ...
            "estimateRoadHeadingFromFeatureSeeds:InvalidSeedValue", ...
            "Seed values must be finite and weights must be positive.");
    end
    if ~isstruct(cfg) || ~isscalar(cfg)
        error( ...
            "estimateRoadHeadingFromFeatureSeeds:InvalidConfig", ...
            "cfg must be a scalar structure.");
    end
end


function parameters = localParameters(cfg)
    parameters = struct( ...
        "searchStepDegrees", localPositiveValue( ...
            cfg, "searchStepDegrees", 1.0), ...
        "minimumLateralDistanceM", localNonnegativeValue( ...
            cfg, "minimumLateralDistanceM", 2.5), ...
        "maximumLateralDistanceM", localPositiveValue( ...
            cfg, "maximumLateralDistanceM", 18.0), ...
        "interceptBinWidthM", localPositiveValue( ...
            cfg, "interceptBinWidthM", 0.08), ...
        "seedDistanceToleranceM", localPositiveValue( ...
            cfg, "seedDistanceToleranceM", 0.30), ...
        "minimumLineSupport", max(2, floor(localPositiveValue( ...
            cfg, "minimumLineSupport", 20))), ...
        "minimumLineSpanM", localNonnegativeValue( ...
            cfg, "minimumLineSpanM", 5.0), ...
        "spanSaturationM", localPositiveValue( ...
            cfg, "spanSaturationM", 30.0), ...
        "maximumSeedWeight", localPositiveValue( ...
            cfg, "maximumSeedWeight", 4.0), ...
        "marginSeparationDegrees", localPositiveValue( ...
            cfg, "marginSeparationDegrees", 8.0));
    if parameters.maximumLateralDistanceM ...
            < parameters.minimumLateralDistanceM
        error( ...
            "estimateRoadHeadingFromFeatureSeeds:InvalidLateralRange", ...
            "maximumLateralDistanceM must not be below " + ...
            "minimumLateralDistanceM.");
    end
end


function model = localBestOffsetModel( ...
        longitudinal, lateral, azimuth, weight, parameters)
    usable = abs(lateral) >= parameters.minimumLateralDistanceM ...
        & abs(lateral) <= parameters.maximumLateralDistanceM;
    longitudinal = longitudinal(usable);
    lateral = lateral(usable);
    azimuth = azimuth(usable);
    weight = weight(usable);
    model = localEmptyOffsetModel();
    if numel(longitudinal) < parameters.minimumLineSupport
        return
    end

    interceptBin = round(lateral / parameters.interceptBinWidthM);
    [uniqueBin, ~, binGroup] = unique(interceptBin);
    binCount = accumarray(binGroup, 1);
    minimumBinCount = max(2, ceil( ...
        parameters.minimumLineSupport ...
        * parameters.interceptBinWidthM ...
        / (2.0 * parameters.seedDistanceToleranceM)));
    viableBin = uniqueBin(binCount >= minimumBinCount);
    bestScore = -inf;
    % One shared azimuth grouping serves every candidate bin. Group maxima
    % ordered by group index match the ascending unique(azimuth) order of
    % the original per-azimuth scan, so the capped-weight sum accumulates
    % identical values in the identical order.
    [~, ~, azimuthGroup] = unique(azimuth);
    azimuthGroupCount = max(azimuthGroup);
    for binIndex = 1:numel(viableBin)
        centerM = viableBin(binIndex) ...
            * parameters.interceptBinWidthM;
        inlier = abs(lateral - centerM) ...
            <= parameters.seedDistanceToleranceM;
        inlierGroups = azimuthGroup(inlier);
        if numel(inlierGroups) < parameters.minimumLineSupport
            continue
        end
        groupPresent = accumarray( ...
            inlierGroups, 1, [azimuthGroupCount, 1]) > 0;
        support = nnz(groupPresent);
        if support < parameters.minimumLineSupport
            continue
        end
        currentLongitudinal = longitudinal(inlier);
        spanM = max(currentLongitudinal) ...
            - min(currentLongitudinal);
        if spanM < parameters.minimumLineSpanM
            continue
        end

        maximumWeightByGroup = accumarray( ...
            inlierGroups, weight(inlier), ...
            [azimuthGroupCount, 1], @max, -inf);
        weightedSupport = sum(min( ...
            maximumWeightByGroup(groupPresent), ...
            parameters.maximumSeedWeight));
        spanFactor = sqrt(min( ...
            spanM, parameters.spanSaturationM) ...
            / parameters.spanSaturationM);
        score = weightedSupport * spanFactor;
        if score > bestScore
            bestScore = score;
            model.found = true;
            model.offsetM = median(lateral(inlier));
            model.azimuthSupport = support;
            model.longitudinalSpanM = spanM;
            model.score = score;
        end
    end
end


function model = localEmptyOffsetModel()
    model = struct( ...
        "found", false, ...
        "offsetM", NaN, ...
        "azimuthSupport", 0, ...
        "longitudinalSpanM", 0.0, ...
        "score", -inf);
end


function estimate = localEmptyEstimate()
    estimate = struct( ...
        "found", false, ...
        "headingDegrees", 0.0, ...
        "offsetM", NaN, ...
        "score", -inf, ...
        "scoreMargin", NaN, ...
        "azimuthSupport", 0, ...
        "longitudinalSpanM", 0.0, ...
        "scoreProfile", table());
end


function scoreTable = localScoreTable( ...
        headingDegrees, score, offsetM, azimuthSupport, ...
        longitudinalSpanM)
    scoreTable = table( ...
        headingDegrees(:), score(:), offsetM(:), ...
        azimuthSupport(:), longitudinalSpanM(:), ...
        VariableNames=[ ...
            "headingDegrees", "score", "offsetM", ...
            "azimuthSupport", "longitudinalSpanM"]);
end


function value = localPositiveValue(cfg, fieldName, defaultValue)
    value = localConfigValue(cfg, fieldName, defaultValue);
    if value <= 0.0
        error( ...
            "estimateRoadHeadingFromFeatureSeeds:InvalidConfigValue", ...
            "%s must be positive.", fieldName);
    end
end


function value = localNonnegativeValue(cfg, fieldName, defaultValue)
    value = localConfigValue(cfg, fieldName, defaultValue);
    if value < 0.0
        error( ...
            "estimateRoadHeadingFromFeatureSeeds:InvalidConfigValue", ...
            "%s must be nonnegative.", fieldName);
    end
end


function value = localConfigValue(cfg, fieldName, defaultValue)
    value = defaultValue;
    if isfield(cfg, fieldName)
        value = double(cfg.(fieldName));
    end
    if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
            || ~isfinite(value)
        error( ...
            "estimateRoadHeadingFromFeatureSeeds:InvalidConfigValue", ...
            "%s must be a finite real scalar.", fieldName);
    end
end
