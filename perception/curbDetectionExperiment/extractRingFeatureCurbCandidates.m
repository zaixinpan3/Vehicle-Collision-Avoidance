function [candidateMask, diagnostics] = ...
        extractRingFeatureCurbCandidates(frame, cfg)
%EXTRACTRINGFEATURECURBCANDIDATES Curb candidates from LiDAR ring features.
%   [CANDIDATEMASK, DIAGNOSTICS] =
%   EXTRACTRINGFEATURECURBCANDIDATES(FRAME, CFG) returns the organized-grid
%   cells that carry local geometric evidence of a curb. No road frame is
%   estimated, no boundary curve is fitted, and no temporal state is used:
%   the returned cells are a per-sweep observation, and turning them into
%   curb geometry is the tracker's job.
%
%   Locality is defined by sweep row and column throughout, never by metric
%   distance, which keeps every test independent of range: adjacent columns
%   are adjacent beams however far they reach.
%
%   A cell is a candidate when all of the following hold.
%     1. The ring-feature families accept it, and it carries adjacent-ring
%        compression evidence. Compression responds to a raised curb face
%        rather than to flat pavement structure.
%     2. It lies in a height band about the road level of this same sweep.
%        A curb sits on the road surface and rises only a little above it.
%     3. It lies within the radial limit. Further out the beams land metres
%        apart and the ring-local features describe the surface too coarsely
%        to call a curb.
%     4. It has company: enough other candidates within a neighbouring row
%        and column window. A curb is continuous; clutter arrives alone.
%   A curb is a step, so the sharpest evidence for one is the second
%   difference of height across three consecutive rings at a fixed azimuth,
%   which Huang's three-beam residual measures. A constant slope
%   contributes nothing to a second difference, so unlike any test against
%   a road level it is immune to the road's own gradient and crown - the
%   very thing that lets distant pavement pass a height band, since ground
%   returns sit a median 0.02 m above the fitted road level at 4 to 8 m but
%   0.37 m at 20 to 24 m. Ranked over the labelled and user-reported points
%   it is by far the most discriminating of the 27 features, separating
%   curb from road surface at an area under the curve of 0.940 against
%   0.717 for the ring compression. It is only defined where the triplet
%   exists, on about half the curb points, so it is applied as a veto: a
%   cell reporting no step is dropped, a cell with no triplet is left
%   alone. Raising it past 0.020 m stops rejecting further road surface
%   while continuing to cost curb points.
%
%   What separates a curb from road surface here is the ring compression,
%   not the shape. Its threshold is set to the smallest value that clears
%   the road-surface population outright: over 168 labelled curb points the
%   compression runs from a tenth percentile of -0.04 to a median of 0.44,
%   while road surface ahead of the vehicle reaches 0.21 at most, so 0.25
%   sits just clear of the latter and keeps 70.2 per cent of the former.
%   Raising it further only costs curb points - 67.9 per cent at 0.30 and
%   60.1 per cent at 0.35 - without rejecting anything more.
%
%   The two populations do overlap: a fifth of labelled curb points fall
%   below 0.14, which is where road surface typically sits, so some loss is
%   not avoidable by any threshold. The height step does not separate them
%   at all either, at 0.013 against 0.007 to 0.024.
%
%   There is deliberately no test on the shape of a candidate's
%   neighbourhood. One was built, measured and removed. Its window had to
%   be sized in sweep rows and columns while the shape it judged is
%   physical, and because ring spacing grows as the square of range while
%   azimuth spacing grows linearly, no fixed window is square at more than
%   one distance: the original plus or minus eight by eight spanned 21 m by
%   1.4 m at 15 m range and so measured its own shape, scoring road surface
%   at 0.86 against a curb's 0.89. Scaling the azimuth radius with range
%   fixed the geometry but not the outcome, because the two distributions
%   overlap. With the compression threshold and the height ceiling set from
%   their own measured distributions the test earned nothing: removing it
%   raises the development-label rate from 0.4985 to 0.5158 and recovers
%   curb points it had been discarding.
%
%   The height ceiling is set the same way, from the heights curbs actually
%   reach here rather than from what a kerb usually measures. The labelled
%   curb points run to 0.59 m above road level, because the outer boundary
%   of this route is a raised promenade edge rather than a kerb, so a
%   ceiling of 0.35 m admitted only 91.7 per cent of them and cut a whole
%   stretch of the left boundary. At 0.65 m all of them are admitted and
%   raising it to 0.80 m gains nothing further.
%
%   Supported CFG fields and defaults are:
%       curbCandidateCompressionThreshold        0.40
%       curbCandidateCrossRingMinimumSlopeChange 0.020 per metre
%       curbCandidateMinimumRadialStep           0.020 m
%       curbCandidateRoadLevelMinimumRange       4.0 m
%       curbCandidateRoadLevelMaximumRange      12.0 m
%       curbCandidateHeightBelowRoad             0.50 m
%       curbCandidateHeightAboveRoad             0.65 m
%       curbCandidateMaximumRadialDistance      22.0 m
%       curbCandidateCompanyRingRadius           1 row
%       curbCandidateCompanyAzimuthRadius       16 columns
%       curbCandidateMinimumCompany              4 cells
%   CFG is also forwarded to computeLidarRingCurbFeatures and to
%   fuseLidarRingCurbFeatures, so their own options still apply.
%
%   DIAGNOSTICS reports the surviving cell count after each stage, the road
%   level used, and the fusion diagnostics structure.

    if nargin < 2 || isempty(cfg)
        cfg = struct();
    end

    parameters = localCandidateParameters(cfg);
    [features, featureValidity] = ...
        computeLidarRingCurbFeatures(frame, cfg);
    fusionConfig = localFusionConfig(cfg, parameters);
    [fusionMask, fusionDiagnostics] = fuseLidarRingCurbFeatures( ...
        features, featureValidity, fusionConfig);

    x = double(frame.x);
    y = double(frame.y);
    z = double(frame.z);
    valid = isfinite(x) & isfinite(y) & isfinite(z);
    if isfield(frame, "valid")
        valid = valid & logical(frame.valid);
    end

    radialDistance = hypot(x, y);
    roadLevelSamples = valid ...
        & radialDistance >= parameters.roadLevelMinimumRange ...
        & radialDistance <= parameters.roadLevelMaximumRange;
    if any(roadLevelSamples, "all")
        roadLevelZ = median(z(roadLevelSamples));
    else
        roadLevelZ = median(z(valid));
    end

    candidateMask = fusionMask ...
        & localCrossRingStepMask(frame, parameters) ...
        & localCurbScore(features, parameters) ...
        & valid ...
        & z >= roadLevelZ - parameters.heightBelowRoad ...
        & z <= roadLevelZ + parameters.heightAboveRoad ...
        & radialDistance <= parameters.maximumRadialDistance;
    afterEvidence = nnz(candidateMask);

    % Radial step veto. Huang's three-beam residual is the second
    % difference of height across three consecutive rings at one azimuth,
    % so a constant slope contributes nothing to it and only a step does.
    % That is what makes it immune to the road's own gradient and crown,
    % which is precisely what defeats a height gate measured against a
    % single road level: on the development and user-reported points it
    % separates curb from road surface at an area under the curve of 0.940,
    % against 0.717 for the ring compression the main gate uses. It is only
    % defined on about half the curb points, so it is applied as a veto
    % rather than a requirement: where the triplet exists and reports no
    % step the cell is dropped, and where it does not exist nothing is
    % concluded.
    radialStepAvailable = logical(featureValidity.huangHdInRadius);
    radialStep = double(features.huangHdInRadius);
    candidateMask = candidateMask ...
        & ~(radialStepAvailable ...
           & radialStep < parameters.minimumRadialStep);
    afterRadialStep = nnz(candidateMask);

    candidateMask = localKeepAccompanied(candidateMask, parameters);
    candidateMask = localKeepMetricallyAccompanied( ...
        candidateMask, frame, parameters);
    afterCompany = nnz(candidateMask);

    diagnostics = struct( ...
        "roadLevelZ", roadLevelZ, ...
        "fusionCellCount", nnz(fusionMask), ...
        "evidenceCellCount", afterEvidence, ...
        "radialStepCellCount", afterRadialStep, ...
        "companyCellCount", afterCompany, ...
        "candidateCellCount", nnz(candidateMask), ...
        "parameters", parameters, ...
        "fusion", fusionDiagnostics);
end

function parameters = localCandidateParameters(cfg)
    parameters = struct( ...
        "compressionThreshold", localConfigValue( ...
           cfg, "curbCandidateCompressionThreshold", 0.40), ...
        "roadLevelMinimumRange", localConfigValue( ...
           cfg, "curbCandidateRoadLevelMinimumRange", 4.0), ...
        "roadLevelMaximumRange", localConfigValue( ...
           cfg, "curbCandidateRoadLevelMaximumRange", 12.0), ...
        "heightBelowRoad", max(0.0, localConfigValue( ...
           cfg, "curbCandidateHeightBelowRoad", 0.50)), ...
        "heightAboveRoad", max(0.0, localConfigValue( ...
           cfg, "curbCandidateHeightAboveRoad", 0.65)), ...
        "maximumRadialDistance", max(eps, localConfigValue( ...
           cfg, "curbCandidateMaximumRadialDistance", 22.0)), ...
        "companyRingRadius", max(0, floor(localConfigValue( ...
           cfg, "curbCandidateCompanyRingRadius", 1))), ...
        "companyAzimuthRadius", max(0, floor(localConfigValue( ...
           cfg, "curbCandidateCompanyAzimuthRadius", 16))), ...
        "minimumCompany", max(0, floor(localConfigValue( ...
           cfg, "curbCandidateMinimumCompany", 4))), ...
        "minimumRadialStep", max(0.0, localConfigValue( ...
           cfg, "curbCandidateMinimumRadialStep", 0.020)), ...
        "crossRingMinimumSlopeChange", max(0.0, localConfigValue( ...
           cfg, "curbCandidateCrossRingMinimumSlopeChange", 0.020)), ...
        "crossRingSearchDepth", max(1, floor(localConfigValue( ...
           cfg, "curbCandidateCrossRingSearchDepth", 3))), ...
        "crossRingMinimumGap", max(eps, localConfigValue( ...
           cfg, "curbCandidateCrossRingMinimumGap", 1.0e-3)), ...
        "metricCompanyRadius", max(eps, localConfigValue( ...
           cfg, "curbCandidateMetricCompanyRadius", 1.50)), ...
        "minimumMetricCompany", max(0, floor(localConfigValue( ...
           cfg, "curbCandidateMinimumMetricCompany", 2))), ...
        "minimumScore", localConfigValue( ...
           cfg, "curbCandidateMinimumScore", -0.80), ...
        "minimumCompanyToJudgeShape", 4);
end

function fusionConfig = localFusionConfig(cfg, parameters)
%LOCALFUSIONCONFIG Forward detector fusion options, then set the threshold.
    fusionConfig = struct("enabled", true);
    fieldMappings = [ ...
        "curbRingFeatureMinimumFamilyVotes", "minimumFamilyVotes"; ...
        "curbRingFeatureMinimumRadialMemberVotes", ...
           "minimumRadialMemberVotes"; ...
        "curbRingFeatureSaliencyEnabled", "saliencyEnabled"; ...
        "curbRingFeatureLocalHalfWidthBins", "localHalfWidthBins"; ...
        "curbRingFeatureLocalExclusionBins", "localExclusionBins"; ...
        "curbRingFeatureMinimumLocalNeighborCount", ...
           "minimumLocalNeighborCount"; ...
        "curbRingFeatureLocalPercentileThreshold", ...
           "localPercentileThreshold"; ...
        "curbRingFeatureMinimumSalientFamilyVotes", ...
           "minimumSalientFamilyVotes"; ...
        "curbRingFeatureContinuityRingRadius", "continuityRingRadius"; ...
        "curbRingFeatureContinuityAzimuthRadius", ...
           "continuityAzimuthRadius"; ...
        "curbRingFeatureContinuityMinimumNeighbors", ...
           "continuityMinimumNeighbors"];
    for mappingIndex = 1:size(fieldMappings, 1)
        detectorField = fieldMappings(mappingIndex, 1);
        if isfield(cfg, detectorField)
           fusionConfig.(fieldMappings(mappingIndex, 2)) = ...
               cfg.(detectorField);
        end
    end

    thresholds = struct();
    if isfield(cfg, "curbRingFeatureThresholds") ...
           && isstruct(cfg.curbRingFeatureThresholds)
        thresholds = cfg.curbRingFeatureThresholds;
    end
    % The fusion default is calibrated to leak no development hard negative,
    % which costs curb returns wherever the compression response is weak.
    if ~isfield(thresholds, "hataCompressionNormalized")
        thresholds.hataCompressionNormalized = ...
           parameters.compressionThreshold;
    end
    fusionConfig.thresholds = thresholds;
end

function keptMask = localKeepAccompanied(candidateMask, parameters)
%LOCALKEEPACCOMPANIED Drop candidates that stand alone in the sweep grid.
%   Columns wrap through 360 degrees; rows do not.
    if parameters.minimumCompany <= 0
        keptMask = candidateMask;
        return;
    end
    numericMask = double(candidateMask);
    [ringCount, azimuthCount] = size(numericMask);
    ringRadius = min(parameters.companyRingRadius, ringCount - 1);
    azimuthRadius = min( ...
        parameters.companyAzimuthRadius, azimuthCount - 1);
    neighborCount = zeros(size(numericMask));
    for azimuthOffset = -azimuthRadius:azimuthRadius
        shifted = circshift(numericMask, [0, azimuthOffset]);
        neighborCount = neighborCount ...
           + movsum(shifted, [ringRadius, ringRadius], 1);
    end
    neighborCount = neighborCount - numericMask;
    keptMask = candidateMask ...
        & neighborCount >= parameters.minimumCompany;
end

function keptMask = localKeepMetricallyAccompanied( ...
        candidateMask, frame, parameters)
%LOCALKEEPMETRICALLYACCOMPANIED Drop candidates that stand alone in space.
%   The company test above counts neighbours in sweep rows and columns,
%   and at range those index steps are metres wide: one ring apart is
%   1.7 m of ground at 19 m, so a cell can satisfy it while nothing lies
%   anywhere near it. Frame 468 shows the consequence. Three cells the
%   user identified as obvious outliers are candidates with zero other
%   candidates within a metre of them, while the cells lying on the fitted
%   curve have a median of 55 neighbours in the same radius.
%
%   Counting in metres instead separates the two populations at an area
%   under the curve of 0.923 over the labelled curb points and the reported
%   road-surface points, with nine curb points in ten having at least two
%   neighbours within 1.5 m and the median road-surface point having none.
    if parameters.minimumMetricCompany <= 0
        keptMask = candidateMask;
        return;
    end
    candidateIndices = find(candidateMask);
    if numel(candidateIndices) <= parameters.minimumMetricCompany
        keptMask = false(size(candidateMask));
        return;
    end
    pointX = double(frame.x(candidateIndices));
    pointY = double(frame.y(candidateIndices));
    pointZ = double(frame.z(candidateIndices));
    squaredRadius = parameters.metricCompanyRadius^2;

    % Counted in chunks so the pairwise comparison never allocates more
    % than a bounded block, whatever the candidate count.
    pointCount = numel(candidateIndices);
    neighbourCount = zeros(pointCount, 1);
    chunkSize = 512;
    for chunkStart = 1:chunkSize:pointCount
        chunkStop = min(chunkStart + chunkSize - 1, pointCount);
        chunk = chunkStart:chunkStop;
        squaredDistance = ...
           (pointX(chunk) - pointX.').^2 ...
           + (pointY(chunk) - pointY.').^2 ...
           + (pointZ(chunk) - pointZ.').^2;
        neighbourCount(chunk) = ...
           sum(squaredDistance < squaredRadius, 2) - 1;
    end

    keptMask = false(size(candidateMask));
    keptMask(candidateIndices( ...
        neighbourCount >= parameters.minimumMetricCompany)) = true;
end

function acceptedMask = localCrossRingStepMask(frame, parameters)
%LOCALCROSSRINGSTEPMASK Keep cells where the ground profile bends.
%   A curb is a step in the ground, and a step is visible across the rings
%   rather than along one. This measures, at a fixed azimuth, the change in
%   height gradient between the ring inside a cell and the ring outside it.
%   A flat surface contributes nothing to that change and neither does a
%   constant grade, so the road's own camber and the rise of distant
%   pavement, which defeat every test against a fitted road level, cancel
%   here by construction.
%
%   This is what the feature set was missing. Its own height step is taken
%   along a ring, between neighbouring azimuths, where a curb is followed
%   lengthwise and barely changes height: measured over the labelled curb
%   points and the road-surface points the user reported it separates them
%   at an area under the curve of 0.61. Every other cross-ring feature
%   present measures range or spacing rather than height. The gradient
%   change measured here separates the same two populations at 0.957, with
%   a median of 0.080 on curb against 0.0037 on road surface, and it needs
%   no fitting to the dataset.
%
%   The neighbour on each side is the nearest valid ring within a short
%   search rather than strictly the adjacent one, which raises the fraction
%   of cells where the measure exists from 28 to 80 per cent.
    z = double(frame.z);
    radial = hypot(double(frame.x), double(frame.y));
    valid = isfinite(z) & isfinite(radial);
    if isfield(frame, "valid")
        valid = valid & logical(frame.valid);
    end
    [ringCount, azimuthCount] = size(z);

    upperZ = nan(ringCount, azimuthCount);
    upperRadial = nan(ringCount, azimuthCount);
    lowerZ = nan(ringCount, azimuthCount);
    lowerRadial = nan(ringCount, azimuthCount);
    % A search may not reach past the sweep it is measured on.
    searchDepth = min(parameters.crossRingSearchDepth, ringCount - 1);
    for offset = 1:searchDepth
        shiftedZ = [nan(offset, azimuthCount); z(1:end - offset, :)];
        shiftedRadial = [nan(offset, azimuthCount); ...
           radial(1:end - offset, :)];
        shiftedValid = [false(offset, azimuthCount); ...
           valid(1:end - offset, :)];
        fill = isnan(upperZ) & shiftedValid;
        upperZ(fill) = shiftedZ(fill);
        upperRadial(fill) = shiftedRadial(fill);

        shiftedZ = [z(offset + 1:end, :); nan(offset, azimuthCount)];
        shiftedRadial = [radial(offset + 1:end, :); ...
           nan(offset, azimuthCount)];
        shiftedValid = [valid(offset + 1:end, :); ...
           false(offset, azimuthCount)];
        fill = isnan(lowerZ) & shiftedValid;
        lowerZ(fill) = shiftedZ(fill);
        lowerRadial(fill) = shiftedRadial(fill);
    end

    upperGap = radial - upperRadial;
    lowerGap = lowerRadial - radial;
    resolvable = valid & isfinite(upperZ) & isfinite(lowerZ) ...
        & abs(upperGap) >= parameters.crossRingMinimumGap ...
        & abs(lowerGap) >= parameters.crossRingMinimumGap;
    slopeChange = nan(ringCount, azimuthCount);
    slopeChange(resolvable) = abs( ...
        (lowerZ(resolvable) - z(resolvable)) ./ lowerGap(resolvable) ...
        - (z(resolvable) - upperZ(resolvable)) ./ upperGap(resolvable));

    % Where the profile cannot be resolved the cell is left to the other
    % gates rather than being rejected on missing evidence.
    acceptedMask = ~resolvable ...
        | slopeChange >= parameters.crossRingMinimumSlopeChange;
end

function acceptedMask = localCurbScore(features, parameters)
%LOCALCURBSCORE Linear curb-versus-pavement score over the ring features.
%   The cross-ring gate above answers the geometric question and this
%   answers what is left of it. No single ring feature separates a curb
%   from the pavement that fools this detector: over 335 user-labelled curb
%   points and 125 reported road-surface points the best reaches an area
%   under the curve of 0.76, while this combination reaches 0.87.
%
%   Coefficients are a regularised Fisher discriminant on standardised
%   features, with a missing feature replaced by its own training median so
%   an undefined feature is scored as unremarkable rather than excluding
%   the cell. They are fitted to one route in one town. Unlike the
%   cross-ring gate, which is a geometric statement that needs no fitting,
%   these numbers should be refitted if the dataset changes.
%
%   The reflectivity-derived intensityRange term was removed from the
%   feature set; the remaining coefficients are the original fit with that
%   term dropped and have not been refitted.
    featureNames = [ ...
        "heightStep"; ...
        "rangeStep"; ...
        "localHeightStd"; ...
        "localRangeStd"; ...
        "localHeightRange"; ...
        "localMeanElevation"; ...
        "chenSmoothness"; ...
        "leftSmoothArcLength"; ...
        "rightSmoothArcLength"; ...
        "sameRingSlopeMagnitude"; ...
        "sameRingSlopeChange"; ...
        "bilateralTangentDeflection"; ...
        "huangTangentialStrength"; ...
        "huangRadiusRatioDeviation"; ...
        "huangHdInRadius"; ...
        "hataDifferentialHeightResponse"; ...
        "radiusRatioReconstructionDeviation"; ...
        "wangHorizontalSpacingRatio"; ...
        "wangHorizontalSpacingExcess"; ...
        "radialSecondDifference"; ...
        "localChordResidual"; ...
        "hataCompressionRaw"; ...
        "hataCompressionExpected"; ...
        "hataCompressionNormalized"; ...
        "ilpLineFitResidual"; ...
        "ilpLineSlopeMagnitude"];
    centre = [ ...
          1.116157e-02,   8.460867e-02,   1.270024e-02,   1.024358e-01, ...
          3.075778e-02,  -2.079358e+00,   5.752164e-03,   4.103937e-01, ...
          4.068315e-01,   3.791450e-02,   1.975792e-03,   1.059360e-02, ...
          2.637978e-01,   5.464371e-03,   4.803819e-02,   3.255188e-02, ...
          1.391196e-04,   1.055006e+00,   4.166378e-02,   3.132807e-02, ...
          1.021284e-03,   8.700047e-01,   1.847760e+00,   4.594970e-01, ...
          1.561079e-03,   2.942028e-02
];
    scale = [ ...
          1.896013e-02,   1.489994e-01,   1.329139e-02,   1.146016e-01, ...
          3.128275e-02,   2.845057e-01,   5.494665e-03,   2.985896e-01, ...
          3.030528e-01,   4.692819e-02,   4.953573e-02,   3.687392e-01, ...
          2.177010e-01,   8.593622e-03,   2.387670e+00,   4.270350e-02, ...
          4.042214e-03,   4.133893e-01,   4.155435e-01,   1.505950e-01, ...
          5.950229e-02,   9.869538e-01,   1.257532e+00,   2.874541e-01, ...
          7.984542e-03,   2.742312e-02
];
    weights = [ ...
         -6.973100e-02,   1.536704e-01,  -4.857191e-02,   1.561639e-01, ...
         -9.535286e-02,  -3.422381e-01,  -1.437617e-01,  -2.757192e-01, ...
         -1.805760e-01,   1.060832e-01,   1.812347e-01,  -3.278133e-01, ...
          2.279967e-01,   1.031038e-02,   4.593243e-02,  -1.002752e-01, ...
          1.235950e-01,   1.054557e-01,  -4.259852e-02,  -1.469204e-01, ...
          5.252545e-02,   5.769943e-01,  -1.096201e+00,   9.160548e-01, ...
          3.293181e-01,   1.352658e-01
].';
    grid = features.(featureNames(1));
    score = zeros(size(grid));
    for featureIndex = 1:numel(featureNames)
        page = double(features.(featureNames(featureIndex)));
        page(~isfinite(page)) = centre(featureIndex);
        score = score + weights(featureIndex) ...
           * (page - centre(featureIndex)) / scale(featureIndex);
    end
    acceptedMask = score >= parameters.minimumScore;
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
