function estimators = prepareObserverComparisonEstimators(names, cfg)
% prepareObserverComparisonEstimators Build the paired estimator table for one configuration.
%
% names is a string array containing "structured" (this work) and Sharma
% variants named sharma[Corrected](Certified|Matched)(Hold|Predictor):
%   Corrected  restores the ego-velocity term omitted from eqs. (62)-(63);
%   Certified  theta above the Theorem 1 threshold, Matched scales theta so the
%              slowest Sharma pole equals this work's slowest physical pole;
%   Hold       zero-order-held GPS/radar samples, Predictor the output-predictor
%              sampled realization the comparator uses.
% cfg is the nrmmTrackingConfig-style configuration of the scenario. Every entry
% carries the runtime function handle, the synthesized design and a description.
% The comparator design is synthesized once and shared by the matched variants.

    arguments
        names (1, :) string
        cfg (1, 1) struct
    end

    structuredDesign = synthesizeNrmmObserverGains(cfg);
    estimators = struct([]);
    for name = names
        switch name
            case "structured"
                estimator = struct("name", name, "runtime", @onlineNrmmTrackingRuntime, ...
                    "design", structuredDesign, ...
                    "description", "cascaded measured-input NRMM observer (this work)");
            otherwise
                estimator = localSharmaEstimator(name, cfg, structuredDesign);
        end
        if isempty(estimators)
            estimators = estimator;
        else
            estimators(end+1) = estimator; %#ok<AGROW>
        end
    end
end

function estimator = localSharmaEstimator(name, cfg, structuredDesign)
% localSharmaEstimator Decode sharma<Corrected><Certified|Matched><Hold|Predictor>.
    label = string(name);
    if ~startsWith(label, "sharma")
        error("prepareObserverComparisonEstimators:unknownEstimator", ...
            "Unknown estimator %s.", label);
    end
    remainder = extractAfter(label, "sharma");
    variant = "published";
    variantText = "as published";
    if startsWith(remainder, "Corrected")
        variant = "corrected";
        variantText = "with the omitted ego-velocity term restored";
        remainder = extractAfter(remainder, "Corrected");
    end
    if startsWith(remainder, "Certified")
        theta = "certified";
        thetaText = "theta above the Theorem 1 threshold";
        remainder = extractAfter(remainder, "Certified");
    elseif startsWith(remainder, "Matched")
        theta = "matched";
        thetaText = "theta matched to this work's slowest pole";
        remainder = extractAfter(remainder, "Matched");
    else
        error("prepareObserverComparisonEstimators:unknownEstimator", ...
            "Unknown estimator %s.", label);
    end
    switch remainder
        case "Hold"
            realization = "zeroOrderHold";
            realizationText = "zero-order-held samples";
        case "Predictor"
            realization = "outputPredictor";
            realizationText = "output-predictor samples";
        otherwise
            error("prepareObserverComparisonEstimators:unknownEstimator", ...
                "Unknown estimator %s.", label);
    end
    design = sharmaMultistageObserverDesign(cfg, Variant=variant, Theta=theta, ...
        ReferenceDesign=structuredDesign, SampledRealization=realization);
    estimator = struct("name", label, "runtime", @sharmaMultistageObserverRuntime, ...
        "design", design, "description", sprintf("Sharma 2026 %s, %s, %s", ...
            variantText, thetaText, realizationText));
end
