function audit = auditNrmmTruthEnclosure(output, egoState, targetTruth, cfg)
%auditNrmmTruthEnclosure Check simulation truth against published NRMM bounds.
% This offline diagnostic never changes measurements, estimates or controls.
% Ego order is [x;y;yaw;vx;vy;r]. Checks apply at the supplied timestamp;
% they do not establish intersample bounds, target jerk, or a global theorem.

    validateattributes(egoState,{'double'},{'real','finite','vector','numel',6});
    egoState = egoState(:);
    domain = cfg.ego.domain;
    model = cfg.ego.yaw;
    speed = norm(egoState(4:5));
    sideslip = abs(atan2(egoState(5),egoState(4)));
    mismatch = abs(egoState(6)-egoState(5)/model.rearAxleDistance);
    premiseSlack = [speed-domain.speedMinimum;domain.speedMaximum-speed; ...
        domain.yawRateMaximum-abs(egoState(6)); ...
        model.sideslipDomainMaximum-sideslip; ...
        model.singleTrackYawRateMismatchMaximum-mismatch];
    center = [output.egoPositionInertial;output.egoYaw; ...
        output.egoBodyVelocity;output.egoYawRate];
    error = center-egoState;
    error(3) = atan2(sin(error(3)),cos(error(3)));
    egoBound = output.controllerStateErrorBound(:);
    available = output.controllerErrorBound.available && all(isfinite(egoBound));
    egoSlack = egoBound-abs(error);
    % An unavailable infinite radius is not evidence of containment.
    if ~available,egoSlack(:) = NaN;end
    targetSlack = NaN(6,1);
    targetAvailable = isempty(output.targetEstimates);
    targetChecked = false(6,1);
    if ~isempty(output.targetEstimates)
        target = output.targetEstimates(1);
        targetAvailable = target.controllerErrorBound.available;
        fields = ["targetPositionInertial","targetVelocityInertial","targetAccelerationInertial"];
        for index = 1:3
            if isfield(targetTruth,fields(index))
                rows = 2*index-1:2*index;
                targetChecked(rows) = true;
                targetSlack(rows) = target.controllerErrorBound.bounds(rows) ...
                    -abs(target.(fields(index))(:)-targetTruth.(fields(index))(:));
            end
        end
        if ~targetAvailable,targetSlack(:) = NaN;end
    end
    tolerance = 1e-9;
    audit = struct("time",output.stateTime, ...
        "egoPremiseNames",["minimumSpeed";"maximumSpeed";"yawRate";"sideslip";"rearKinematicMismatch"], ...
        "egoPremiseSlack",premiseSlack,"egoPremisesSatisfied",all(premiseSlack>=-tolerance), ...
        "rearKinematicMismatch",mismatch,"egoError",error,"egoBoundSlack",egoSlack, ...
        "egoBoundAvailable",available,"egoContained",available && all(egoSlack>=-tolerance), ...
        "targetBoundSlack",targetSlack,"targetCheckedComponents",targetChecked, ...
        "targetBoundAvailable",targetAvailable, ...
        "checkedTargetComponentsContained",targetAvailable && all(targetSlack(targetChecked)>=-tolerance), ...
        "scope","sampled ego premises and available state components; no continuous-time or target-motion proof");
end
