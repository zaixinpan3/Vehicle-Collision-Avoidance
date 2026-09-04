function correspondence = certifiedRotationCorrespondence( ...
        inertialVector, bodyVector, inertialErrorMaximum, ...
        bodyErrorMaximum, modelAngleMaximum)
% certifiedRotationCorrespondence Certify one planar rotation measurement.
%
% The ideal vectors satisfy aI = R(psi)*bE. Bounded perturbations of the
% measured inertial and body vectors give the pseudo-heading
%
%   h = angle(aIMeasured)-angle(bEMeasured)
%
% and the certified radius
%
%   r = delta(aIMeasured, epsilonI)
%       + delta(bEMeasured, epsilonE) + betaMaximum.
%
% The correspondence is informative only when both vector uncertainty
% balls exclude the origin and the resulting circular arc is proper
% (r < pi). Otherwise radius is Inf and no yaw direction is claimed.

    inertialVector = localPlanarVector(inertialVector, "inertialVector");
    bodyVector = localPlanarVector(bodyVector, "bodyVector");
    inertialErrorMaximum = localNonnegativeScalar( ...
        inertialErrorMaximum, "inertialErrorMaximum");
    bodyErrorMaximum = localNonnegativeScalar( ...
        bodyErrorMaximum, "bodyErrorMaximum");
    modelAngleMaximum = localNonnegativeScalar( ...
        modelAngleMaximum, "modelAngleMaximum");
    if modelAngleMaximum >= pi
        error("certifiedRotationCorrespondence:invalidModelAngle", ...
            "modelAngleMaximum must be smaller than pi radians.");
    end

    [inertialRadius, inertialInformative, inertialMagnitude] = ...
        localDirectionErrorRadius(inertialVector, inertialErrorMaximum);
    [bodyRadius, bodyInformative, bodyMagnitude] = ...
        localDirectionErrorRadius(bodyVector, bodyErrorMaximum);
    rotationCosine = dot(bodyVector, inertialVector);
    rotationSine = bodyVector(1)*inertialVector(2) ...
        - bodyVector(2)*inertialVector(1);
    heading = atan2(rotationSine, rotationCosine);
    rawRadius = inertialRadius+bodyRadius+modelAngleMaximum;
    informative = inertialInformative && bodyInformative ...
        && rawRadius < pi;
    if informative
        radius = rawRadius;
    else
        radius = Inf;
    end

    correspondence = struct( ...
        "heading", heading, ...
        "radius", radius, ...
        "informative", informative, ...
        "rawRadius", rawRadius, ...
        "inertialDirectionRadius", inertialRadius, ...
        "bodyDirectionRadius", bodyRadius, ...
        "modelAngleMaximum", modelAngleMaximum, ...
        "inertialMagnitude", inertialMagnitude, ...
        "bodyMagnitude", bodyMagnitude);
end

function [radius, informative, magnitude] = ...
        localDirectionErrorRadius(measuredVector, errorMaximum)
% localDirectionErrorRadius Exact tangent-cone direction-error radius.
%
% If z = x+d with ||d|| <= epsilon and ||z|| > epsilon, the true vector
% x lies inside the origin-centered tangent cone through the uncertainty
% ball around z, so the angle between z and x is at most
% asin(epsilon/||z||). When ||z|| <= epsilon, that ball contains the
% origin and the measurement certifies no direction.

    magnitude = norm(measuredVector);
    informative = magnitude > errorMaximum;
    if informative
        radius = asin(min(1.0, errorMaximum/magnitude));
    else
        radius = Inf;
    end
end

function vector = localPlanarVector(vector, name)
    vector = double(vector);
    if ~isequal(size(vector), [2, 1]) || any(~isfinite(vector))
        error("certifiedRotationCorrespondence:invalidVector", ...
            "%s must be a finite two-vector.", name);
    end
end

function value = localNonnegativeScalar(value, name)
    value = double(value);
    if ~isscalar(value) || ~isfinite(value) || value < 0.0
        error("certifiedRotationCorrespondence:invalidBound", ...
            "%s must be a finite nonnegative scalar.", name);
    end
end
