function [transition, measurementMap, stateFlow, inputFlow] = sampledFeedbackTransition(a,b,gain,duration)
%sampledFeedbackTransition Transition for feedback sampled once and held.
% u(t)=v+gain*(e(0)+nu), so e(t)=transition*e(0)+measurementMap*nu
% plus the integrated affine residual. This floating-point map is not an
% outward-rounded enclosure of nonlinear dynamics or arbitrary disturbances.
    arguments
        a (:,:) double {mustBeReal,mustBeFinite}
        b (:,:) double {mustBeReal,mustBeFinite}
        gain (:,:) double {mustBeReal,mustBeFinite}
        duration (1,1) double {mustBeNonnegative,mustBeFinite}
    end
    n=size(a,1);m=size(b,2);
    assert(isequal(size(a),[n,n]) && size(b,1)==n && isequal(size(gain),[m,n]), ...
        'collisionAvoidanceController:invalidFeedbackDimensions','Feedback dimensions must agree.');
    lifted=expm(duration*[a,b;zeros(m,n+m)]);
    stateFlow=lifted(1:n,1:n);inputFlow=lifted(1:n,n+(1:m));
    measurementMap=inputFlow*gain;
    transition=stateFlow+measurementMap;
end
