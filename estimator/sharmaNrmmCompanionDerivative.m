function [derivative, reconstruction] = sharmaNrmmCompanionDerivative(state, input, variant, speedFloor)
% sharmaNrmmCompanionDerivative Evaluate the Sharma (2026) NRMM companion map.
%
% Reproduces the companion-form process model of Sharma, Alai and Rajamani,
% "Simultaneous ego-vehicle state estimation and vehicle trajectory tracking
% using a multistage high gain observer", Transp. Res. Part C 182 (2026),
% 105411, Section 3.3, equations (47)-(64). The state is
%
%   state = [rx; rxDot; rxDdot; ry; ryDot; ryDdot]          (paper eq. 47)
%
% and the ego inputs are input.bodyVelocity = [vx; vy], input.bodyAcceleration
% = [ax; ay] and input.yawRate = psiEdot (paper u_C2, eq. 63). The returned
% derivative is F*state + G*f_C2 with f_C2 = [f1; f2] from eqs. (62)-(63).
%
% variant selects how the third derivative is evaluated:
%   "published" evaluates eqs. (62)-(63) exactly as printed.
%   "corrected" adds the ego body-velocity second-derivative term that the
%               printed equations omit. Differentiating r'' = s - psiEdot*J*q
%               - vDot - psiEdot*J*rDot once more under the paper's own
%               Assumption 1 (aDot = 0, psiEddot = 0, so vDdot = -psiEdot*J*vDot,
%               eqs. 52-53) gives r''' = published + psiEdot*J*vDot, i.e.
%               f1 gains -psiEdot*vyDot and f2 gains +psiEdot*vxDot.
% Both variants keep Assumption 1; neither models ego jerk or ego angular
% acceleration.
%
% speedFloor guards the 1/V_C^2 division of eq. (59) when observer peaking
% drives the reconstructed target speed toward zero; the paper does not
% specify such a guard. reconstruction reports V_Cx, V_Cy, zeta, psiCdot,
% A_Cx, A_Cy and whether the guard was active.

    arguments
        state (6, 1) double {mustBeReal}
        input (1, 1) struct
        variant (1, 1) string {mustBeMember(variant, ["published", "corrected"])}
        speedFloor (1, 1) double {mustBeNonnegative}
    end

    vx = input.bodyVelocity(1);
    vy = input.bodyVelocity(2);
    ax = input.bodyAcceleration(1);
    ay = input.bodyAcceleration(2);
    psiEdot = input.yawRate;

    rx = state(1);
    rxDot = state(2);
    rxDdot = state(3);
    ry = state(4);
    ryDot = state(5);
    ryDdot = state(6);

    % Eqs. (50)-(51): body-frame derivative of the ego velocity components.
    vxDot = ax + vy*psiEdot;
    vyDot = ay - vx*psiEdot;
    % Eqs. (54)-(55): target absolute velocity in the ego frame.
    vCx = rxDot + vx - psiEdot*ry;
    vCy = ryDot + vy + psiEdot*rx;
    speedSquared = vCx^2 + vCy^2;
    guardActive = speedSquared < speedFloor^2;
    speedSquaredGuarded = max(speedSquared, speedFloor^2);
    % Eqs. (57)-(58).
    zetaX = rxDdot + vxDot - psiEdot*ryDot - vCy*psiEdot;
    zetaY = ryDdot + vyDot + psiEdot*rxDot + vCx*psiEdot;
    % Eq. (59).
    psiCdot = (zetaY*vCx - zetaX*vCy)/speedSquaredGuarded;
    % Eqs. (60)-(61).
    aCx = zetaX + vCy*psiCdot;
    aCy = zetaY - vCx*psiCdot;
    % Eqs. (62)-(63) as printed.
    relativeRate = psiCdot - psiEdot;
    f1 = -3.0*aCy*psiCdot + 2.0*psiEdot*aCy - vCx*relativeRate^2 + psiEdot*ryDdot;
    f2 = 3.0*aCx*psiCdot - 2.0*psiEdot*aCx - vCy*relativeRate^2 - psiEdot*rxDdot;
    if variant == "corrected"
        f1 = f1 - psiEdot*vyDot;
        f2 = f2 + psiEdot*vxDot;
    end

    derivative = [rxDot; rxDdot; f1; ryDot; ryDdot; f2];
    if nargout < 2
        return
    end
    reconstruction = struct( ...
        "targetVelocity", [vCx; vCy], ...
        "targetSpeed", sqrt(speedSquared), ...
        "zeta", [zetaX; zetaY], ...
        "targetYawRate", psiCdot, ...
        "targetAcceleration", [aCx; aCy], ...
        "egoBodyVelocityDerivative", [vxDot; vyDot], ...
        "speedGuardActive", guardActive);
end
