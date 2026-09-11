# Longitudinal force balance and road load

The current input is Fahmy's signed braking ratio beta. Its static-load
force scale is `gBeta=sum(mu_i*Fzi)/m`, so gross acceleration is
`gBeta*beta`; the old independent effectiveness gain is removed.
All reported accelerations below are physical quantities in m/s^2, not the
second decision coordinate. See [LTV_BICYCLE_MODEL.md](LTV_BICYCLE_MODEL.md).

On a flat road in still air the reduced force balance is

\[
m(\dot v_x-v_y r)=m g_\beta\beta-F_{\mathrm{aero}}(v_x)
-F_{\mathrm{roll}}(v_x)+mb.
\]

Here `beta` is the dimensionless second optimization input and `b` is an
independently supplied residual acceleration bias. Aerodynamic and rolling
forces are explicit physical terms. The command publishes the axle force
requests `beta*mu_i*Fzi`; the PassVeh14DOF adapter converts them to wheel
drive torque or brake pressure. `totalLongitudinalActuatorForce` denotes
the gross equivalent force. `totalLongitudinalTireForce` and
`axleLongitudinalTireForce` denote reduced contact forces after allocating
rolling loss by static axle load. `axleLongitudinalForce` is the pre-loss
adapter field. Subtracting aerodynamic force from total contact force gives
the same net force as subtracting both road loads from actuator force.
`command.longitudinalAcceleration = gBeta*beta` remains a derived diagnostic.

`ltvBicycleModel.roadLoad` implements

\[
F_{\mathrm{aero}}=\tfrac12\rho_{\mathrm{air}} C_d A_f v_x|v_x|,\qquad
F_{\mathrm{roll}}=mg(c_0+c_1|v_x|+c_4|v_x|^4)
\tanh(v_x/v_{\mathrm{tr}}).
\]

The rolling coefficients have units 1, s/m, and (s/m)^4. Their defaults are
0.01, 0, and 0; default transition speed is 0.5 m/s. Resistance opposes
motion, vanishes at rest, and cannot spontaneously accelerate a stopped
vehicle backwards. The transition is a reduced low-speed model, not a
static-friction or wheel-lock certificate. Generic air density, drag
coefficient, and frontal area defaults are 1.225 kg/m^3, 0.30, and 2.2 m^2.
All road-load coefficients must be finite nonnegative scalars, and the
transition speed must be positive.

At each scheduled speed, `ltvBicycleModel` uses the affine expansion

\[
F_{\mathrm{road}}(v_x)\simeq F_{\mathrm{road}}(\bar v)
+F'_{\mathrm{road}}(\bar v)(v_x-\bar v).
\]

Both the speed damping and affine intercept enter the exact held-input
matrix exponential. The continuous CLF reads the same generator. The
Riccati cache includes road-load parameters, avoiding reuse of a certificate
for a different speed-damping model. Initial schedule inputs include the
force required to counter road load. This is a feasible-plan initialization
quantity, not an acceleration target in the objective. Terminal rest still
uses `[0; -b/gBeta]`, since road load is zero there.

The modified Fiala model uses static axle loads and the scheduled applied
actuator demand. Separate front/rear friction polygons, load-transfer limits,
and utilization checks are removed. Aerodynamic pitch, wheel moments and
dynamic normal loads remain outside the reduced model. The output reports instantaneous nonlinear road load
at measured speed; it agrees with the affine derivative at the scheduling
point. A carried plan can move away from that point, so nonlinear and
scheduled derivatives must not be conflated.

## Plant parameter mapping

The scenario driver reads body pressure, air temperature, drag coefficient
and frontal area from the installed PassVeh14DOF template. Air density is
`Pabs/(287.058*Tair)`; the installed values give 1.2930 kg/m^3,
`Cd = 0.30`, and frontal area 2.11 m^2. For the wheel, mask fields can retain
inactive parameters while the built-in tire preset overrides them. The
adapter therefore reads the initialized `vdynMF` vector after the zero-time
plant initialization. It validates the MATLAB release (R2026a), tire type,
vector length, model identifier, and active-preset flag before using the
verified layout. An unsupported release or preset fails explicitly.

The active rolling coefficients are `[0.01, 0, 0.0004, 0.00004, 0, 0,
0.85, -0.4]`. The inactive mask contains a different vector, starting at
0.0069908; using it underestimated rolling resistance in a development
trial. The active reference load is 4300 N, reference pressure 262000 Pa,
reference speed 16.7 m/s, and unloaded radius 0.33454 m. The actual pressure
source is 220000 Pa. The result preserves the inactive coefficients and
active-parameter provenance. The equivalent rolling polynomial uses each
wheel's static normal load and initialized effective rolling radius:

\[
S=\sum_i\frac{R_0}{R_{e,i}}F_{z0}
\left(\frac{F_{z,i}}{F_{z0}}\right)^{q_{sy7}}
\left(\frac{p}{p_0}\right)^{q_{sy8}},\quad
(c_0,c_1,c_4)=\frac{S}{mg}
\left(q_{sy1},\frac{q_{sy3}}{V_0},\frac{q_{sy4}}{V_0^4}\right).
\]

This is the zero-camber, static-load reduction with unit rolling scale.
The installed template has `QSY2 = QSY5 = QSY6 = 0`; the adapter explicitly
rejects nonzero values rather than silently discarding longitudinal-force
or camber dependence. Raw wheel vertical force (`wheelNormalLoad`, the
existing trace field name), rolling moment, longitudinal force, body drag,
gravity, net body force, and pitch are retained in `result.plantTrace`.
The raw wheel vertical-force output must not be equated automatically with
the ground-normal force inside the rolling-moment law: wheel weight and
vertical dynamics can contribute to that difference.

The body already models aerodynamic load and the wheel already models
rolling resistance; those plant forces are not added again in Simulink.
The correction brings their reduced representation into the controller.
The body model is described in the [MathWorks Vehicle Body 6DOF documentation](https://www.mathworks.com/help/vdynblks/ref/vehiclebody6dof.html).
The wheel's dynamic rolling moment and axle torque balance are described
in the [Combined Slip Wheel 2DOF documentation](https://www.mathworks.com/help/vdynblks/ref/combinedslipwheel2dof.html).
The pressure/speed rolling polynomial and smooth direction convention are
also documented in [MathWorks Rolling Resistance](https://www.mathworks.com/help/sdl/ref/rollingresistance.html).

Wheel inertia, longitudinal slip, time-varying rolling radius/load, axle
viscous loss, aerodynamic lift/pitch, wind and grade are not reproduced by
this reduced model. No nonzero validated plant-residual enclosure has been
installed. Safety claims remain restricted to the declared affine model's
prediction nodes and the explicitly supported uncertainty class.

## Why zero speed error is still a separate question

The joint objective remains `h*sum(u'*Ru*u) + rho*delta^2`, with no linear
term, desired acceleration, LQR tracking input, or lexicographic solve.
At straight cruise, force balance requires
`betaEquilibrium = (Froad(vReference)/m-b)/gBeta`. This value is generally
nonzero. A raw input penalty prefers smaller input, while a quadratic CLF
has zero gradient at zero error. Soft derivative constraints alone therefore
do not make the desired physical cruise state invariant. Adding correct
resistance fixes missing physics; it does not prove asymptotic tracking for
this objective. A future change in tracking priorities or input coordinates
must be assessed separately rather than hidden in this force correction.

## Diagnostic evidence before the correction

The previous two 30-second trajectories were replayed exactly: all 1200
commands matched the saved commands. During the straight 25–30 s window,
mean speed error was -0.177398 m/s, predicted acceleration was 0.178496
m/s^2, and actual finite-difference acceleration was -0.000000535 m/s^2.
Mean required CLF decay magnitude was 0.0396673 V/s, while mean relaxation
was only 0.00008618 V/s (0.217%). The model predicted decreasing V at every
late sample although the physical speed remained nearly constant.

Five fixed-state samples per scene were also solved with slack first and
input second. Zero slack was feasible in all ten probes; for the first
straight probe it changed the command from 0.223011 to 0.223602 m/s^2.
These were optimization probes on saved states, not new physical closed-loop
trajectories. They show that slack priority was not the main source of the
observed model/plant acceleration gap.
