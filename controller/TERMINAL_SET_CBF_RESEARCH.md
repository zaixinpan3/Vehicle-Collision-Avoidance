# Terminal requirement: certified encounter discharge

Decision date: September 7, 2026. The governing formulation is
[ENCOUNTER_SCOPED_CBF_CLF.md](ENCOUNTER_SCOPED_CBF_CLF.md). Its invariant
object is the availability of a valid certified continuation while an encounter
is active. The endpoint is a verified event or handoff; it need not be a
permanently invariant physical ego–target set.

## Required endpoint

A complete continuation must reach one of the following:

1. A certified encounter-exit guard that discharges every assigned obligation.
2. A domain covered by another already certified continuation, with a verified
   transition and no interval without safety coverage.
3. A separately certified holding/handoff regime, including its validity and
   transfer conditions.

For a crossing, clearance of the entire uncertain target footprint from the
shared conflict region requires an applicable route or monitoring contract
before it can discharge that encounter. Current radar exit, target publication
loss, forecast expiry, instantaneous separation, and stopping do not establish
the guard. An unresolved encounter remains active.

The target-motion contract covers the finite portion during which its
obligation remains active. An endpoint before forecast expiry can close a
certificate if its guard is verified. A safe prefix ending at forecast expiry
without discharge or valid transfer cannot close it.

## Predictive barrier and tracking roles

The finite-to-exit construction carries a verified witness, nonnegative
barrier margin \(b_k\), and remaining interval count \(n_k\). It certifies
the entire held-input interval and every admitted successor continuation.
An accepted replacement has verified margin
\(\mu_k\ge(1-\gamma)b_k\), with \(0<\gamma\le1\). A valid observation conditions
the covered futures; the controller retains the tail with
\(b_{k+1}=\mu_k\) and \(n_{k+1}=n_k-1\).

No terminal target node is appended. Solver failure uses the inherited
certificate with \(\mu=b_k\). Keeping the deadline gives a finite-exit
property; the safety requirement itself permits certified holding or renewal.
Any deadline extension must be certified before replacing the incumbent.

An explicit predictive CLF separately bounds tracking-error dissipation over
every held interval, with nonnegative relaxation and complete reference/metric
derivatives. Maneuver, control, input smoothness, and switching choices enter
the optimization. Neither a tracking cost nor a selected separating normal
replaces these CLF and maneuver requirements.

## Existing rest construction and its limits

The version-8 implementation remains described in
[PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md). Its head and tail use
the same scheduled six-state bicycle, exact held-input affine integration,
signed braking-ratio/Fiala force scale, and physical input/slip-domain rows.
This removed the earlier dynamic-head/kinematic-tail mismatch. Exact affine
flow is still an approximation of the physical plant unless a valid residual
enclosure is supplied.

At zero scheduled speed, arbitrary admitted station, lateral offset, and
heading with zero velocities are an equilibrium under
`[0; -bias/brakingRatioAccelerationGain]`. Exact-rest terminal equalities
or the [dissipative extension](DISSIPATIVE_TERMINAL_CERTIFICATE.md) can certify
ego behavior under their specific model premises. The braking schedule is an
initial optimization template, not a safety proof.

The current target row concerns the finite last predicted pose. It does not
use infinite-future target support and does not certify encounter discharge.
Ego rest or an ego dissipative funnel does not make the joint endpoint safe
after that forecast. The runtime still checks prediction nodes and removes
target rows when publication ceases; it therefore does not yet meet the
revised contract.

Rest/dissipation remains an optional construction for a separately certified
holding regime. Its rejection of persistent forcing is not a universal rule
for finite encounter certificates. Bounded nonzero residuals can be admitted
when a sound finite tube and its discharge/transfer guard pass verification.
No uncertainty may be zeroed merely to make that verification succeed.

The revised specification lists the required interfaces, proof premises, and
behavioral acceptance cases. It does not claim that a concrete exit guard,
sampled-data physical safety theorem, or revised runtime has already been
implemented.
