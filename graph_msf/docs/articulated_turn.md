# Articulated turn state and runtime

The native factors in
[`ArticulatedTurnFactors.h`](../include/graph_msf/factors/non_expression/ArticulatedTurnFactors.h)
couple a cabin pose and a slew-joint angle inside one GTSAM graph. They are
used by the opt-in GraphManager runtime in
[`ArticulatedTurnGraph.h`](../include/graph_msf/core/ArticulatedTurnGraph.h).
MoleEstimator enables the integration with `articulated_turn.enable`; its
[operator contract](https://github.com/leggedrobotics/moleworks_ros/blob/main/mole_estimator/docs/articulated_turn.md)
covers input qualification, output health and the default-disabled rollout.

The existing cabin IMU, GNSS and localization factors remain the only use of
those measurements. The added states are a scalar joint angle `q(k)` at each
cabin pose key and a shared scalar BASE heading `h(s)` for each parked station.
For the existing cabin/BASE frame convention:

```text
R_world_base(k) = R_world_imu(k) * R_imu_cabin * Rz(-q(k))
```

The IMU-to-cabin extrinsic is fixed in this model. Use the full rotation:
subtracting Euler yaw angles is incorrect on tilted terrain.

## Factors and evidence ownership

| Factor | Variables | Evidence |
|---|---|---|
| Existing cabin IMU/GNSS factors | Cabin pose, velocity, IMU bias | Existing raw cabin measurements, each once |
| `ParkedBaseHeadingFactor` | Cabin pose, `q(k)`, `h(s)` | Validated constant BASE heading during a station |
| `BaseGravityFactor` | Cabin pose, `q(k)` | Independently calibrated up direction expressed in BASE |
| `BaseRotationBetweenFactor` | Two cabin poses and their `q` values | Independent BASE rotation increment from BASE/leg gyros |
| `BaseAhrsBetweenFactor` | Two cabin poses, their `q` values, BASE gyro bias | Independent raw BASE gyro preintegration with a shared bias state |
| `BaseVerticalGyroFactor` | Two cabin poses, their `q` values, raw sensor-frame leg bias | Independent gravity-projected leg gyro integral |
| `BaselineHeadingFactor` | Cabin pose | Raw GNSS antenna-baseline azimuth with full extrinsic geometry |
| Initial joint prior | Initial `q` | A measured joint reference, or an explicitly uncertain startup prior |

`h(s)` gets an initial numerical guess, not a second independent prior derived
from the same cabin/encoder data. Every parked factor references that same
variable. On level terrain, heading observations alone cannot remove the
common joint-zero/BASE-heading ambiguity.

Independent, calibrated BASE gravity can supply additional joint information
on a slope. The existing MoleEstimator leg-gravity anchors are initialized
using its cabin estimate and joint angle; those anchors therefore cannot be
treated as independent absolute references. Runtime integration must represent
their common calibration uncertainty, or use independently measured mounting
calibration. Repeated factors must not average this shared error away.

The rotation-between measurement maps BASE coordinates at `j` into BASE
coordinates at `i`. It must come from independent BASE angular sensing.
Reusing cabin-minus-BASE gyro as an independent graph measurement would
duplicate cabin IMU information. Current HAL gravity attitudes have arbitrary
yaw: their quaternion difference is not a valid full BASE rotation increment.
Raw leg gyros need time-aligned leg kinematics, moving-leg rejection and bias
handling before they supply this factor. A persistent shared gyro bias needs
an explicit bias state or a correctly correlated measurement model; increasing
independent factor noise does not model that correlation.

`BaseAhrsBetweenFactor` provides that bias-aware rotation likelihood through
GTSAM AHRS preintegration. The caller owns the bias prior, bias evolution and
sample/frame validation. A bias prior is uncertain evidence, not proof that
BASE gyro bias is observable while driving. On level terrain, unknown BASE
gyro bias can be exchanged for joint-angle drift; its uncertainty must remain
in the joint estimate.

## Driving contract

While driving, a fixed BASE-heading constraint would incorrectly turn chassis
rotation into apparent joint motion. Release the parked constraint on fresh
drive/steer/leg-motion evidence, independently of how slowly the chassis moves.
Unknown or stale motion evidence must not establish a parked station.
The existing MoleEstimator `base_yaw_lock` selects filter covariance and
permits low-rate motion during holdover; its active flag alone is not
evidence for this graph's constant-heading factor.

Track the same continuous joint state through parked, driving and parked
transitions. A new parked station gets a new shared heading variable, but not
a fresh encoder zero. During motion, use independent BASE gyro increments and
their bias uncertainty. If these measurements are unavailable, report degraded
joint observability; do not silently keep the old BASE heading or jump back to
the known-biased encoder.

The current MoleEstimator differential turn-rate output already contains the
cabin gyro. It is suitable for its existing separate rate filter, but feeding
it into these factors would count the cabin IMU twice. Its gravity-projected
scalar leg rate is also not a full three-dimensional BASE rotation increment,
nor an Euler-heading derivative when roll/pitch change. The runtime sensor
adapter must supply independently validated BASE-frame gyro measurements.

## Runtime integration

1. The runtime queues immutable, timestamped leg and motion evidence and creates `q(k)`, its
   factors and timestamps atomically alongside the corresponding cabin
   `X(k), V(k), B(k)` batch in GraphManager. Do not independently insert a joint
   state against a nearest pose key that may be marginalized concurrently.
2. It retains the active station heading by refreshing its timestamp at actual
   graph time. Stop refreshing it when the station ends. Use a new station ID
   after real movement; do not reset the continuous joint state. Missing motion
   evidence suspends the parked constraint and increases uncertainty.
3. Graph insertion and optimization share a lifecycle lock. The public snapshot
   changes only after a successful optimizer update. Optimizer failures roll
   back and latch a fault; staged keys cannot be reused. Graph clock rewinds
   also latch a fault. Restart is required for a new initialization epoch.
4. MoleEstimator bypasses the old encoder-pretransformed leg-gravity and
   stationary BASE velocity factors. The current runtime uses independent
   gravity-projected raw leg gyros, not uncalibrated full-attitude increments.
   Per-leg sensor-frame bias states persist through bias random-walk factors.
   Maximum interval/rate gates and an explicit commutator allowance account
   for the short-interval vertical rotation approximation.
5. It extracts `X(k)`, `q(k)` and their **joint** covariance from one successful
   optimized snapshot. Assemble covariance blocks by explicit keys; GTSAM may
   reorder joint-marginal keys. Extend the supported GraphMSF optimizer backends
   and logging to handle scalar joint/station states.
6. MoleEstimator propagates the optimized joint state to control time using synchronized
   gyros and a timestamp history. This is output prediction, not another
   independent graph factor. Publish angle, velocity, health and BASE pose from
   the same state. Keep branch continuity across `±pi`; factor periodicity alone
   does not select the desired unwrapped branch.

[`ArticulatedBaseState.h`](../include/graph_msf/interface/ArticulatedBaseState.h)
implements the pose and covariance transformation for step 5, including the
cabin-pose/joint-angle cross terms. Adding separate cabin and joint marginal
variances is generally wrong. Its output covariance uses explicitly documented
local GTSAM and world ROS coordinate conventions.

## Validation scope

`test_articulated_turn` uses native GTSAM optimization, marginals and fixed-lag
smoothing. It checks factor geometry/Jacobians, joint uncertainty, motion and
BASE covariance. These synthetic tests establish model behavior; they do not
establish live sensor quality, timing, estimator throughput or deployment
readiness.

A subsequent standalone native GTSAM experiment consumed raw cabin gyro,
accelerometer and GNSS baseline heading, and used LiDAR only for scoring.
On 123/127 eligible recorded swings its median/p90 relative error was
0.0704°/0.1589°, compared with 1.1150°/1.9389° for the existing filtered joint.
Four swings lacked a valid startup interval. The initial approximately 2°
absolute-reference uncertainty remained; this is not an absolute-zero test.
The complete native graph step had 0.389 ms median and 4.702 ms p99 wall time;
four of 128,332 updates exceeded 100 ms on the shared PC.

The reproducible experiment and complete results live in
[`mole_estimator/experiments/articulated_turn`](https://github.com/leggedrobotics/moleworks_ros/tree/main/mole_estimator/experiments/articulated_turn).
That standalone experiment is conditional on parked stations and is separate
from the runtime integration described above. Synthetic driving tests check the model
geometry, transitions and bias uncertainty; they do not establish field
driving accuracy. Available parked ICP pairs are not independent joint truth
while the chassis moves.

Full estimator replay also needs authoritative GNSS quality evidence: the
inspected field recordings contain status-0 NavSatFix messages but omit both
the PVT quality input and corrected RTK fix used by the live estimator.
Do not fabricate RTK status or weaken the estimator gate to make replay start.
