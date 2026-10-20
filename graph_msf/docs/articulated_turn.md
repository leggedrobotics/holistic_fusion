# Articulated turn state: experimental GTSAM model

The native factors in
[`ArticulatedTurnFactors.h`](../include/graph_msf/factors/non_expression/ArticulatedTurnFactors.h)
couple a cabin pose and a slew-joint angle inside one GTSAM graph. They are
model-level building blocks with solver tests. **GraphManager and MoleEstimator
do not yet create these states or publish their results.** No ROS parameter
enables this model.

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

## Required runtime integration

1. Queue immutable, timestamped leg and motion evidence. Create `q(k)`, its
   factors and timestamps atomically alongside the corresponding cabin
   `X(k), V(k), B(k)` batch in GraphManager. Do not independently insert a joint
   state against a nearest pose key that may be marginalized concurrently.
2. Retain the active station heading by refreshing its timestamp at actual
   graph time. Stop refreshing it when the station ends. Use a new station ID
   after real movement; do not reset the continuous joint state. Missing motion
   evidence suspends the parked constraint and increases uncertainty.
3. Commit lifecycle bookkeeping with successful optimizer updates. A rejected
   or rolled-back batch must not leave orphaned states or an invalid anchor.
   Clock rewinds and graph resets require a new initialization epoch.
4. Replace the existing encoder-pretransformed leg-gravity factor with the
   coupled factor after its calibration uncertainty is modeled. Keeping both
   would count the same leg measurement twice.
5. Extract `X(k)`, `q(k)` and their **joint** covariance from one successful
   optimized snapshot. Assemble covariance blocks by explicit keys; GTSAM may
   reorder joint-marginal keys. Extend the supported GraphMSF optimizer backends
   and logging to handle scalar joint/station states.
6. Propagate the optimized joint state to control time using synchronized
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

The earlier recorded-data observer evaluation was a separate conditional
feasibility experiment using saved cabin estimates. It was not a replay of this
coupled GTSAM model. Full estimator replay also needs authoritative GNSS quality
evidence: the inspected field recordings contain status-0 NavSatFix messages but
omit both the PVT quality input and corrected RTK fix used by the live estimator.
Do not fabricate RTK status or weaken the estimator gate to make replay start.
