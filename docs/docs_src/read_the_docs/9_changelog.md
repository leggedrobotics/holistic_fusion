# Changelog

This page summarizes the most important changes on the `main` branch of Holistic Fusion, newest first.
Smaller fixes are not listed here; see the [commit history](https://github.com/leggedrobotics/holistic_fusion/commits/main) for details.

## 2026

### September 2026: Switch to GTSAM 4.3.0 (PR #46)

* **GTSAM 4.3.0 is now the required version.** `graph_msf` uses `find_package(GTSAM 4.3 REQUIRED)`; GTSAM 4.2 is no
  longer supported. The Docker images (`docker/submodules/gtsam.sh`) and the [installation instructions](1_installation.md)
  build GTSAM 4.3.0 (tests and examples disabled to speed up the build). GTSAM 4.3 needs CMake >= 3.20, Eigen >= 3.4, a
  compiler newer than gcc 9 and (if enabled) oneTBB; the Noetic (Ubuntu 20.04) Docker image therefore installs a newer CMake
  (pip), Eigen 3.4.0 (into `/usr/local`) and gcc-10 (now the default compiler of that image) and builds GTSAM without TBB.
  Ubuntu 22.04 (Humble) already ships suitable versions.
* **No more Boost in `graph_msf`.** All `boost::optional`, `boost::shared_ptr` and `boost::filesystem` usages were replaced by
  their C++17 standard-library counterparts, following GTSAM's own move away from Boost. Public signatures affected (only if you
  call them from your own code):
    * `UnaryMeasurementAbsolute` / `UnaryMeasurementXDAbsolute`: the optional alignment noise arguments are now
      `std::optional<Eigen::Matrix<double, 6, 1>>` (pass `std::nullopt` instead of `boost::none`).
    * `FileLogger::writePose3ToCsvFile` and `graph_msf::writePose3ToCsvFile`: the optional covariance argument is now a
      `std::optional<Eigen::Matrix<double, 6, 6>>`.
* **`gtsam_unstable` is no longer needed.** The fixed-lag smoothers used by `graph_msf` moved into the stable GTSAM library in
  4.3, so `graph_msf`, `graph_msf_catkin` and the exported CMake config only depend on `gtsam` now.
* **All ROS 1 packages compile as C++17.** GTSAM 4.3 headers require C++17; `pure_imu_integration` and
  `excavator_holistic_graph` were still built as C++14 and now use C++17 like the other packages.
* **Custom factors use the new GTSAM factor API.** `YawFactor`, `PitchFactor` and `RollFactor` derive from
  `gtsam::NoiseModelFactorN<gtsam::Pose3>` and implement `evaluateError(const Pose3&, gtsam::OptionalMatrixType)`.
* **Config change (breaking): Earth-rotation / Coriolis parameters.** The IMU noise parameters `use2ndOrderCoriolis` and
  `omegaCoriolis` were removed and replaced by
    * `earthRotationCompensation` (bool): use GTSAM's exact rotating-frame IMU model,
    * `latitudeDeg` (double): geodetic latitude, from which the Earth-rate vector is computed (Ω = 7.2921159e-5 rad/s),
    * `worldFrameNorthAligned` (bool): set to `true` only if the y-axis of the world frame points north (ENU); then the
      horizontal Earth-rate component Ω·cos(lat) is used in addition to the vertical one Ω·sin(lat).

  Reasons: with GTSAM 4.3, `use2ndOrderCoriolis` has no effect anymore (the exact model is used whenever a rotation rate
  is set), and the previous example value `omegaCoriolis: 1.07e-04` was the meteorological Coriolis parameter 2·Ω·sin(lat),
  i.e. twice the vertical Earth-rate component that GTSAM expects (5.4e-05 rad/s for Zurich). Since GTSAM 4.3 also
  compensates the Earth rate in the attitude integration, the old value would have acted like a 0.003 deg/s yaw-rate bias.
  Old config files fail to load until the two parameters are replaced.
* **Behavioural notes (from GTSAM itself, no change in Holistic Fusion needed):**
    * `BetweenFactor`/`PriorFactor` now use the Lie-group (local) Jacobians by default, the SO(3)/SE(3) exponential and
      logarithm maps have been reworked, and iSAM2 / `IncrementalFixedLagSmoother` received marginalization fixes. Expect
      slightly different (generally better) numerical results compared to GTSAM 4.2.
    * IMU preintegration: whenever a navigation-frame rotation rate is set (see the config change above), GTSAM 4.3 uses
      the exact rotating-frame dynamics (Coriolis and centrifugal accelerations and Earth-rate compensation of the attitude).
    * Noise models are validated more strictly (e.g. negative sigmas throw).

### September 2026: T-RO publication and new project page

* Paper published in IEEE Transactions on Robotics (T-RO), new project page and updated links (PR #44, #45).
* Bugfix: the optimization thread is now properly terminated on destruction (PR #43).

### July 2026: Local velocity factors for moving sensor frames

* The local velocity unary factor supports moving (non-rigid) sensor frames (PR #39) and makes the gyroscope bias observable
  through the lever-arm term (PR #38).

## 2025

### Summer 2025: ROS 2 support and build-system separation

* `graph_msf` became a pure CMake library, independent of the build system. Thin wrapper packages (`graph_msf_catkin` for
  ROS 1, ament/colcon support for ROS 2) expose it to the respective workspaces.
* New ROS 2 (Humble) integration layer `graph_msf_ros2` and the `smb_estimator_graph_ros2` example (PR #12 and follow-ups).
* Docker images and CI for both ROS 1 Noetic and ROS 2 Humble.
