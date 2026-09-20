# Changelog

This page summarizes the most important changes on the `main` branch of Holistic Fusion, newest first.
Smaller fixes are not listed here; see the [commit history](https://github.com/leggedrobotics/holistic_fusion/commits/main) for details.

## 2026

### September 2026: Switch to GTSAM 4.3.0

* **GTSAM 4.3.0 is now the required version.** `graph_msf` uses `find_package(GTSAM 4.3 REQUIRED)`; GTSAM 4.2 is no
  longer supported. The Docker images (`docker/submodules/gtsam.sh`) and the [installation instructions](1_installation.md)
  build GTSAM 4.3.0 (tests and examples disabled to speed up the build).
* **No more Boost in `graph_msf`.** All `boost::optional`, `boost::shared_ptr` and `boost::filesystem` usages were replaced by
  their C++17 standard-library counterparts, following GTSAM's own move away from Boost. Public signatures affected (only if you
  call them from your own code):
    * `UnaryMeasurementAbsolute` / `UnaryMeasurementXDAbsolute`: the optional alignment noise arguments are now
      `std::optional<Eigen::Matrix<double, 6, 1>>` (pass `std::nullopt` instead of `boost::none`).
    * `FileLogger::writePose3ToCsvFile` and `graph_msf::writePose3ToCsvFile`: the optional covariance argument is now a
      `std::optional<Eigen::Matrix<double, 6, 6>>`.
* **`gtsam_unstable` is no longer needed.** The fixed-lag smoothers used by `graph_msf` moved into the stable GTSAM library in
  4.3, so `graph_msf`, `graph_msf_catkin` and the exported CMake config only depend on `gtsam` now.
* **Custom factors use the new GTSAM factor API.** `YawFactor`, `PitchFactor` and `RollFactor` derive from
  `gtsam::NoiseModelFactorN<gtsam::Pose3>` and implement `evaluateError(const Pose3&, gtsam::OptionalMatrixType)`.
* **Behavioural notes (from GTSAM itself, no change in Holistic Fusion needed):** `BetweenFactor`/`PriorFactor` now use the
  Lie-group (local) Jacobians by default, the SO(3)/SE(3) exponential and logarithm maps have been reworked, and iSAM2 /
  `IncrementalFixedLagSmoother` received marginalization fixes. Expect slightly different (generally better) numerical results
  compared to GTSAM 4.2.

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
