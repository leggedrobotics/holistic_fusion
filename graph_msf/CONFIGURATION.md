# GraphMSF configuration

`GraphConfig` is the core configuration contract. ROS adapters expose YAML names without the trailing underscore and usually without the `Flag` suffix. A field can be supported without affecting every optimizer or measurement type.

## IMU covariance

The accelerometer and gyroscope covariance inputs are continuous-time noise densities squared:

```text
accelerometerCovariance = I * accNoiseDensity²
gyroscopeCovariance = I * gyroNoiseDensity²
```

`biasAccStdDevForIntegration` and `biasOmegaStdDevForIntegration` are not supported parameters. GTSAM 4.3 does not apply their former setter, `setBiasAccOmegaInit`. Initial bias uncertainty belongs in the prior on the first bias state. See [GTSAM PR #2199, Remove initial cov from CombinedImuFactor](https://github.com/borglab/gtsam/pull/2199) for the upstream rationale.

To preserve the effective covariance of an existing configuration, migrate its YAML values once:

```text
accNoiseDensity_new = sqrt(accNoiseDensity_old² + biasAccStdDevForIntegration_old²)
gyrNoiseDensity_new = sqrt(gyrNoiseDensity_old² + biasOmegaStdDevForIntegration_old²)
```

Then delete both legacy keys. ROS YAML uses `gyrNoiseDensity` for the C++ field `gyroNoiseDensity_`. Keep enough decimal digits to preserve the squared values. The runtime uses only these final noise densities.

The supplied configurations include this migration. Their values describe effective measurement uncertainty and retain the estimator weighting. They are not independent measurements of sensor noise. Tune them against data when adopting a calibrated physical noise model.

This conversion preserves the legacy independent diagonal covariance contribution with tangent preintegration and measurements expressed at the IMU origin. It does not migrate accelerometer–gyroscope cross-covariance terms or imply identical estimates across GTSAM versions.

`accBiasRandomWalkNoiseDensity` and `gyroBiasRandomWalkNoiseDensity` describe bias evolution. `initialAccBiasStdDev` and `initialGyroBiasStdDev` set the uncertainty of the first bias state. These settings have separate effects.

The supplied GTSAM build disables `GTSAM_ALLOW_DEPRECATED_SINCE_V43`. Calls to the ignored compatibility APIs therefore fail to compile.

## Initialization

`initialization_params.static_at_startup` maps to `GraphConfig::staticAtStartup_`.
All supplied configurations set it to `true` and require a stationary startup window.
If it is `false`, setup selects a zero gyro-bias mean and throws a `std::logic_error` stating that in-motion initialization is not implemented.
No IMU samples are averaged and no estimator worker thread starts in that mode.
A zero bias mean is not a calibrated bias estimate and does not set its uncertainty to zero.
`initialGyroBiasStdDev` controls the prior uncertainty.

GraphMsf uses a stationary IMU window to initialize attitude and gyro bias. This replaces `gyroBiasPrior` before graph initialization. That field remains an input for callers that initialize GraphManager directly.

With `estimateGravityFromImu=true`, the window supplies the gravity magnitude and the configured accelerometer bias prior is retained. With `false`, the configured gravity magnitude is retained and the accelerometer bias prior is estimated from the gravity residual. The world gravity vector is derived from the final magnitude when the graph is initialized. It is not an independent gravity-direction setting.

## Parameter applicability

Names below are the exact C++ fields without their trailing underscore.

| Fields | Applies when |
|---|---|
| `verboseLevel` | Selected diagnostics. It does not silence all output. |
| `odomNotJumpAtStartFlag` | Initial world/odometry alignment. |
| `logRealTimeStateToMemoryFlag`, `logLatencyAndUpdateDurationToMemoryFlag` | Collection in memory. Call the matching export method to write files. |
| `imuRate`, `createStateEveryNthImuMeasurement`, `imuBufferLength`, `imuTimeOffset`, `isImuAccInG` | IMU buffering, timing, graph-state creation, and unit conversion. |
| `useImuSignalLowPassFilter`, `imuLowPassFilterCutoffFreqHz` | The cutoff applies only when filtering is enabled. |
| `staticAtStartup`, `estimateGravityFromImuFlag`, `gravityMagnitude`, `W_gravityVector` | Initialization and propagation as described above. |
| `realTimeSmootherLag`, `realTimeSmootherUseIsamFlag`, `realTimeSmootherUseCholeskyFactorizationFlag` | The selected real-time fixed-lag smoother. |
| `useAdditionalSlowBatchSmootherFlag`, `slowBatchSmootherUseIsamFlag`, `slowBatchSmootherUseCholeskyFactorizationFlag` | Batch settings apply only when the additional batch smoother is enabled. Both ISAM2 and LM are implemented. |
| `minOptimizationFrequency`, `maxOptimizationFrequency`, `additionalOptimizationIterations` | Real-time optimization scheduling and extra updates. |
| `usingBiasForPreIntegrationFlag` | Whether prediction uses the estimated bias or zero bias. |
| `useWindowForMarginalsComputationFlag`, `windowSizeSecondsForMarginalsComputation` | LM batch marginal calculation. The window size applies only with windowing enabled. |
| `optimizeReferenceFramePosesWrtWorldFlag`, `referenceFramePosesResetThreshold`, `centerMeasurementsAtKeyframePositionBeforeAlignmentFlag`, `createReferenceAlignmentKeyframeEveryNSeconds` | Reference-frame alignment of applicable absolute measurements. |
| `optimizeExtrinsicSensorToSensorCorrectedOffsetFlag` | Extrinsic estimation for applicable measurements. |
| `accNoiseDensity`, `integrationNoiseDensity`, `gyroNoiseDensity`, `accBiasRandomWalkNoiseDensity`, `gyroBiasRandomWalkNoiseDensity` | IMU integration covariance. |
| `earthRotationCompensationFlag`, `latitudeDeg`, `worldFrameNorthAlignedFlag` | Latitude and north alignment apply only with Earth rotation compensation enabled. |
| `accBiasPrior`, `gyroBiasPrior` | Initial bias means, subject to the initialization policy above. |
| `initialPositionStdDev`, `initialOrientationStdDev`, `initialVelocityStdDev`, `initialAccBiasStdDev`, `initialGyroBiasStdDev` | Initial state priors and applicable optimizer recovery priors. |
| `gaussNewtonWildfireThreshold`, `findUnusedFactorSlotsFlag`, `enableDetailedResultsFlag` | ISAM2 only. Detailed results are computed internally. No automatic detailed-result export is provided. |
| `positionReLinTh`, `rotationReLinTh`, `velocityReLinTh`, `accBiasReLinTh`, `gyroBiasReLinTh`, `landmarkReLinTh` | ISAM2 relinearization of the corresponding variable type. |
| `referenceFrameReLinTh`, `calibrationReLinTh`, `displacementReLinTh` | ISAM2 with the corresponding reference-frame or extrinsic variables enabled. |
| `relinearizeSkip`, `enableRelinearizationFlag`, `evaluateNonlinearErrorFlag`, `cacheLinearizedFactorsFlag`, `enablePartialRelinearizationCheckFlag` | ISAM2 only. `relinearizeSkip` is a positive update interval between relinearization checks. |
| `maxSearchDeviation` | Timestamp-to-graph-key matching. ROS adapters derive it from the graph-state interval. |

All 65 fields have consumers in the built core. `GraphMsfDualGraph` is not part of the core build and is not a supported alternative implementation.

## Optimizer API limits

`optimize(maxIterations)` uses the iteration limit in the LM batch backend. The ISAM2 batch backend performs one update. The fixed-lag wrappers return their current estimate. Do not use this argument to control fixed-lag update iterations.

GTSAM's LM fixed-lag smoother uses scalar damping. It does not implement every damping option available to the full LM batch optimizer. These options are not exposed as GraphConfig settings.

## Earth rotation migration

`use2ndOrderCoriolis` and the scalar `omegaCoriolis` are not supported configuration keys. Use `earthRotationCompensation`, `latitudeDeg`, and `worldFrameNorthAligned` for the GTSAM 4.3 rotating-frame model. This model is not numerically equivalent to the GTSAM 4.2 Coriolis correction.

Enable `worldFrameNorthAligned` only when the world y-axis points north. Otherwise only the vertical Earth-rate component is applied.
