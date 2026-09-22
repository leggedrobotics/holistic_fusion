# GraphMSF configuration

`GraphConfig` is the core configuration contract. ROS adapters expose YAML names without the trailing underscore and usually without the `Flag` suffix. A field can be supported without affecting every optimizer or measurement type.

## IMU covariance

The accelerometer and gyroscope covariance inputs are continuous-time noise densities squared:

```text
accelerometerCovariance = I * (accNoiseDensity² + biasAccStdDevForIntegration²)
gyroscopeCovariance = I * (gyroNoiseDensity² + biasOmegaStdDevForIntegration²)
```

The two `bias*StdDevForIntegration` fields are Holistic Fusion integration-noise settings. They have the same units as the corresponding sensor noise density. They are not initial bias-state priors. Zero disables their contribution.

GTSAM 4.3 does not apply `setBiasAccOmegaInit`. Holistic Fusion applies the independent diagonal contributions through the supported accelerometer and gyroscope covariance setters. This preserves the contribution made by these fields in GTSAM 4.2 with the tangent preintegration backend and measurements expressed at the IMU origin. It does not claim identical factor residuals or identical estimates across GTSAM versions.

`accBiasRandomWalkNoiseDensity` and `gyroBiasRandomWalkNoiseDensity` describe bias evolution. `initialAccBiasStdDev` and `initialGyroBiasStdDev` set the uncertainty of the first bias state. These settings have separate effects.

The supplied GTSAM build disables `GTSAM_ALLOW_DEPRECATED_SINCE_V43`. Calls to the ignored compatibility APIs therefore fail to compile.

## Initialization

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
| `estimateGravityFromImuFlag`, `gravityMagnitude`, `W_gravityVector` | Initialization and propagation as described above. |
| `realTimeSmootherLag`, `realTimeSmootherUseIsamFlag`, `realTimeSmootherUseCholeskyFactorizationFlag` | The selected real-time fixed-lag smoother. |
| `useAdditionalSlowBatchSmootherFlag`, `slowBatchSmootherUseIsamFlag`, `slowBatchSmootherUseCholeskyFactorizationFlag` | Batch settings apply only when the additional batch smoother is enabled. Both ISAM2 and LM are implemented. |
| `minOptimizationFrequency`, `maxOptimizationFrequency`, `additionalOptimizationIterations` | Real-time optimization scheduling and extra updates. |
| `usingBiasForPreIntegrationFlag` | Whether prediction uses the estimated bias or zero bias. |
| `useWindowForMarginalsComputationFlag`, `windowSizeSecondsForMarginalsComputation` | LM batch marginal calculation. The window size applies only with windowing enabled. |
| `optimizeReferenceFramePosesWrtWorldFlag`, `referenceFramePosesResetThreshold`, `centerMeasurementsAtKeyframePositionBeforeAlignmentFlag`, `createReferenceAlignmentKeyframeEveryNSeconds` | Reference-frame alignment of applicable absolute measurements. |
| `optimizeExtrinsicSensorToSensorCorrectedOffsetFlag` | Extrinsic estimation for applicable measurements. |
| `accNoiseDensity`, `integrationNoiseDensity`, `gyroNoiseDensity`, `accBiasRandomWalkNoiseDensity`, `gyroBiasRandomWalkNoiseDensity`, `biasAccStdDevForIntegration`, `biasOmegaStdDevForIntegration` | IMU integration covariance. |
| `earthRotationCompensationFlag`, `latitudeDeg`, `worldFrameNorthAlignedFlag` | Latitude and north alignment apply only with Earth rotation compensation enabled. |
| `accBiasPrior`, `gyroBiasPrior` | Initial bias means, subject to the initialization policy above. |
| `initialPositionStdDev`, `initialOrientationStdDev`, `initialVelocityStdDev`, `initialAccBiasStdDev`, `initialGyroBiasStdDev` | Initial state priors and applicable optimizer recovery priors. |
| `gaussNewtonWildfireThreshold`, `findUnusedFactorSlotsFlag`, `enableDetailedResultsFlag` | ISAM2 only. Detailed results are computed internally. No automatic detailed-result export is provided. |
| `positionReLinTh`, `rotationReLinTh`, `velocityReLinTh`, `accBiasReLinTh`, `gyroBiasReLinTh`, `landmarkReLinTh` | ISAM2 relinearization of the corresponding variable type. |
| `referenceFrameReLinTh`, `calibrationReLinTh`, `displacementReLinTh` | ISAM2 with the corresponding reference-frame or extrinsic variables enabled. |
| `relinearizeSkip`, `enableRelinearizationFlag`, `evaluateNonlinearErrorFlag`, `cacheLinearizedFactorsFlag`, `enablePartialRelinearizationCheckFlag` | ISAM2 only. `relinearizeSkip` is a positive update interval between relinearization checks. |
| `maxSearchDeviation` | Timestamp-to-graph-key matching. ROS adapters derive it from the graph-state interval. |

All 66 fields have consumers in the built core. `GraphMsfDualGraph` is not part of the core build and is not a supported alternative implementation.

## Optimizer API limits

`optimize(maxIterations)` uses the iteration limit in the LM batch backend. The ISAM2 batch backend performs one update. The fixed-lag wrappers return their current estimate. Do not use this argument to control fixed-lag update iterations.

GTSAM's LM fixed-lag smoother uses scalar damping. It does not implement every damping option available to the full LM batch optimizer. These options are not exposed as GraphConfig settings.

## Earth rotation migration

`use2ndOrderCoriolis` and the scalar `omegaCoriolis` are not supported configuration keys. Use `earthRotationCompensation`, `latitudeDeg`, and `worldFrameNorthAligned` for the GTSAM 4.3 rotating-frame model. This model is not numerically equivalent to the GTSAM 4.2 Coriolis correction.

Enable `worldFrameNorthAligned` only when the world y-axis points north. Otherwise only the vertical Earth-rate component is applied.
