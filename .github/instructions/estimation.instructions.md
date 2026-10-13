---
applyTo: "libraries/AP_NavEKF/**,libraries/AP_NavEKF2/**,libraries/AP_NavEKF3/**,libraries/AP_AHRS/**,libraries/AP_DAL/**,Tools/Replay/**"
---

# State estimation rules

- EKF2 and EKF3 read their inputs only through `AP_DAL`, so that Replay can
  reproduce a run from a log recorded with `LOG_REPLAY`. New estimator inputs
  go through the DAL and are logged by it; reading a sensor library directly
  breaks Replay. The `test replay` workflow only builds Replay and a
  standalone DAL tool, so it does not catch this.
- Check frames and conventions: NED earth frame, body frame, quaternion order
  and rotation direction.
- Kalman filter changes keep the covariance symmetric and positive definite,
  with sensible innovation variances and consistent units in the measurement
  model.
- Look for singularities (Euler angles near gimbal lock, sideslip at low
  airspeed) and aliasing when sensor data is downsampled without filtering.
- Mind CPU cost: estimators run on every board, often more than one instance.
  Avoid needless `sqrtf` and trigonometry in per-sample code.
- Changes to estimator behaviour need evidence from logs: replayed real logs
  or SITL runs showing the effect, not only a build.
