# Earth-rate correction in stationary gyro-bias estimation

## Scope

Branch: `fix/earth-rate-gyro-bias`, based on the deployed manual-declination
source `7cfb72feeaaad43f4b9b2fb17c428e06cb2e7331` (v1.0.1).

Production change is a minimal backport of upstream PX4 commit
[`1624564f79f1534430069ec725d2116b6569d977`](https://github.com/PX4/PX4-Autopilot/commit/1624564f79f1534430069ec725d2116b6569d977):

- In `ZeroGyroUpdate.cpp`, subtract modeled Earth rotation in the body frame
  before treating a stationary gyro measurement as a bias observation.
- In `ekf.h`, expose the existing Earth-rate vector through a const getter.

No parameter defaults, magnetic-declination behavior, sensor routing, board
configuration, controllers, actuator allocation, bridge, or cuVSLAM code change.
No new state or ongoing external heading input is introduced. The getter uses
the same latitude-dependent model already used by attitude prediction. If that
model is still zero because position is unavailable, the correction is zero.

## Why the correction is needed

At rest, the calibrated gyro reading contains sensor bias plus Earth rotation.
Previously, the stationary bias observation included both. Attitude prediction
then subtracted the learned bias and the Earth-rate vector separately, creating
a residual angular rate. The incorrect learned bias remains after the vehicle
stops being classified as stationary; the defect is not confined to the ground.

At latitude 40.4135896 degrees, the approximately level yaw contribution is
about 0.1625 degrees/minute. A recorded stationary interval on 18 November 2026
showed 4.752 degrees of heading change in 28.329 minutes (0.170 degrees/minute).
This close match supports the diagnosis but does not identify every source of
mission heading error. The saved QGC firmware signature is consistent with the
v1.0.1 source; its version/parameter readbacks predate the stationary interval.

The current source's attitude predictor and old stationary-bias calculation are
unchanged from maintainer base `dc7f3f822e`. This is not a regression introduced
by the manual-declination patch.

## Regression tests

Tests exercise the real PX4 EKF with synthetic IMU, accelerometer, barometer and
magnetometer measurements, not a separate approximation of its equations.
They are added to `test_EKF_gyroscope.cpp` and use compass initialization only.
The synthetic gyro explicitly includes Earth rotation plus a known sensor bias.

Eight latitude/attitude combinations include northern and southern hemispheres,
80-degree latitude, four level headings, yaw wrap boundaries, and nonzero roll
and pitch. Each combination checks:

1. Bias converges to the sensor bias rather than absorbing Earth rotation.
2. Heading remains within 0.03 degrees over five simulated minutes at rest.
3. Heading remains within 0.03 degrees over five further minutes with the rest
   flag false and no continuous compass yaw fusion. This is a nonstationary
   estimator mode, not a full translational flight simulation.
4. Four 90-degree turns in each direction track correctly, including crossings
   of +/-180 degrees. Error at each settled quarter-turn is below 0.05 degrees.
   Quaternion-reset count does not change and compass yaw fusion remains off.

Bias tests allow the estimator to settle after establishing the geographic
origin. This synthetic settling period is not a new operational startup delay.
Ideal noiseless tests do not establish real hardware heading precision.

Results on 18 November 2026:

- Original production calculation with final regression tests: 8 bias/drift
  tests fail; the 8 shorter turn tests pass. This distinguishes the persistent
  bias defect from ordinary turn integration.
- Corrected calculation: all 16 added cases and 3 original gyro cases pass.
- Full EKF suite: **225 test cases across 26 executables pass**, including all
  67 magnetometer/manual-declination cases.
- PX4 style checks for the three edited C++ files and `git diff --check` pass.

Example original-code result at 40.41359 degrees, level, heading zero:
0.813163 degrees over five minutes stationary, followed by 0.813123 degrees
over five minutes with the rest flag false. Corrected results satisfy the
0.03-degree thresholds. Inspect the XML for individual synthetic results.

Evidence under `build/px4_sitl_test/`:
`earth-rate-before.xml`, `earth-rate-before.log`, `earth-rate-after.xml`,
`earth-rate-after.log`, `earth-rate-ekf-tests.xml`, and
`Testing/Temporary/LastTest.log`.

## What this does not establish about mission errors

A new 4–6 degree error over a short mission is not explained by the measured
Earth-rate component alone. Five level minutes corresponds to about 0.8 degrees;
include ground waiting time when comparing the start and end.

Other plausible mechanisms, not diagnosed from the available telemetry:

- Residual gyro bias, including temperature-dependent changes and vibration.
- Gyro scale or cross-axis calibration error during rotations. For illustration,
  a 1% rate scale error accumulates 3.6 degrees over a 360-degree turn; no such
  scale error has been measured on this aircraft.
- Ordinary EKF corrections driven by position aiding. `EKF2_MAG_TYPE=6` disables
  continuous magnetic yaw fusion but does not make heading pure gyro integration.
  GNSS position fusion computes Kalman gains for the full state and can update
  attitude/gyro bias through cross-covariance. Thus VO trajectory errors in the
  HIL_GPS positions can indirectly influence PX4 heading even with EV yaw off.
- Discrete estimator yaw resets, sensor selection/calibration changes, or an
  incorrect initial reference. These require timestamped evidence rather than
  inference from the final QGC arrow alone.

To distinguish these, obtain the affected mission's PX4 `.ulg`, the bridge log,
and marked physical start/end orientations and times. Relevant ULog data include
vehicle attitude and local-position heading/reset counters, estimator sensor
bias, IMU device selection/temperature/vibration/clipping, estimator flags and
GNSS innovations/test ratios. The existing QGC telemetry lacks sufficient raw
IMU/bias/reset information to isolate the reported mission error.

## Build and hardware verification

Build only `cubepilot_cubeorangeplus_default`, retaining the v1.0.1 toolchain and
board configuration. Package verification must check board ID 1063, source hash,
decompressed image equality with the linked binary, flash size, and unchanged
parameter names/defaults against v1.0.1. Keep v1.0.1 for rollback.

No aircraft was flashed or parameter changed for these tests. Hardware checks
are still required: confirm Cube Orange Plus identity and direct USB connection,
back up parameters, ensure disarmed/ground/propulsion-disabled status, then flash
only with operator approval. Preserve the intentional manual declination and
compass-init-only settings. Do not remove the declination fix or enable EV yaw.

Before flight, repeat the 30-minute stationary recording and marked clockwise/
counterclockwise turn-and-return tests. Check initial heading, drift rate, reset
counters, sensor health, parameter persistence and real-world north/east motion.
Use a physical alignment mark or independently surveyed direction, not another
magnetometer inside the same steel cage. A successful software build is not
flight-readiness approval.
