# DF3 onboard runtime

Enable the current runtime with `OPTIONS=USE_DF3` on a normal Betaflight target
or board configuration. `mk/df3.mk` owns the defaults and backend selection.
No library from the simulator repository is needed to build firmware.

The optional `USE_DF3_PROFILE` flag adds timing diagnostics. For example,
`OPTIONS="USE_DF3 USE_DF3_PROFILE"` includes the existing timing counters,
retained failure details and first-sensor automatic profile capture. Without
it, the runtime and normal state/diagnostic telemetry remain available, but the
detailed profiler and its service/failure recorder are omitted.

`USE_DF3_BLACKBOX` separately enables detailed onboard controller/fusion logs.
It is off by default: normal builds omit the DF3 trace collection, fields and
extra storage, while ordinary Betaflight Blackbox logging remains available.
Use `OPTIONS="USE_DF3 USE_DF3_BLACKBOX"` when those observations are needed.
In those builds, `df3_blackbox` selects logging at runtime: 0 off (default),
1 lateral, 2 vertical, 3 both. Snapshot and fusion-trace collection are disabled
when disarmed, when this selection is off, or when the Blackbox device is NONE.

The standard runtime uses 30 Hz windowed fusion, resumable deadline-budgeted
work, and a 250 Hz outer controller. `df3_runtime.h` defines this complete
runtime for both C and ARM kernels; its implementation guards are not separate
production options. Profiling does not select a different runtime.
Controller gains and sensor geometry remain in the existing `df3_*` settings;
this build selection does not replace a board's calibrated profile.

## Output corrections

The current-time EKF projection remains separate from the position/velocity
sent to the controller and state telemetry. A small translation observer
predicts that output using the preceding estimated physical acceleration,
then applies the disagreement with the new projection using a 15 ms time
constant. A consistent constant-acceleration trajectory passes without lag;
measurement-induced corrections are spread over successive output calls.
This does not remove acceleration noise or establish absolute XY position.

Acceleration, attitude and bias outputs are unchanged. The observer never
feeds back into the EKF, its covariance, range policy or bias adaptation.
`covarianceTimeUs` still identifies the internal fused state's covariance;
it is not a covariance estimate for the smoothed output. Sensor freshness
and fault checks remain authoritative, and the normal fusion reset clears
the observer before another flight. The cost is 48 bytes of fixed state,
one exponential and constant work per valid output, with no new option or
telemetry payload. Longer correction times require renewed closed-loop
validation: smoothing can reduce damping and increase physical jitter.

## Onboard Blackbox observations

With `USE_DF3 USE_DF3_BLACKBOX` and the target's normal `USE_BLACKBOX`, schema 4
selects a subset of the 147 fields defined in `df3_blackbox.h`: 113 lateral,
75 vertical, or all 147. Shared fields include attitude, heading, sensor ages,
range clearance, body forces, biases and validity. The header records
`df3_log_axes`; the selection stays fixed throughout each recording. Setting
`df3_blackbox = 0` omits DF3 fields while retaining ordinary Blackbox logging.
Normal Blackbox arming and storage settings still apply; DF3 main frames are
capped at 250 Hz, with slower requested P-frame rates retained. Retain
motor, magnetometer, battery, gyro and accelerometer fields for diagnosis.

On FlashFS builds, MSP2_DF3_BLACKBOX (0x30D4) reads settings/storage and
MSP2_SET_DF3_BLACKBOX (0x30D5) changes them. Both use a 17-byte envelope:
version 1, nonzero transaction ID (u32), and expected FC UID (three u32s),
all little-endian. SET appends one byte: axes 0–3 or action 4 to erase flight
logs. The 30-byte response echoes the envelope then axes, ready, editable,
device, sample-rate (five u8s), total bytes and used bytes (two u32s).

Writes require disarmed and stopped logging. Selecting nonzero axes saves the
selection in its own PG and configures FLASH, NORMAL, and requested 1/16 sampling
without a reboot. The rate cap is applied when initializing Blackbox, without
changing the saved sample-rate setting or MSP response. The log's I/P interval
headers record the effective intervals. Erase requires NORMAL mode, erases only the log partition, and never
repeats for the last accepted transaction ID. Poll read status until ready;
firmware and configuration are separate from log storage. Nimbus confirms
log deletion, holds arming until completion/readback, and does not retry an
uncertain erase write. Download wanted logs before erasing.

The adapter freezes a snapshot after each 250 Hz controller calculation.
Reading it for Blackbox does not advance the controller or output observer.
`df3Sample` identifies repeated snapshots, and `time - df3AgeUs` gives the
snapshot's FC time. All other `*AgeUs` values are relative to that snapshot;
subtract them as well to obtain a measurement's FC timestamp. Unwrap the normal
Blackbox microsecond clock before subtracting ages. `-1` means no usable time.
`df3FusionAgeUs` refers to covariance time, not the 15 ms output time constant.

`df3OutputReason` records the latest `df3FusionOutput` result. After the first
valid output while armed and collecting diagnostics, the first invalid call
latches `df3OutputFaultReason`, `df3OutputFaultMs`,
`df3OutputFaultNominalAgeUs` and `df3OutputFaultPhase`. They survive recovery
and later failures until disarm resets the collector. The timestamp is FC
uptime in milliseconds modulo 2^30, independent of the later Blackbox sample.
Nominal age is measured at that failure, before an incomplete projection can
advance the committed cache; -1 means absent/future. Phase is the worker's
internal fusion phase, or 0 when idle. `df3OutputFaultCount` counts invalid
output calls after the first valid call, modulo 2^30, not distinct episodes.
Startup invalidity updates the current reason but does not occupy the first
failure latch. These fields are shared by lateral and vertical logs.
`df3ControlFaultOutputReason` separately retains the output reason consumed
when the controller first enters FAULT: -1 until that happens, 0 if the output
was valid (so another controller guard caused the fault). This distinguishes
the shutdown from earlier invalid polls that did not reach an engaged controller.

| Reason | Meaning |
| ---: | --- |
| 0 | Valid output |
| 1 | Missing argument |
| 2 | No published estimate |
| 3 | Estimator failure already latched |
| 4 | Estimator not initialized |
| 5 | Current time precedes tick, bootstrap or covariance time |
| 6 | Committed nominal timestamp is in the future |
| 7 | Gyro-history interval unavailable (coverage, gap or tail limit) |
| 8 | Nominal projection failed its finite-state/quaternion checks |
| 9 | Eight-step output catch-up limit exhausted |
| 10 | Committed snapshot invalid |
| 11 | Bootstrap settling interval incomplete |
| 12 / 13 | Committed IMU timestamp in the future / older than 200 ms |
| 14 / 15 | Committed attitude timestamp in the future / older than 250 ms |
| 16 | Output observer failed (time order or nonfinite motion) |
| 17 | Research-only queued-IMU projection age invalid |

The first failing guard determines the reason. These observations do not alter
the eight-step limit, estimator validity, controller fault latch or recovery
policy. Reason 3 identifies a previously failed estimator; it does not identify
which earlier fusion operation failed. Range-reference validity is separate
from output validity and remains visible in `df3Flags`.

Unless noted below, floating values are SI values multiplied by 1000 and
rounded. Position/velocity/acceleration are local NED, so upward is negative Z.
Finite values clip to ±1,000,000,000; `-1,000,000,001` denotes nonfinite data.
Validity flags and sequence numbers distinguish initialization from real zeros.

| Fields | Meaning / other scales |
| --- | --- |
| `df3RefPz/Vz/Az`, `df3Pz/Vz/Az` | Controller's effective reference (including hold/landing fallback) and the actual estimate it used |
| `df3ProjectedPz/Vz` | Current-time EKF projection before the output observer |
| `df3NativeYaw`, `df3YawReference`, `df3Yaw`, `df3YawRate` | Native BF heading, effective yaw target, DF3 heading (degrees ×1000), and commanded native yaw rate (degrees/s ×1000); wrap heading differences before comparing |
| `df3Qw/Qx/Qy/Qz` | Body-FRD to local-NED quaternion ×1,000,000 |
| `df3ForceX/Y/Z` | Latest FC-filtered specific force in body FRD, before window reduction; not raw sensor-register data |
| `df3RangeRaw`, `df3RangeDown`, `df3RangeStrength` | Latest admitted raw slant range in mm, negative tilt-corrected clearance ×1000, and unscaled signal strength |
| `df3RangeAgeUs`, `df3RangeRxAgeUs`, `df3ImuAgeUs` | Adapter acquisition/receipt ages; range acquisition time is the existing mapped MTF time |
| `df3Terrain`, `df3BiasX/Y/Z` | Terrain NED down and body-FRD accelerometer biases |
| `df3ImuSeq`, `df3RangeSeq` | Observation counters; only a change denotes a new fusion observation |
| `df3ImuFusionAgeUs`, `df3RangeFusionAgeUs` | Acquisition ages of those fusion observations |
| `df3ImuZ/Innovation/R` | Windowed body-Z specific force, pre-update residual, and actual adaptive variance; variance is (m/s²)² ×1000 |
| `df3ImuRoughness` | Vertical reducer roughness ×1,000,000; diagnostic only, not added to Z observation variance |
| `df3RangeZ/Innovation/R` | Range-derived global NED position, residual and actual range variance; variance is m² ×1,000,000 |
| `df3ImuDv/Da`, `df3RangeDv/Da` | Applied NED-Z velocity/acceleration corrections ×1,000,000; zero for rejected/skipped updates |
| `df3ImuStatus`, `df3RangeStatus` | `df3Status_e`; range `-1` means the policy skipped the Kalman update. Interpret only with a nonzero sequence |
| `df3RangeFlags` | Policy mode in bits 0–1; has-position, plausible and surface-locked in bits 2–4 |
| `df3Ff`, `df3P`, `df3V`, `df3I` | Actual acceleration contributions: feedforward, Kp×position error, Kv×velocity error and Ki×candidate integral |
| `df3AccelRequest/Applied` | Z acceleration before/after controller limits |
| `df3Integral`, `df3IntegrationHeld` | Retained integral (m·s ×1000) and antiwindup decision; candidate I can differ from retained I |
| `df3Hover`, `df3ThrottleRequest/Applied` | Voltage-compensated hover collective and collective before/after controller clipping, all ×10,000; final motor outputs remain the normal Blackbox fields |
| `df3RefAgeUs`, `df3RefSeq` | Accepted reference receipt age and wire sequence |
| `df3Flags` | Control mode in bits 0–2; authority, estimate-valid, vertical-reference-valid and reference-present in bits 3–6 |
| `df3Diagnostics`, `df3Epoch`, `df3Queue` | Existing fault mask, state epoch and pending fusion-event count |
| `df3LogDrops` | Bytes rejected by Blackbox's output buffer since boot |
| `df3RefPx/Vx/Ax`, `df3RefPy/Vy/Ay`, `df3Px/Vx/Ax`, `df3Py/Vy/Ay` | Effective X/Y references and estimates actually used by control, local NED |
| `df3ProjectedPx/Vx`, `df3ProjectedPy/Vy` | X/Y current-time EKF projection before the 15 ms output observer |
| `df3Ffx/y`, `df3Ptermx/y`, `df3Vtermx/y`, `df3Itermx/y` | X/Y feedforward and position, velocity and candidate-integral acceleration contributions |
| `df3AccelRequestx/y`, `df3AccelAppliedx/y` | X/Y acceleration before/after tilt limiting and flow-loss suppression |
| `df3Integralx/y`, `df3IntegrationHeldx/y` | Retained lateral integrals and conditional antiwindup decisions; flow loss also freezes the candidate |
| `df3RollRequest`, `df3PitchRequest` | Angle commands sent to Betaflight, degrees ×1000. BF pitch has the opposite sign to quaternion FRD pitch |
| `df3ImuX/Y`, `df3ImuInnovationX/Y`, `df3ImuRX/RY` | Windowed body-FRD lateral specific force, pre-update residual and actual variance, using the existing `df3ImuSeq/FusionAgeUs/Status` |
| `df3ImuDvX/Y`, `df3ImuDaX/Y` | Applied local-NED X/Y IMU velocity/acceleration corrections ×1,000,000 |
| `df3ImuRoughnessXY` | Lateral reducer roughness ×1,000,000; diagnostic only, not added to X/Y observation variance |
| `df3FlowSeq/FusionAgeUs/Status` | Consumed flow-observation sequence, acquisition age and fusion result; `-1` means skipped/not yet completed |
| `df3FlowX/Y`, `df3FlowInnovationX/Y`, `df3FlowRX/RY` | Effective body-FRD COM velocity observation/residual after quantization handling; actual R is (m/s)² ×1,000,000 |
| `df3FlowDvX/Y`, `df3FlowDaX/Y` | Applied flow corrections to local-NED X/Y velocity/acceleration ×1,000,000; zero on rejection or skip |
| `df3FlowRawX/Y`, `df3FlowRotationX/Y`, `df3FlowLeverX/Y` | Latest enqueued report's body velocity after distance scaling, rotation correction and lens-offset correction, m/s ×1000; their sum is the pre-quantization COM observation |
| `df3FlowAgeUs`, `df3FlowIntervalUs` | Midpoint age and duration of that frontend report's acquisition interval, microseconds |
| `df3FlowQuality`, `df3FlowClearance`, `df3FlowGyroP/Q` | Quality (0–255), tilt-corrected clearance (m ×1000), and interval-average body-FRD roll/pitch gyro rates (rad/s ×1,000,000) for the same enqueued report |
| `df3FlowAccepted/Rejected` | Frontend enqueue-success and discard counters; these are distinct from EKF acceptance/rejection |

IMU fields describe the latest completed operation. Range/flow sequences begin
when a report is consumed; status remains -1 until an update completes, or
stays -1 if policy skips it. Group repeated samples by sequence and use the last
snapshot to inspect the result. Completion may precede publication of its whole
epoch. A core error also retains its `df3Status_e` code; consult validity/fault
flags (in particular, -1 is also `DF3_INVALID_ARGUMENT`). A skipped range has no applied correction;
its global-position measurement is unavailable if the policy had no position.
These are snapshots, not a complete raw-sensor replay: skipped sequence numbers
mean intermediate observations were not captured. Match frontend and fused flow
by their acquisition ages, not by neighboring log rows: they refer to different
points in the delayed pipeline. Quantization can change the effective flow
observation/variance, so it need not equal the frontend sum. The log contains no
physical ground truth. Compare slow/fast takeoffs using common FC timestamps, spectra,
range/IMU residuals, correction magnitudes, controller terms and actual motors.

The log header includes schema version, DF firmware version, configured XYZ gains,
hover/acceleration scale, calibration float bit patterns and observer time.
I-frames store signed variable-length integers; P-frames use previous-value
differences in eight-field groups. An unchanged group costs one tag byte.
Readers need room for up to 256 main fields, 4096-byte headers and 2048-byte
frames. Pandora's vendored decoder has matching limits; rebuild that decoder
before importing these logs. The 64 original DF3 fields retain their names,
order and scaling, but readers with the old capacity limits can discard the
expanded frames or headers.

### Recording capacity and throughput

MCU firmware flash and Blackbox recording flash are separate resources on the
AIR75 II. A successful firmware link proves only that the program fits. Read
`flash_info` on the disarmed aircraft to obtain actual FlashFS `size`,
`usedSize`, `freeSize`, buffer space and `Blackbox droppedBytes`. A missing/zero
FlashFS volume cannot record. Do not infer chip capacity from the MCU model.

Ordinary Blackbox uses a fixed PID-loop divisor, not automatic CPU-load
throttling. With DF3 fields selected, the effective divisor is increased to
keep main I/P frames at or below 250 Hz for the configured PID period. A
requested 1/16 becomes 1/32 at 8 kHz and stays 1/16 at 4 kHz; slower P rates
are preserved. I-frame boundaries are aligned with the P cadence so they
cannot insert an extra short interval. This limits main-frame production,
not event/GPS records, flash DMA bursts or total CPU time. Existing I-only
behavior still applies when the requested P interval exceeds the I interval.
With `df3_blackbox = 0`, normal Betaflight scheduling is unchanged.
Inspect `df3Sample` for gaps; logging faster cannot increase DF3's update rate.
Runtime collection remains off on
existing aircraft until a nonzero selection is saved; the LUX 1.3.44 setup
profile selects lateral diagnostics.

Measure a representative short recording and calculate
`bytesPerSecond = (usedSizeAfter - usedSizeBefore) / recordedSeconds` after the
normal disarm/flush. Budget `0.75 * freeSize / bytesPerSecond` seconds for the
next recording, allowing 25% headroom. Include noisy motion in the rate sample:
variable-length deltas grow with signal activity. For a conservative bound,
the DF3 addition alone is at most 580/385/754 bytes per P-frame for
lateral/vertical/both (565/375/735 per I-frame), plus existing Blackbox fields,
headers and events. These are bounds, not measured recording rates. Measure
storage use after changing the selected axes or sample rate.

DF3 builds use a 2048-byte asynchronous flash ring (2047 usable) to fit the
larger frames; indices are wide enough to wrap correctly. Full-buffer writes
are rejected and counted instead of overwriting bytes owned by flash DMA.
This does not guarantee sustained flash throughput: compare the drop counter
before/after recording, inspect decoded frame gaps, and verify task timing on
hardware. A partial frame from any dropped byte is not valid evidence. Download
and preserve wanted flights before using the existing erase operation.

## Sensor timing and noise weighting

DF3 uses the complete MTF packet's raw millimetre range, tilt-corrected and
stamped at the mapped acquisition time. It preserves the rangefinder validity
gate; the legacy five-sample median remains available to other consumers.
Stamping that median as a current observation adds a motion-dependent delay
that can be mistaken for accelerometer bias.

Flow conversion uses that same raw range report and carries its signal strength
to fusion. The MTF-02's observed quantization is 10 cm/s at 1 m (the wire unit is
still 1 cm/s), so velocity bin width is `0.1 * height` m/s. SITL can declare a
different sensor resolution through its existing environment override.

Flow's quality/rotation variance keeps its existing floor below 1 m and scales
with height squared above it. Independent range uncertainty also contributes:
linearizing `v = height * angularFlow` at predicted velocity gives a shared X/Y
scale error. A diagonal upper bound preserves the quantized update's diagonal
covariance requirement. This does not add gyro-bias covariance twice; that
uncertainty is already propagated through the observation Jacobian. The range
noise history available at the flow midpoint is used with this packet's strength.
These conservative weights need flight validation; simulation cannot establish
the real camera's delay, vibration sensitivity or complete noise distribution.

The IMU reducer carries a bounded second-difference noise proxy alongside the
unchanged windowed specific force. This proxy remains diagnostic on all axes:
high-rate roughness does not measure uncertainty of the window mean. IMU
observation variance uses its nominal covariance and innovation-based outlier
weighting. Range weighting uses a bounded third-difference noise proxy and a
positive variance floor. These proxies reject low-order motion; they are not
calibrated estimates of every sensor's true noise distribution.
Range variance is `(0.001 + 5 * measuredNoiseVariance)` m² before the existing
signal-strength scaling. Raising IMU uncertainty alone can increase acceleration
corrections from range; validate these weights together.

Nominal IMU variances are 0.324, 0.324 and 1.200 (m/s²)². Jerk process
intensities are 2.7, 2.7 and 5.4, and accelerometer-bias random-walk intensities
are 0.1, 0.1 and 1.0 (m/s²)²/s. Body-Z bias tracking is restored after the
1.3.35 disable experiment, with initial variance 0.36 (m/s²)². The extra
roughness-based observation variance is removed on all three axes; measured
roughness is retained for diagnostics only.
Prediction constants are shared by synchronous and
multirate implementations. The existing bias states, covariance update,
stationary constraints and terrain policy remain responsible for bias learning.
Do not boost body-Z bias covariance from signed IMU innovation alone: real
periodic acceleration also produces persistent residuals over each half-cycle.
The former 200 ms average could add another 9 (m/s²)²/s during unbiased 1 Hz
motion, making the bias estimate follow motion and delaying velocity feedback.
Bias learning now uses only the nominal process noise and Kalman updates; there
is no extra innovation-driven boost, hard bias clamp, or change to controller
integration. The 1 Hz observation regression checks phase and amplitude as well
as false bias. This restriction reduces the reproduced error, but does not
eliminate acceleration lag or establish closed-loop flight stability.

A rejected flow report does not invalidate a previously admitted measurement.
Only a successful enqueue refreshes its age; sustained loss still expires at
the existing 150 ms controller bound. No extra telemetry, debug option or
controller input is introduced by these changes.

## Fixed-gain outer controller

FC 1.3.42 adds disarmed, addressed MSP tuning without changing the 1.3.41
defaults. `MSP2_DF3_LQR` (0x30D2) reads and `MSP2_SET_DF3_LQR` (0x30D3)
validates, persists and reloads all nine controller gains. Both use a 17-byte
header: version 1, nonzero little-endian u32 transaction ID, and three u32
hardware UID words. SET and replies append nine little-endian u16 values:
Kp XYZ, Kv XYZ, Ki XYZ, each in SI units multiplied by 1000. Limits match
the CLI: 30000, 20000, 10000 respectively. The complete SET is checked before
any PG value changes; identical values do not rewrite EEPROM. Reload clears
controller integrals while preserving the estimator and reference connection.

Nimbus solves the 250 Hz discrete Riccati equation from diagonal Q
(position, velocity, integral) and scalar R independently for each axis. It
accounts for integrating the current error before feedback and checks the
rounded gains against the ideal model. Only gains are persisted in the
existing DF3 PG: Q/R remain labeled SDK drafts, since gains cannot uniquely
identify their cost matrices. The SDK serializes calibration and LQR
transactions, requires fresh disarmed telemetry, and verifies a save with
an independent GET. A lost SET acknowledgement triggers readback, not another
SET. No controller scheduling, estimator noise, observer time, calibration,
PG layout, or ELRS firmware change is needed for this feature.

FC 1.3.27 restores the original FC 1.3.22 flight-profile Riccati gains:
Kp/Kv/Ki = 1.208/1.641/0.105 on X/Y and 2.987/2.642/0.501 on Z.
The controller uses the configured gains directly for both feedback and
antiwindup. There are no quiet/recovery endpoints, error-dependent gain
thresholds or gain-scheduling option. Firmware task scheduling is unchanged.

Firmware settings store the fixed gains in SI units multiplied by 1000.
Updating firmware preserves saved PG values, so matching NimbusOS AIR75 II
flash profiles restore all nine gains on existing aircraft too. This returns
both lateral and vertical coefficients to the original R7 design, without
applying the subsequent Qv multipliers.

The estimator improvements and 15 ms output observer are retained. An invalid
vertical reference still suppresses Z position feedback and integral accumulation,
while configured velocity feedback and the existing integral remain active.
Reference-loss handling, native/manual priority and actuator limits are unchanged.
Restoring historical coefficients does not by itself qualify the resulting
estimator/controller combination on hardware.

Z accumulated position-error headroom is 3 m·s, allowing 1.503 m/s² of sustained
correction at Ki = 0.501.
The previous 0.5 m·s limit saturated during a long simulated battery discharge.
Existing acceleration, throttle and conditional antiwindup limits still apply;
this headroom does not replace a valid hover/thrust calibration.

Native stress tests and full simulations measure errors, spikes and physical
motion spectra separately. These results do not establish vibration rejection,
timing margins or flight qualification for a particular aircraft.

## Backend and target support

- Cortex-M4/M7 hard-float targets compile `joseph_m4f.S` and
  `df3_reset_m4f.S` directly. Their scalar floating-point instructions are
  compatible with both cores. `joseph.h` defines and checks the kernel ABI.
- SITL and other architectures use the portable C implementations with the
  same runtime. Hardware budgeting uses Betaflight's cycle-counter API;
  a new MCU port must provide that API and enough CPU and memory capacity.
- Only STM32G474xx selects the existing DF3 executable-CCM linker layout.
  Other targets use their normal executable-memory layout.
- DF3 C objects retain `-O3 -fno-fast-math -ffp-contract=off`. Do not enable
  floating-point reassociation or fused multiply-add in these calculations.

Build compatibility is separate from board qualification. Check linked RAM,
flash and stack margin, sensor/receiver configuration, and on-device deadline
behavior before treating another board as flight ready. CPU emulation verifies
arithmetic and ABI behavior; it does not measure hardware execution deadlines.

## Research variants

Production has one runtime; `mk/df3.mk` contains no research mode, external
assembly library, or emulator selection. Its source lists and assembler flags
are derived from the target. Normal builds need only `USE_DF3`.

Retained DF_Sim experiment builders select their own `tools/df3/research.mk`
through the Makefile's `DF3_BUILD_RULES` override. That optional file owns the
historical explicit runtime options and Unicorn linking. It is neither included
nor required by a normal Betaflight checkout. `mk/mcu/SITL.mk` contains only
host-platform setup.

The internal conditional C implementations remain available for numerical
regression tests using direct compiler invocations. Those tests compare C/ARM,
synchronous/resumable and 30/100 Hz variants; these are not production switches.
Research builds and direct-compiler comparisons omit `df3_runtime.h` and retain
their explicit historical definitions. `USE_DF3_SCHED_BENCH` belongs to this
research path; it is not enabled by the production profiling option.

The simulator's `assembly/` directory retains the C/x86 comparison kernels,
emulator, benchmarks and frozen baselines. Its builders consume the current
production ARM kernel here; frozen baseline inputs retain their original code.

## Source map

- `df3_core.c`: estimator arithmetic, including resumable and gyro-history phases.
- `df3_estimator.c`: sensor/event fusion and committed estimate publication.
- `df3_resumable.h`, `df3_multirate.h`: job ownership and sensor-window contracts.
- `df3_flow.c`, `df3_policy.c`: flow compensation and observation policy.
- `df3_control.c`: local outer controller and fallback behavior.
- `df3_protocol.h`, `df3_reference.c`, `df3_state.c`, `df3_diagnostics.c`:
  wire layout, reference validation and telemetry.
- `df3_betaflight.c`: sensor, scheduler and controller integration.
- `df3_profile.c`, `df3_budget.h`: retained timing facilities and work budget.

### Included C implementation files

All six `.inc` files are **production code** in the standard `USE_DF3` runtime.
They are handwritten C fragments included into the owning `.c` file, not
separately compiled sources. This keeps their implementation next to the
private helpers, types and constants of that translation unit without exposing
additional public interfaces. Their filenames describe their responsibilities;
the `.inc` extension does not indicate experimental status.

| File | Included by | Production responsibility |
| --- | --- | --- |
| `df3_resumable_core.inc` | `df3_core.c` | Split prediction and measurement-update arithmetic into bounded phases. |
| `df3_multirate_core.inc` | `df3_resumable_core.inc` | Integrate gyro history, propagate rotation/bias uncertainty, and form the windowed observation model. |
| `df3_resumable_estimator.inc` | `df3_estimator.c` | Own fusion transactions, sequence sensor events, and publish/project committed estimates. |
| `df3_multirate_estimator.inc` | `df3_resumable_estimator.inc` | Integrate the acceleration observation model and gyro-bias Jacobian over the acquisition window. |
| `df3_multirate_frontend.inc` | `df3_flow.c` | Reduce accelerometer samples into windows, limit attitude admission, and track stationary eligibility. |
| `df3_multirate_flow.inc` | `df3_flow.c` | Apply optical-flow quantization correction in bounded steps. |

**Retained research paths:** the non-multirate alternatives in the resumable
implementation, the 100 Hz cadence, and ARM emulation in SITL remain available
for comparisons through the research builders or direct compiler invocations.
Comments mark those alternatives where they differ from the production path.
The portable C backend and native ARM backend are both production
implementations selected by the target; neither is an experimental fallback.
The shared multirate code is used by both production and research builds.

The local `.clang-format` covers C, headers and included C fragments. Kernel
instructions and arithmetic order must stay unchanged during formatting edits.
