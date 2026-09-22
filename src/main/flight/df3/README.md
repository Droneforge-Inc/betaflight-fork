# DF3 onboard runtime

Enable the current runtime with `OPTIONS=USE_DF3` on a normal Betaflight target
or board configuration. `mk/df3.mk` owns the defaults and backend selection.
No library from the simulator repository is needed to build firmware.

The only additional production DF3 option is `USE_DF3_PROFILE`. For example,
`OPTIONS="USE_DF3 USE_DF3_PROFILE"` includes the existing timing counters,
retained failure details and first-sensor automatic profile capture. Without
it, the runtime and normal state/diagnostic telemetry remain available, but the
detailed profiler and its service/failure recorder are omitted.

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

With `USE_DF3 USE_DF3_BLACKBOX` and the target's normal `USE_BLACKBOX`, each main
log frame includes 136 DF3 fields (schema 2) defined in `df3_blackbox.h`.
No extra PG, radio packet or debug-mode selection is required. Normal Blackbox enable,
arming, sample-rate and storage settings still apply. Existing motor, battery,
gyro and accelerometer fields remain available; retain those for diagnosis.

The adapter freezes a snapshot after each 250 Hz controller calculation.
Reading it for Blackbox does not advance the controller or output observer.
`df3Sample` identifies repeated snapshots, and `time - df3AgeUs` gives the
snapshot's FC time. All other `*AgeUs` values are relative to that snapshot;
subtract them as well to obtain a measurement's FC timestamp. Unwrap the normal
Blackbox microsecond clock before subtracting ages. `-1` means no usable time.
`df3FusionAgeUs` refers to covariance time, not the 15 ms output time constant.

Unless noted below, floating values are SI values multiplied by 1000 and
rounded. Position/velocity/acceleration are local NED, so upward is negative Z.
Finite values clip to ±1,000,000,000; `-1,000,000,001` denotes nonfinite data.
Validity flags and sequence numbers distinguish initialization from real zeros.

| Fields | Meaning / other scales |
| --- | --- |
| `df3RefPz/Vz/Az`, `df3Pz/Vz/Az` | Controller's effective reference (including hold/landing fallback) and the actual estimate it used |
| `df3ProjectedPz/Vz` | Current-time EKF projection before the output observer |
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
| `df3ImuRoughnessXY` | Lateral reducer roughness ×1,000,000; current lateral R adds `1000 * roughness` to both body axes |
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

Begin around 500 Hz Blackbox sampling for the 250 Hz control snapshots; inspect
`df3Sample` for gaps and use lower rates only if the needed transients remain
resolved. The saved `1/4` sample-rate setting is a fraction of the PID rate,
not a fixed frequency. Logging faster cannot increase DF3's update rate.
No logging settings or stored flights are changed automatically.

Measure a representative short recording and calculate
`bytesPerSecond = (usedSizeAfter - usedSizeBefore) / recordedSeconds` after the
normal disarm/flush. Budget `0.75 * freeSize / bytesPerSecond` seconds for the
next recording, allowing 25% headroom. Include noisy motion in the rate sample:
variable-length deltas grow with signal activity. For a conservative bound,
the DF3 addition alone is at most 697 bytes per logged P-frame (680 per I-frame),
plus the existing Blackbox fields, headers and events. This is a bound, not a
measured aircraft recording rate. The 72 new fields add nine P-frame tag bytes
when unchanged, plus their signed deltas when they change and 72–360 bytes per
I-frame. Measure storage use again after this update; the old recording-duration
budget is no longer valid.

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
unchanged windowed specific force. The fusion step increases measurement
variance when that proxy rises. The proxy is tuned for the existing 25 Hz FC
accelerometer low-pass; bypassing or changing that filter requires retuning.
Range weighting similarly uses a bounded third-difference noise proxy and a
positive variance floor. These proxies reject low-order motion; they are tuning
signals, not calibrated estimates of every sensor's true noise distribution.
Range variance is `(0.001 + 5 * measuredNoiseVariance)` m² before the existing
signal-strength scaling. Raising IMU uncertainty alone can increase acceleration
corrections from range; validate these weights together.

Nominal IMU variances are 0.324, 0.324 and 1.200 (m/s²)². Jerk process
intensities are 2.7, 2.7 and 5.4, and accelerometer-bias random-walk intensities
are 0.1, 0.1 and 1.0. Prediction constants are shared by synchronous and
multirate implementations. The existing bias states, covariance update,
stationary constraints and terrain policy remain responsible for bias learning.
Persistent, range-anchored body-Z IMU innovation uses a 200 ms signed average.
Outside a 0.15 m/s² deadband it can add bounded bias covariance; this avoids
keeping the fast bias response active during ordinary clean motion. Both the
deadband and covariance bound must be validated together with sensor weighting.

A rejected flow report does not invalidate a previously admitted measurement.
Only a successful enqueue refreshes its age; sustained loss still expires at
the existing 150 ms controller bound. No extra telemetry, debug option or
controller input is introduced by these changes.

## Fixed-gain outer controller

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
