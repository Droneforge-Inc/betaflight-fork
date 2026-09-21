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
