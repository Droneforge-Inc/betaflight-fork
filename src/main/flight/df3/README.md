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
