# Betaflight with AP autonomy

`USE_AP_AUTONOMY` statically links ArduPilot EKF3, position control and outer
attitude control into Betaflight. Betaflight retains its gyro/rate PID, mixer,
motor drivers, arming/failsafe logic and the existing DF3 CRSF/MSP interface.
The source port and integration were AI-assisted.

The required sources are inside this fork:

- `lib/main/ardupilot/`: dependency snapshot, licenses, upstream revision,
  hashes and the existing optical-flow acquisition timestamp patch.
- `src/main/flight/ap_autonomy/backend/`: one HAL/parameter/math configuration,
  estimator/controller bridges, bounded allocation and runtime initialization.
- `src/main/flight/ap_autonomy/`: native sensor/reference adapters, local frame,
  lifecycle, motor conversion and Blackbox fields.
- `mk/ap_autonomy.mk`: C++ compilation and target selection through the normal
  Betaflight build. No parent ArduPilot checkout or AP shared libraries are used.

## Build

Use Betaflight's usual ARM toolchain and initialized board-config submodule.
An existing config checkout can instead be selected with `CONFIG_DIR`, and the
ARM toolchain with `ARM_SDK_DIR`.

```sh
make CONFIG=BETAFPVG473_V2 \
  OPTIONS="USE_DF3 USE_AP_AUTONOMY USE_DF3_BLACKBOX" \
  OBJECT_DIR="$PWD/obj/ap-g4" -j8 \
  "$PWD/obj/ap-g4/betaflight_STM32G47X_BETAFPVG473_V2.elf"

make CONFIG=SPEEDYBEEF405MINI \
  OPTIONS="USE_DF3 USE_AP_AUTONOMY USE_DF3_BLACKBOX USE_MAG" \
  OBJECT_DIR="$PWD/obj/ap-f405" -j8 \
  "$PWD/obj/ap-f405/betaflight_STM32F405_SPEEDYBEEF405MINI.elf"

make TARGET=SITL \
  OPTIONS="USE_DF3 USE_AP_AUTONOMY USE_BLACKBOX USE_DF3_BLACKBOX SITL_CRSF_TAP" \
  OBJECT_DIR="$PWD/obj/ap-sitl" -j8

make -C src/main/flight/ap_autonomy/backend test
python3 lib/main/ardupilot/vendor.py --verify
```

`USE_DF3` without `USE_AP_AUTONOMY` still selects the original DF3 backend.
Its assembly kernels and resumable worker are not compiled into the AP backend.
The embedded AP build targets G474 and F405 boards and uses CRSF for
receiver/telemetry, retaining ELRS v3, OSD, VTX, Blackbox and motor features.
Alternative receiver protocols are excluded. BF's explicit speed-optimized
source list keeps its existing optimization; remaining BF code uses `-Os`.
AP code uses `-Os`, hardware float and IEEE finite checks, without fast math or
blanket conversion of double constants to float.

The SpeedyBee F405 Mini AP image preserves the F405's 1 MiB flash layout and
16 KiB configuration sector, with a 6 KiB main stack in CCM and a separate
6 KiB worker stack in SRAM.
Other F4 builds keep their existing stack reservation. Its ICM42688P uses the
existing receipt-timed IMU input path; BMI270 sample-history, raw cumulative
sample diagnostics and internal saturation observations are unavailable.
For `SPEEDYBEEF405MINI`, `mk/ap_autonomy.mk` enables `USE_AP_ICM42688P_4KHZ`,
selecting a 4 kHz ICM42688P gyro rate (250 us period). With `pid_process_denom = 1`,
the PID loop also runs at 4 kHz. This provides scheduling room for AP's 120 us
initial worker admission; the former 8 kHz configuration starved AP work.
Embedded aiding remains indoor flow/range; GPS/barometer delivery is not added
by selecting this board. DF_Sim's Air75 release/staging tool is G4-specific and
must not be used to package the F405 image.

The BMI270 publishes completed, timestamped register samples outside its live
DMA buffer. EKF gyro increments use calibrated, aligned rates before BF software
filters, with AP's trapezoidal/coning integration. Acceleration keeps its raw
sample integral, with the sensor period learned from acquisition counts and FC
timestamps. Angle and velocity increments carry separate durations; a missing
partner does not discard the other integral. Controller gyro filtering and the
20 Hz controller acceleration filter remain separate from these EKF inputs.

On G4, the 250 Hz adapter uses `-Os` to fit flash; raw gyro integration and
upstream estimator/controller kernels retain their existing optimization.
This is a code-size choice, not a measured deadline guarantee. Hardware timing
must be checked after flashing. Existing Blackbox `apInputDt` describes the
acceleration interval; it is no longer necessarily the gyro interval.

For disarmed hardware timing captures, add `USE_AP_PROFILE` (or pass `--profile`
to the AP release command). In the CLI, `ap_profile start` records 20 seconds;
wait without issuing other commands, then read `ap_profile`. It reports cycle
counts and duration histograms for the task, flow preprocessing, EKF input,
EKF update, output readback, and controller. The final histogram bucket (`0`)
means greater than 2048 us. Sections are inclusive: do not sum task and child
times. The empty probe measures timer overhead, not all nested bookkeeping.
Captures include interrupt time and stop before CLI output. Profiling is idle
until explicitly started and does not change estimator state or configuration.

The G4 AP linker map preserves the reset vector and configuration addresses,
enforces the physical 512 KiB flash boundary, places immutable magnetic tables
in spare space after the vector image, and reserves separate 6 KiB main and
cooperative-worker stacks. Data RAM
includes a bounded 12 KiB AP allocation arena. Link success does not measure
stack high-water or execution time.

## Hardware boundary and current limits

The G4 adapter currently selects indoor flow/range with a one-time disarmed
Betaflight yaw bootstrap, then EKF3's no-heading-sensor policy with the same
initialized local yaw reference. It uses calibrated acceleration before BF's
output low-pass filter and averages only fresh BMI270 samples. It retains the
existing MTF parser. The MTF clock is extended and mapped to the FC clock using
the minimum observed receipt offset, matching DF3's hardware timing approach;
fixed sensor/transport delay is not independently measured by this mapping.
Flow compensation waits for actual gyro coverage through the sample window.
CRSF reference bytes are copied atomically from the interrupt mailbox before
decoding. Initialization failure leaves autonomy unavailable and inhibits arming.

GPS/barometer/compass aiding remains exercised by the simulator and backend
tests; delivery from physical BF GPS/barometer drivers and automatic source
switching are not implemented by this hardware adapter. There is no claim that
local yaw supplies geographic north for compass-free outdoor navigation.

The G4 backend yields at cooperative checkpoints in prediction/fusion while
preserving each numerical update. The worker budgets and scheduler are unchanged
from 1.4.3. This port remains a development build: the current IMU and controller
changes require physical deadline, stack-watermark and sensor-timing validation.
Building alone does not update the firmware
version or SDK bundles. DF_Sim's release tool stages this backend explicitly
with `--backend ap --blackbox --version <version> --stage`; it audits physical
memory, preserves the existing profiles/radio images, updates the SDK catalog
and records the development qualification status. It never flashes hardware.

## Validation performed

On 2026-09-26, the Air75 II G4 target linked with the AP backend and Blackbox;
the original DF3 G4 target also rebuilt successfully. Native combined backend
tests and ASan/UBSan tests passed, covering both aiding modes, initialization,
allocation failure/reuse, no additional allocations after alignment, controller
reconstruction and bounded float diagnostics.

The statically linked SITL executable completed a 78-second indoor MATLAB
multi-waypoint mission, including takeoff, landing and disarm, without GPS or
compass packets. These are integration checks, not a 2 cm accuracy qualification.
DF_Sim artifacts are under `build/ap-autonomy/embedded-port/`; the final mission
records the executable hash and independent simulated ground truth.

The controller now receives a separate sensor-rate 20 Hz two-pole filtered
acceleration, with EKF body bias removed before navigation-frame rotation. Raw
EKF delta-velocity integration remains unchanged. The custom vertical-gap
fallback has been replaced by upstream vibration evidence and recovery; this
exposes a remaining range-only motor-bias qualification failure. This development
candidate is being packaged as the explicitly requested 1.4.7 diagnostic update;
staging does not qualify its flight behavior. See the workspace
`autopilot/INDOOR_QUALIFICATION.md` for current staging status and retained tests.
