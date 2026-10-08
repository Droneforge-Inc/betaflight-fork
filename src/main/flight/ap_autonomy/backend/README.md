# Embedded AP backend

This directory adapts the vendored EKF3 and position/attitude controllers to
Betaflight. Both share one static build, AP_Param implementation, math library,
HAL and feature configuration. No host shared libraries or files outside this
Betaflight checkout are required. The algorithms remain in
`lib/main/ardupilot`; its manifest records provenance and upstream modifications.
`ap_terrain.cpp` supplies the port-specific range-primary terrain policy through
explicit opt-in hooks in that snapshot.

`sources.mk` provides the sources, include order and C++ options. Compile
`allocator.cpp` with `AP_BACKEND_ALLOCATOR_IMPL=1`; the forced `config.h` routes
every other AP translation unit's allocation expressions to the backend arena.
Betaflight C files retain their existing compiler options and allocation rules.
Do not apply fast math or single-precision literal conversion to the AP build.
The shared configuration declares the G4's 512 KiB flash capacity, selecting
upstream's float position type in hardware and host validation. The empty HAL
board's 2 MiB default would instead select software double position arithmetic
on the G4. Coordinates are local metres; float spacing remains below 1 mm at
10 km from the origin. Global latitude/longitude retain their integer encoding.

## Runtime contracts

- Betaflight implements `ap_runtime_time_us()` with its monotonic hardware clock
  and `ap_runtime_panic()` with a non-returning fatal path. Sensor timestamps and
  this clock must share their origin; elapsed time is not derived from CPU speed.
- All entry points run on one cooperative scheduler thread. The compatibility
  semaphore is deliberately nonblocking and provides no protection for callers
  in interrupts or concurrent threads.
- Initialization explicitly constructs HAL objects. On ARM it also invokes the
  linked `.init_array` once, because the G4 startup omits C++ initialization.
  The G4 linker must retain `__init_array_start` / `__init_array_end`.
- EKF sensor buffers and objects use a fixed 12 KiB, zero-filling arena. The
  reported available memory is the largest actual free block. Allocation
  failure returns null; invalid frees enter the fatal path. The controller uses
  fixed placement storage and can be reset without heap churn. The EKF instance
  persists for the boot lifetime, including disarmed filter reinitialization.
- Upstream parameter layouts/defaults are retained. Betaflight owns persistent
  settings; AP parameter `save` does not write another EEPROM format.
- Diagnostics use ArduPilot's bounded stream formatter, including floating-point
  fields in GPS prearm checks. They do not depend on newlib-nano's optional float
  formatter or its heap. Truncation preserves a null terminator and the full
  required output length.
- The only AHRS view is the bridge's estimator snapshot. The full AP AHRS
  frontend is disabled, preventing incompatible `AP::ahrs()` definitions in
  the estimator and controller object files.
- External yaw is compiled for the initial local heading; unused external
  position/velocity fusion is disabled. GPS/compass and flow/range retain their
  existing input contracts and fusion paths.

## Terrain policy

The policy compares delayed projected-range change with an IMU displacement
reference that is isolated from height-fusion velocity/bias corrections. Coherent
unexplained range change suspends height fusion. Confirmed transitions update
the terrain offset once per unique range timestamp; they never also update
vehicle height. Stable surfaces permit ordinary height fusion again. Raw slant
range remains available for optical-flow geometry.

Ambiguity expires after 1.5 seconds without renewing the deadline. A confirmed
step can reacquire on its plateau; an unconfirmed long slope cannot establish a
new room-height datum merely because its range has stopped changing. Missing
range followed by a changed surface also remains unqualified. The bridge keeps
raw freshness checks and propagates expiry into its existing navigation fault
path. These rules are conditional terrain handling, not absolute altitude
observability from range and IMU alone.

During coasting, vertical position correction and hover learning are suspended.
Requested velocity/acceleration and their stabilizing controller paths remain
active. The ordinary height pass/reset timestamp is never refreshed by a
terrain-only observation. Absorbed terrain offsets are correlated with the
vehicle estimate; this policy does not claim a calibrated absolute-height bound.

## Validation

Run `make -C src/main/flight/ap_autonomy/backend test` from the Betaflight root.
It links both APIs into one executable and runs 70-second vertical plants with
optical flow/local yaw and with GPS/compass. It checks estimator health,
controller response, one-metre hold, allocator overflow/reuse/coalescing,
controller reconstruction, and no additional allocations after alignment.
It also checks motor-dependent IMU bias, noisy height, degraded vertical control,
zero thrust while disarmed, mixed integer/float diagnostics, truncation and
zero-length output. Separate heading tests cover positive and negative local
origins, ground rotation, the armed-bootstrap guard and missing heading.
Terrain tests cover steps and drop-offs to 2 m, real vertical maneuvers, slopes,
motor bias, repeated/out-of-order timestamps, and missing/invalid measurements.
Controller tests verify repeated initialization, position-term suspension,
preserved velocity/acceleration commands and hover-learning freeze/recovery.

These deterministic host tests validate linkage and behavior, not hardware
timing or flight accuracy. The embedded worker cooperatively suspends EKF
covariance and fusion work; checkpoint tests compare its numerical results to
uninterrupted execution. Worst-case G4 execution time, interruption budget, stack watermark,
and sensor timing must be measured before enabling a flight release. A successful
link alone does not establish parity with the previous DF3 scheduler.
