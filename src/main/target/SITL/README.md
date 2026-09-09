# Standalone Droneforge Betaflight SITL

This target builds the firmware in this repository as a native Linux executable.
It does not link DFSim, SimITL, Jolt, or any motor/propeller/rigid-body physics.
An external simulator owns physical state, sensor generation, and transport models.

## Build and verify

From the repository root:

```sh
make TARGET=SITL -j8
python src/main/target/SITL/smoke_test.py obj/main/betaflight_SITL.elf
```

Native GCC, Make, binutils and OpenSSL (build hash command) are required. No ARM
SDK or config submodule is used by this target. Run each instance in a separate
working directory: its 32 KiB `eeprom.bin` is created there.

```sh
mkdir -p ../runs/baseline
cd ../runs/baseline
../../betaflight-fork/obj/main/betaflight_SITL.elf
```

Optional first argument: IPv4 destination for motor packets (default 127.0.0.1).
The executable uses fixed ports; do not run two instances simultaneously.

## Atomic DF_Sim interface (used by the MATLAB main model)

UDP 9003 also accepts the versioned 112-byte request in `dfsim_protocol.h`;
the 112-byte reply returns from that socket to the requesting loopback peer.
Little-endian x86-64 layouts (Python `struct` notation):

```
request: <IIQII7d16H
reply:   <IIQQQIIII4f3f3f3hHffII
```

Request: magic `0x31494644`, sequence (starts 1), sampleUs (starts 0), stepUs
(1–10000), flags (bit 0 RC_FRESH), body-FRD gyro[3] rad/s, conventional specific
force[3] m/s², pressure Pa, channels[16] AETR then AUX in 750–2250 µs.
The first request must contain fresh RC. Both inertial vectors convert to
firmware axes with `diag(1,-1,-1)`, then to int16 virtual-device counts.
This correct proper-rotation mapping differs from the historical legacy mapping.

Reply: magic `0x314f4644`, sequence, sampleUs, endUs, firmwareUs, status, armed,
armingDisableFlags, flightModeFlags; normalized motors[4] **M1..M4**; filtered
gyroBF[3] deg/s, accelBF[3] m/s²; attitude[3] in 0.1 deg, baroReady;
pressure Pa, altitude cm, configured gyro/PID periods µs. Native telemetry axes
are preserved. The protocol has no vehicle state or ground-truth attitude input.

Each accepted request installs fresh RC if flagged and pressure, holds its IMU
sample available while advancing by stepUs with scheduler increments at most
25 µs, then replies with the completed outputs. Input sampleUs must equal the
previous endUs. Firmware boot/blocking-delay offsets remain visible in firmwareUs.
No packets means no experiment-time advancement; MCU task execution costs are
not emulated. Faster firmware tasks reread held external sensor samples.

The first accepted transport mode claims the process; do not mix atomic and
legacy messages. Atomic mode additionally owns one loopback peer/port. Exact
retransmission of the last request returns the cached reply without advancing.
Invalid sequence/time/values/flags are rejected without changing state; statuses
1/2/3/4 denote those errors, 5 legacy-mode conflict, 6 peer mismatch. Malformed
lengths/magic and non-loopback atomic requests receive no reply.

The MATLAB `dfsim.SITLBridge` owns startup, sequence checking and error cleanup.
The separate `BetaflightLoop` System block returns cached current-boundary
outputs and advances firmware in its update phase, matching the held-command
plant step. See the enclosing `matlab/CLOSED_LOOP.md` for pre-roll, RC cadence,
sampling approximation and sensor configuration.

`tools/test_atomic_sitl.py` in DF_Sim checks exact duplicate handling, rejected
packets, real RC arm/disarm, negative feedback on all body axes, active pressure
delivery and RC-loss DROP failsafe with the AIR75 seed. No arming bypass or
hardware controller retuning is introduced. EEPROM layout is unchanged.

### Optional v2 battery voltage

DFI2 (`0x32494644`) uses the same input layout plus one little-endian double
terminal voltage, for 120 bytes. It is explicitly a 1S LiHV interface: 0..4.4 V,
with zero meaning disconnected. DFO2 (`0x324f4644`) appends `<HHBBBB>` to the
v1 response: filtered/latest centivolts, cell count, battery state, voltage
source and zero reserved byte. V1 remains available; do not switch versions
within a process. A version conflict returns status 4.

The first valid v2 request enables simulated ADC sensing and the normal battery
tasks in RAM, forces one cell and sets maximum cell detection voltage to 4.40 V.
No EEPROM is written, and PID voltage compensation is not enabled automatically.
The simulated voltage is quantized through the configured ADC divider and then
processed by native Betaflight filters. Current and consumed-mAh telemetry retain
their existing configured source; modeled current/SOC are not injected here.
See `matlab/BATTERY.md` in DF_Sim for the battery equations, provisional parameters
and opt-in model. `tools/test_battery_sitl.py` verifies v2 behavior.

## Legacy firmware boundary (preserved for existing tools)

- UDP 9003 input: 144-byte `fdm_packet`, 18 little-endian doubles on x86-64.
  Fields: timestamp seconds; angular rates[3] rad/s; specific force[3] m/s²;
  quaternion[4]; velocity[3]; position[3]; pressure Pa.
- UDP 9004 input: 40-byte `rc_packet`: double timestamp and 16 uint16 channels.
  AETR order, nominal 1000–2000. Deliver an RC packet before the corresponding
  state packet; received RC is applied at the completed state boundary.
- UDP 9002 output: four float32 normalized PWM commands in legacy Gazebo order:
  Betaflight motor indices [1,2,3,0]. These are commands, not RPM or thrust.
- UDP 9001 output: native `servo_packet_raw` layout: uint16 motor count, two
  padding bytes, then 16 float32 raw PWM values in Betaflight motor order.
- TCP 5761: UART1 MSP/CLI. Other configured UARTs use 5760 + UART number.
  Firmware serial processing requires simulation time to advance.

The historical input mapping is explicit: accelerometer counts = -specificForce
×256/9.80665 on all axes; gyro counts = [p,-q,-r] ×16.4×180/pi.
Level stationary input is [0,0,-9.80665] specific force. Do not assume these
wire coordinates match Nimbus or the new atomic protocol directly.
Quaternion, velocity, and position fields are retained for packet compatibility
but are not injected into the attitude estimator or used for dynamics.
Betaflight estimates attitude from its virtual gyro/accelerometer. Pressure
feeds the virtual barometer. Optical flow/range and GPS injection are not added.

## Legacy time semantics

The first timestamp should be zero. It is elapsed experiment time, independent
of the internal firmware boot-time offset. Subsequent timestamps must increase;
gaps above 100 ms, backward timestamps, invalid IMU values, invalid pressure,
and malformed packet lengths are rejected without a reply.

For each accepted state packet, the firmware advances to that timestamp with
the PREVIOUSLY delivered sensor and RC values. Scheduler increments are at
most 50 microseconds, including a final shorter remainder. Then it installs
the new sensor sample and available RC input and returns current motor commands.
Send one state packet and wait for its reply before sending the next.

With no state packets, firmware experiment time does not advance. Internal
blocking firmware delays advance virtual time and can add an offset; these must
be accounted for before calibrated timing experiments. The scheduler's
host-specific branch does not spin waiting for virtual CPU time. Task execution
cost on an MCU is not modeled.

This is a compatibility interface, not validated hardware timing equivalence.
RC and MSP remain separate legacy streams. New main-model experiments use the
atomic sequence-tagged interface above; do not adapt its ordering to this one.

## Configuration and limitations

Fresh EEPROM selects virtual PWM, synchronized output, and a single virtual IMU.
It is a generic baseline, NOT the Air75 configuration. No aircraft PID/rate/AUX
dump has been imported. DShot, RPM/ESC feedback, motor dynamics, ELRS scheduling,
and sensor impairments remain external or future interface work. Do not load
a DShot aircraft dump unchanged into this PWM-only baseline.

The enclosing DF_Sim project now provides a separate partial AIR75 V2 seed in
`configs/air75_v2`, generated by `tools/prepare_air75_v2.py`. This does not change
fresh-EEPROM defaults. Its report lists unavailable filters and all adaptations.
SITL virtual PWM bypasses the physical analog pulse-rate restriction in config
validation so the imported PID divider is not silently increased.

The smoke test runs firmware twice in an isolated temporary directory. It checks
MSP version, level acceleration/attitude, nonzero gyro signs, disarmed output,
individual motor-test output mapping, and EEPROM save/reload. It does not prove
arming, closed-loop stability, dynamics accuracy, or flight-configuration parity.

## Provenance

Restored platform files from this fork's own history:
`b31096084^`, before removal of the original `src/main/target/SITL`.
Production firmware remains the current fork checkout; only host build/platform
adaptations and narrowly guarded entry/scheduler hooks were added.

Changes include native toolchain selection, ELF default target, host compiler
compatibility, writable parameter-group linker placement, sensor startup order,
single-thread sensor delivery, external timestamp stepping, correct cycle units,
and virtual PWM defaults. No code from /mnt/ml-data/DFSim is needed to build.
