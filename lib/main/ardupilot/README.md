# ArduPilot estimator and control sources

This is a dependency snapshot for the Betaflight AP autonomy backend. It contains
the upstream EKF3, data-abstraction layer, position/attitude controllers and their
math/filter dependencies. Shared translation units are listed only once in
`sources.mk`. Betaflight's hardware adapter and embedded runtime are separate in
`src/main/flight/ap_autonomy/`.

The snapshot comes from ArduPilot commit
`4c98c9221ad6933ab1c70fba8a3900ba2ff02f3b`. `manifest.json` records every copied
file's SHA-256 and the MAVLink/generator revisions. `local-changes.patch` preserves
the opt-in optical-flow acquisition timestamps, local yaw bootstrap and height
innovation diagnostics.
`embedded-checkpoints.patch` adds optional cooperative scheduling checkpoints;
it leaves estimator equations and their evaluation order unchanged.
`indoor-terrain.patch` adds the opt-in indoor terrain policy and vertical
diagnostic hooks, and separates external yaw support from external
position/velocity support. All patches are recorded in the manifest. The
selection and integration work were AI-assisted.

ArduPilot's license is retained verbatim in `COPYING.txt`; original source notices
remain in place. Generated MAVLink headers retain the accompanying
`MAVLINK_COPYING.txt` and `PYMAVLINK_COPYING.txt`, including the generator-output
license exception. The headers support types required by AP's public headers;
their presence does not enable a MAVLink transport in Betaflight.

AP_HAL's bounded stream formatter and its float conversion helper are also
included for embedded diagnostics, retaining their individual BSD notices.

## Build interface

Set `AP_VENDOR_ROOT` to this directory and include `sources.mk` for
`AP_VENDOR_SOURCES`. Include directories, after the backend's compatibility
headers, are:

```
$(AP_VENDOR_ROOT)/libraries
$(AP_VENDOR_ROOT)/libraries/AP_Common/missing
$(AP_VENDOR_ROOT)/generated
```

The backend must supply one consistent configuration/runtime for all sources.
Copying this snapshot alone does not establish MCU timing or flight readiness.

## Refresh and verify

Refresh from a DF_Sim checkout after building its `autopilot/ekf` and
`autopilot/control` libraries, so their compiler dependency files are current:

```
python3 lib/main/ardupilot/vendor.py --source-root /path/to/DF_Sim
python3 lib/main/ardupilot/vendor.py --verify
```

The extractor reads the actual source lists from both Makefiles and their `.d`
dependency closures, including the generated MAVLink headers. It refuses to
overwrite or delete locally edited snapshot files. An additional conditional
embedded dependency can be included explicitly with
`--extra-file libraries/path/to/header.h`; extra files are recorded in the
manifest and reused on subsequent refreshes. Extra `.cpp` files join the source
list. Verification needs no parent checkout, build products or network.

## Cooperative execution

With `USE_AP_WORKER`, the adapter supplies `apAutonomyCheckpoint()`. Checkpoints
separate generated covariance/flow temporary groups, covariance columns, matrix
rows, sequential observations, EKF phases and reset-history entries. A checkpoint
can suspend a private worker stack without exposing a partially updated estimate.
The adapter must freeze the DAL inputs until that transaction completes and
publish outputs only afterward. With the flag absent the calls inline to no-ops.

Checkpoint spacing targets short execution slices, but the bound must be checked
on the target MCU, including bootstrap, yaw resets, GPS and optical-flow fusion.
Source grouping alone does not establish a worst-case execution time. Refresh
reapplies the integration patches in manifest order with exact context; patch
failures require review.

## Indoor terrain integration

`EK3_FEATURE_DF_TERRAIN` defaults off. When enabled, range-primary height fusion
asks the backend policy whether a delayed sample describes vehicle height or a
surface transition. The policy owns classification history; the EKF retains the
single authoritative terrain offset. A separate frozen-bias IMU displacement
history supplies timestamp-aligned motion evidence for classification only.
It never supplies a synthetic navigation observation.

`EK3_FEATURE_EXTERNAL_YAW` defaults to `EK3_FEATURE_EXTERNAL_NAV`, retaining the
upstream combined configuration. The BF backend enables yaw alone because its
input contract carries a local yaw seed, optical flow/range and GPS/barometer,
but no external position or velocity. This avoids linking unused fusion code
and allocating its sensor buffers on the G4. Yaw fusion, status and recency
checks use the yaw feature consistently.
