# Lumenier LUX F765 + RP1 + MTF02

Build the board config kept in this fork:

```sh
make CONFIG=LUXNDAAF7 CONFIG_DIR=. OPTIONS=USE_DF3 -j8
```

This uses the STM32F765 2 MB flash / 512 KB RAM layout. The reset defaults
preserve the supplied hardware mappings; the NimbusOS profile selects GPS on
UART3, SmartAudio on UART4, MTF02 on UART5, and the RP1 on UART6. AUX3 selects
DF3 Flight Assist. DF3 Blackbox/profiling remain off unless explicitly requested.

The manufacturer's manual identifies the F765 and 8 MHz oscillator/sensor
wiring is independently documented in ArduPilot's `LumenierLUXF765-NDAA/hwdef.dat`.
The aircraft's full Betaflight dump supplied on 2026-09-22 defines the timer/DMA
mapping and its CW270 gyro orientation. This target is reconstructed from those
sources; it is not an upstream Betaflight target download.

Apply NimbusOS's `betaflight/df3/luxndaaf7/config.txt` after flashing and perform
collective calibration for this aircraft before enabling DF3. It contains no
Air75 hover calibration. Hardware bring-up and flight tuning remain to be validated.
