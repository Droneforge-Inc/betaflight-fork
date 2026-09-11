/* DF_Sim host-only atomic protocol. Little-endian Linux ABI, v1 and opt-in v2. */
#pragma once
#include <stdint.h>

#define DFSIM_INPUT_MAGIC  0x31494644u /* DFI1 */
#define DFSIM_OUTPUT_MAGIC 0x314f4644u /* DFO1 */
#define DFSIM_INPUT_V2_MAGIC  0x32494644u /* DFI2: opt-in 1S LiHV voltage */
#define DFSIM_OUTPUT_V2_MAGIC 0x324f4644u /* DFO2 */
#define DFSIM_INPUT_V3_MAGIC  0x33494644u /* DFI3: timestamped CRSF UART, no direct RC */
#define DFSIM_OUTPUT_V3_MAGIC 0x334f4644u /* DFO3: same output layout as v2 */
#define DFSIM_INPUT_V4_MAGIC  0x34494644u /* DFI4: direct/CRSF RC + native MTF UART */
#define DFSIM_OUTPUT_V4_MAGIC 0x344f4644u /* DFO4: v2 plus processed range */
#define DFSIM_INPUT_V5_MAGIC  0x35494644u /* DFI5: v4 layout, activates native optical flow */
#define DFSIM_OUTPUT_V5_MAGIC 0x354f4644u /* DFO5: v4 plus native raw/aligned flow */
#define DFSIM_MAX_UART_EVENTS 128
#define DFSIM_RC_FRESH 1u
#define DFSIM_RC_SERIAL 2u           /* v4 only: v3 UART controls RC, even if count=0 */
#define DFSIM_TOF_FRAME_BYTES 27u

typedef struct {
    uint32_t magic, sequence;
    uint64_t sampleUs;
    uint32_t stepUs, flags;
    double gyroFRD[3];             /* measured body FRD rad/s */
    double specificForceFRD[3];    /* conventional specific force, m/s^2 */
    double pressurePa;
    uint16_t channels[16];         /* AETR then AUX1..12, microseconds */
} dfsim_input_t;

typedef struct {
    uint32_t magic, sequence;
    uint64_t sampleUs, endUs, firmwareUs;
    uint32_t status, armed, armingDisableFlags, flightModeFlags;
    float motors[4];               /* normalized PWM, Betaflight M1..M4 */
    float gyroBF[3];               /* firmware filtered rates, deg/s */
    float accelBF[3];              /* firmware accelerometer, m/s^2 */
    int16_t attitude[3];           /* firmware roll/pitch/yaw, 0.1 deg */
    uint16_t baroReady;
    float baroPressurePa, baroAltitudeCm;
    uint32_t gyroPeriodUs, pidPeriodUs;
} dfsim_output_t;

typedef struct {
    dfsim_input_t base;
    double batteryVoltageV;       /* 0..4.4 V, zero means disconnected, 1S only */
} dfsim_input_v2_t;

typedef struct {
    uint32_t offsetUs;             /* relative to sampleUs, in [0, stepUs) */
    uint8_t value, reserved[3];
} dfsim_uart_event_t;

typedef struct {
    dfsim_input_v2_t battery;       /* flags/channels must be zero in v3 */
    uint32_t count, reserved;
    dfsim_uart_event_t uart[DFSIM_MAX_UART_EVENTS];
} dfsim_input_v3_t;

typedef struct {
    dfsim_input_v3_t serial;          /* v4 flags select direct RC or CRSF UART */
    uint32_t tofFrameLength;         /* 0, or one complete 27-byte MTF02P frame */
    uint32_t reserved;
    uint8_t tofFrame[32];            /* unused bytes must be zero */
} dfsim_input_v4_t;

typedef dfsim_input_v4_t dfsim_input_v5_t;

typedef struct {
    dfsim_output_t base;
    uint16_t batteryFilteredCV, batteryLatestCV; /* native BF units: 0.01 V */
    uint8_t batteryCells, batteryState, batterySource, reserved;
} dfsim_output_v2_t;

typedef struct {
    dfsim_output_v2_t battery;
    int32_t rangeRawCm;              /* native median-filtered slant range */
    int32_t rangeCorrectedCm;        /* native estimate after tilt correction */
    uint32_t rangeFrameSequence;     /* sensor messages accepted by baseline parser */
    uint32_t rangeFrameAgeMs;        /* since accepted sensor message; UINT32_MAX before first */
    float rangeCosTilt;              /* firmware estimate used for latest correction */
    uint8_t rangeHealthy, rangeStatus, rangeStrength, rangePrecision; /* unmodified baseline processing */
} dfsim_output_v4_t;

typedef struct {
    dfsim_output_v4_t range;
    int16_t flowRawX, flowRawY;       /* latest decoded MICOLINK values, cm/s @ 1m */
    int16_t flowAlignedX, flowAlignedY; /* native Y flip and configured alignment */
    uint32_t flowFrameSequence;      /* sensor messages accepted by baseline parser */
    uint32_t flowFrameAgeMs;         /* since accepted sensor message; UINT32_MAX before first */
    uint8_t flowHealthy, flowStatus, flowQuality, reserved; /* baseline health may stay true on stale UART */
    uint32_t flowProcessedSequence;  /* latest UART sequence observed by processing; cached callbacks repeat it */
} dfsim_output_v5_t;

_Static_assert(sizeof(dfsim_input_t) == 112, "DFSim input ABI mismatch");
_Static_assert(sizeof(dfsim_output_t) == 112, "DFSim output ABI mismatch");
_Static_assert(sizeof(dfsim_input_v2_t) == 120, "DFSim v2 input ABI mismatch");
_Static_assert(sizeof(dfsim_output_v2_t) == 120, "DFSim v2 output ABI mismatch");
_Static_assert(sizeof(dfsim_input_v3_t) == 1152, "DFSim v3 input ABI mismatch");
_Static_assert(sizeof(dfsim_input_v4_t) == 1192, "DFSim v4 input ABI mismatch");
_Static_assert(sizeof(dfsim_output_v4_t) == 144, "DFSim v4 output ABI mismatch");
_Static_assert(sizeof(dfsim_input_v5_t) == 1192, "DFSim v5 input ABI mismatch");
_Static_assert(sizeof(dfsim_output_v5_t) == 168, "DFSim v5 output ABI mismatch");
