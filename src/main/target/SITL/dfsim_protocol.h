/* DF_Sim host-only atomic protocol. Little-endian Linux ABI, v1 and opt-in v2. */
#pragma once
#include <stdint.h>

#define DFSIM_INPUT_MAGIC  0x31494644u /* DFI1 */
#define DFSIM_OUTPUT_MAGIC 0x314f4644u /* DFO1 */
#define DFSIM_INPUT_V2_MAGIC  0x32494644u /* DFI2: opt-in 1S LiHV voltage */
#define DFSIM_OUTPUT_V2_MAGIC 0x324f4644u /* DFO2 */
#define DFSIM_INPUT_V3_MAGIC  0x33494644u /* DFI3: timestamped CRSF UART, no direct RC */
#define DFSIM_OUTPUT_V3_MAGIC 0x334f4644u /* DFO3: same output layout as v2 */
#define DFSIM_MAX_UART_EVENTS 128
#define DFSIM_RC_FRESH 1u

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
    dfsim_output_t base;
    uint16_t batteryFilteredCV, batteryLatestCV; /* native BF units: 0.01 V */
    uint8_t batteryCells, batteryState, batterySource, reserved;
} dfsim_output_v2_t;

_Static_assert(sizeof(dfsim_input_t) == 112, "DFSim input ABI mismatch");
_Static_assert(sizeof(dfsim_output_t) == 112, "DFSim output ABI mismatch");
_Static_assert(sizeof(dfsim_input_v2_t) == 120, "DFSim v2 input ABI mismatch");
_Static_assert(sizeof(dfsim_output_v2_t) == 120, "DFSim v2 output ABI mismatch");
_Static_assert(sizeof(dfsim_input_v3_t) == 1152, "DFSim v3 input ABI mismatch");
