/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <errno.h>
#include <time.h>

#include "common/maths.h"

#include "drivers/io.h"
#include "drivers/adc.h"
#include "drivers/dma.h"
#include "drivers/motor.h"
#include "drivers/serial.h"
#include "drivers/serial_tcp.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/pwm_output.h"
#include "drivers/light_led.h"

#include "drivers/timer.h"
#include "timer_def.h"

#include "drivers/accgyro/accgyro_virtual.h"
#include "drivers/barometer/barometer_virtual.h"
#include "flight/imu.h"
#include "fc/runtime_config.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/voltage.h"
#include "sensors/adcinternal.h"

#include "config/feature.h"
#include "config/config.h"
#include "scheduler/scheduler.h"

#include "pg/rx.h"
#include "pg/motor.h"

#include "rx/rx.h"

#include "dyad.h"
#include "target/SITL/udplink.h"
#include "target/SITL/dfsim_protocol.h"

uint32_t SystemCoreClock;

// There are no physical GPIO pins on the host target.
void IOHi(IO_t io) { UNUSED(io); }
void IOLo(IO_t io) { UNUSED(io); }

void targetConfiguration(void)
{
    // Explicit virtual PWM baseline; real-aircraft configuration is separate.
    motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_STANDARD;
    motorConfigMutable()->dev.useUnsyncedPwm = false;
}

static fdm_packet fdmPkt;
static rc_packet rcPkt;
static servo_packet pwmPkt;
static servo_packet_raw pwmRawPkt;

static bool rcFramePending;
static uint64_t elapsedUs;
static unsigned transportMode; /* 0 unclaimed, 1 legacy, 2 DFSim atomic */
static dfsim_input_t lastInput;
static dfsim_output_v2_t lastOutput;
static bool atomicBatteryEnabled;
static double lastBatteryVoltage;
static double simulatedBatteryVoltage;
static struct sockaddr_in atomicPeer;

static struct timespec start_time;
static uint64_t virtualTimeUs;
static pthread_t tcpWorker;
static bool workerRunning = true;
static udpLink_t stateLink, pwmLink, pwmRawLink, rcLink;

static pthread_mutex_t mainLoopLock;
static char simulator_ip[32] = "127.0.0.1";

#define PORT_PWM_RAW    9001    // Out
#define PORT_PWM        9002    // Out
#define PORT_STATE      9003    // In
#define PORT_RC         9004    // In

int targetParseArgs(int argc, char * argv[])
{
    //The first argument should be target IP.
    if (argc > 1) {
        if (inet_pton(AF_INET, argv[1], &(struct in_addr){0}) != 1) {
            fprintf(stderr, "Expected an IPv4 simulator address\n");
            exit(2);
        }
        snprintf(simulator_ip, sizeof(simulator_ip), "%s", argv[1]);
    }

    printf("[SITL] The SITL will output to IP %s:%d (Gazebo) and %s:%d (RealFlightBridge)\n",
           simulator_ip, PORT_PWM, simulator_ip, PORT_PWM_RAW);
    return 0;
}

int timeval_sub(struct timespec *result, struct timespec *x, struct timespec *y);

int lockMainPID(void)
{
    return pthread_mutex_trylock(&mainLoopLock);
}

#define RAD2DEG (180.0 / M_PI)
#define ACC_SCALE (256 / 9.80665)
#define GYRO_SCALE (16.4)

void sendMotorUpdate(void)
{
    udpSend(&pwmLink, &pwmPkt, sizeof(servo_packet));
}

void updateState(const fdm_packet* pkt)
{
    int16_t x,y,z;
    x = constrain(-pkt->imu_linear_acceleration_xyz[0] * ACC_SCALE, -32767, 32767);
    y = constrain(-pkt->imu_linear_acceleration_xyz[1] * ACC_SCALE, -32767, 32767);
    z = constrain(-pkt->imu_linear_acceleration_xyz[2] * ACC_SCALE, -32767, 32767);
    virtualAccSet(virtualAccDev, x, y, z);
//    printf("[acc]%lf,%lf,%lf\n", pkt->imu_linear_acceleration_xyz[0], pkt->imu_linear_acceleration_xyz[1], pkt->imu_linear_acceleration_xyz[2]);

    x = constrain(pkt->imu_angular_velocity_rpy[0] * GYRO_SCALE * RAD2DEG, -32767, 32767);
    y = constrain(-pkt->imu_angular_velocity_rpy[1] * GYRO_SCALE * RAD2DEG, -32767, 32767);
    z = constrain(-pkt->imu_angular_velocity_rpy[2] * GYRO_SCALE * RAD2DEG, -32767, 32767);
    virtualGyroSet(virtualGyroDev, x, y, z);
//    printf("[gyr]%lf,%lf,%lf\n", pkt->imu_angular_velocity_rpy[0], pkt->imu_angular_velocity_rpy[1], pkt->imu_angular_velocity_rpy[2]);

    // temperature in 0.01 C = 25 deg
    virtualBaroSet(pkt->pressure, 2500);
#if !defined(USE_IMU_CALC)
#if defined(SET_IMU_FROM_EULER)
    // set from Euler
    double qw = pkt->imu_orientation_quat[0];
    double qx = pkt->imu_orientation_quat[1];
    double qy = pkt->imu_orientation_quat[2];
    double qz = pkt->imu_orientation_quat[3];
    double ysqr = qy * qy;
    double xf, yf, zf;

    // roll (x-axis rotation)
    double t0 = +2.0 * (qw * qx + qy * qz);
    double t1 = +1.0 - 2.0 * (qx * qx + ysqr);
    xf = atan2(t0, t1) * RAD2DEG;

    // pitch (y-axis rotation)
    double t2 = +2.0 * (qw * qy - qz * qx);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    yf = asin(t2) * RAD2DEG; // from wiki

    // yaw (z-axis rotation)
    double t3 = +2.0 * (qw * qz + qx * qy);
    double t4 = +1.0 - 2.0 * (ysqr + qz * qz);
    zf = atan2(t3, t4) * RAD2DEG;
    imuSetAttitudeRPY(xf, -yf, zf); // yes! pitch was inverted!!
#else
    imuSetAttitudeQuat(pkt->imu_orientation_quat[0], pkt->imu_orientation_quat[1], pkt->imu_orientation_quat[2], pkt->imu_orientation_quat[3]);
#endif
#endif

}

static float readRCSITL(const rxRuntimeState_t *rxRuntimeState, uint8_t channel)
{
    UNUSED(rxRuntimeState);
    return rcPkt.channels[channel];
}

static uint8_t rxRCFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);
    if (!rcFramePending) {
        return RX_FRAME_PENDING;
    }
    rcFramePending = false;
    return RX_FRAME_COMPLETE;
}

static void installRC(const uint16_t *channels)
{
    memmove(rcPkt.channels, channels, sizeof(rcPkt.channels));
    rxRuntimeState.channelCount = SIMULATOR_MAX_RC_CHANNELS;
    rxRuntimeState.rcReadRawFn = readRCSITL;
    rxRuntimeState.rcFrameStatusFn = rxRCFrameStatus;
    rxRuntimeState.rxProvider = RX_PROVIDER_UDP;
    rxRuntimeState.lastRcFrameTimeUs = micros();
    rcFramePending = true;
}

static void installAtomicIMU(const dfsim_input_t *input)
{
    /* FRD -> firmware's forward/left/up sensor frame; proper 180-deg X rotation.
     * Unlike the historical wire adapter, this applies the same frame to both
     * angular rate and conventional specific force. No ground-truth attitude. */
    const double signs[3] = { 1, -1, -1 };
    int16_t a[3], w[3];
    for (unsigned i = 0; i < 3; i++) {
        a[i] = constrain(input->specificForceFRD[i] * signs[i] * ACC_SCALE, -32767, 32767);
        w[i] = constrain(input->gyroFRD[i] * signs[i] * GYRO_SCALE * RAD2DEG, -32767, 32767);
    }
    virtualAccSet(virtualAccDev, a[0], a[1], a[2]);
    virtualGyroSet(virtualGyroDev, w[0], w[1], w[2]);
}

static void replyAtomic(const dfsim_output_v2_t *output, bool withBattery)
{
    sendto(stateLink.fd, output, withBattery ? sizeof(*output) : sizeof(output->base), 0,
        (struct sockaddr *)&stateLink.recv, sizeof(stateLink.recv));
}

static void pollAtomic(const dfsim_input_t *input, bool withBattery, double batteryVoltage)
{
    dfsim_output_v2_t out = { .base = { .magic = withBattery ? DFSIM_OUTPUT_V2_MAGIC : DFSIM_OUTPUT_MAGIC,
        .sequence = input->sequence, .sampleUs = input->sampleUs,
        .endUs = elapsedUs, .firmwareUs = virtualTimeUs } };
    /* Error replies never advance time or replace inputs. A new process resets. */
    if (transportMode == 1) { out.base.status = 5; replyAtomic(&out, withBattery); return; }
    if (transportMode == 2 && (atomicPeer.sin_addr.s_addr != stateLink.recv.sin_addr.s_addr ||
            atomicPeer.sin_port != stateLink.recv.sin_port)) {
        out.base.status = 6; replyAtomic(&out, withBattery); return;
    }
    if (transportMode == 2 && input->sequence == lastInput.sequence) {
        if (memcmp(input, &lastInput, sizeof(*input)) == 0 &&
                (!withBattery || memcmp(&batteryVoltage, &lastBatteryVoltage, sizeof(double)) == 0)) { replyAtomic(&lastOutput, withBattery); return; }
        out.base.status = 2; replyAtomic(&out, withBattery); return;
    }
    if (transportMode == 2 && withBattery != atomicBatteryEnabled) {
        out.base.status = 4; replyAtomic(&out, withBattery); return;
    }
    if (input->sequence != (transportMode == 2 ? lastInput.sequence + 1 : 1)) { out.base.status = 2; }
    else if (input->sampleUs != elapsedUs || input->stepUs == 0 || input->stepUs > 10000 ||
            input->sampleUs + input->stepUs > 86400000000ULL) { out.base.status = 1; }
    else if ((input->flags & ~DFSIM_RC_FRESH) != 0 ||
            (transportMode == 0 && !(input->flags & DFSIM_RC_FRESH))) { out.base.status = 4; }
    else {
        for (unsigned i = 0; i < 3; i++) {
            if (!isfinite(input->gyroFRD[i]) || !isfinite(input->specificForceFRD[i])) { out.base.status = 3; }
        }
        if (!isfinite(input->pressurePa) || input->pressurePa <= 0 || input->pressurePa > 200000) { out.base.status = 3; }
        for (unsigned i = 0; i < 16; i++) {
            if (input->channels[i] < 750 || input->channels[i] > 2250) { out.base.status = 3; }
        }
    }
    if (withBattery && (!isfinite(batteryVoltage) || batteryVoltage < 0 || batteryVoltage > 4.4)) {
        out.base.status = 3;
    }
    if (out.base.status) { replyAtomic(&out, withBattery); return; }
    if (withBattery) {
        simulatedBatteryVoltage = batteryVoltage;
        if (!atomicBatteryEnabled) {
            /* Explicit, process-local 1S LiHV sensing. No EEPROM writes, no
             * change to PID voltage compensation or current-meter settings. */
            batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_ADC;
            batteryConfigMutable()->forceBatteryCellCount = 1;
            batteryConfigMutable()->vbatmaxcellvoltage = 440;
            batteryInit();
            setTaskEnabled(TASK_BATTERY_VOLTAGE, true);
            if (isSagCompensationConfigured()) {
                rescheduleTask(TASK_BATTERY_VOLTAGE, TASK_PERIOD_HZ(FAST_VOLTAGE_TASK_FREQ_HZ));
            }
            setTaskEnabled(TASK_BATTERY_ALERTS, true);
            atomicBatteryEnabled = true;
        }
    }
    transportMode = 2;
    atomicPeer = stateLink.recv;
    virtualBaroSet((int32_t)lrint(input->pressurePa), 2500);
    if (input->flags & DFSIM_RC_FRESH) { installRC(input->channels); }
    const uint64_t endUs = elapsedUs + input->stepUs;
    while (elapsedUs < endUs) {
        /* Held sensor values remain available to native gyro/acc tasks.
         * 25 us exactly subdivides the current 125/250 us gyro/PID periods.
         * This is virtual scheduling, not MCU execution-cost emulation. */
        const uint64_t tick = endUs - elapsedUs < 25 ? endUs - elapsedUs : 25;
        installAtomicIMU(input);
        virtualTimeUs += tick;
        elapsedUs += tick;
        scheduler();
    }
    out.base.endUs = elapsedUs;
    out.base.firmwareUs = virtualTimeUs;
    out.base.armed = !!ARMING_FLAG(ARMED);
    out.base.armingDisableFlags = getArmingDisableFlags();
    out.base.flightModeFlags = flightModeFlags;
    for (unsigned i = 0; i < 4; i++) {
        /* Read already-written virtual PWM commands, without the legacy shuffle. */
        out.base.motors[i] = pwmPkt.motor_speed[(i + 3) % 4];
    }
    for (unsigned i = 0; i < 3; i++) {
        out.base.gyroBF[i] = gyroGetFilteredDownsampled(i);
        out.base.accelBF[i] = acc.accADC[i] * 9.80665f / acc.dev.acc_1G;
        out.base.attitude[i] = attitude.raw[i];
    }
    out.base.baroReady = baroIsCalibrated();
    out.base.baroPressurePa = baro.pressure;
    out.base.baroAltitudeCm = getBaroAltitude();
    out.base.gyroPeriodUs = gyro.sampleLooptime;
    out.base.pidPeriodUs = gyro.targetLooptime;
    if (withBattery) {
        out.batteryFilteredCV = getBatteryVoltage();
        out.batteryLatestCV = getBatteryVoltageLatest();
        out.batteryCells = getBatteryCellCount();
        out.batteryState = getBatteryState();
        out.batterySource = batteryConfig()->voltageMeterSource;
    }
    lastInput = *input; lastOutput = out; lastBatteryVoltage = batteryVoltage;
    replyAtomic(&out, withBattery);
}

void targetPoll(void)
{
    union { fdm_packet legacy; dfsim_input_t atomic; dfsim_input_v2_t battery; } input;
    const int length = udpRecv(&stateLink, &input, sizeof(input), 100);
    if (length >= 4 && input.atomic.magic == DFSIM_INPUT_V2_MAGIC) {
        if (length == sizeof(dfsim_input_v2_t) && ntohl(stateLink.recv.sin_addr.s_addr) == INADDR_LOOPBACK) {
            pollAtomic(&input.battery.base, true, input.battery.batteryVoltageV);
        }
        return;
    }
    if (length >= 4 && input.atomic.magic == DFSIM_INPUT_MAGIC) {
        /* Atomic protocol is simulator-local only. */
        if (length == sizeof(dfsim_input_t) && ntohl(stateLink.recv.sin_addr.s_addr) == INADDR_LOOPBACK) {
            pollAtomic(&input.atomic, false, 0);
        }
        return;
    }
    if (length != sizeof(fdm_packet) || transportMode == 2) { return; }
    const fdm_packet next = input.legacy;
    if (!isfinite(next.timestamp) || next.timestamp < 0 || next.timestamp > 86400) {
        return;
    }
    const uint64_t requestedUs = (uint64_t)llround(next.timestamp * 1e6);
    for (unsigned i = 0; i < 3; ++i) {
        if (!isfinite(next.imu_angular_velocity_rpy[i]) ||
            !isfinite(next.imu_linear_acceleration_xyz[i])) {
            return;
        }
    }
    if (!isfinite(next.pressure) || next.pressure < 0 || next.pressure > 200000) {
        return;
    }
    if (requestedUs < elapsedUs || requestedUs - elapsedUs > 100000) {
        return; // Restart the process for a new timeline; do not silently retime.
    }
    transportMode = 1;
    // Evolve firmware with previously delivered IMU/RC, never future samples.
    while (elapsedUs < requestedUs) {
        const uint64_t tick = requestedUs - elapsedUs < 50 ? requestedUs - elapsedUs : 50;
        virtualTimeUs += tick;
        elapsedUs += tick;
        scheduler();
    }
    fdmPkt = next;
    updateState(&fdmPkt);
    if (udpRecv(&rcLink, &rcPkt, sizeof(rcPkt), 0) == sizeof(rcPkt)) {
        installRC(rcPkt.channels);
    }
    sendMotorUpdate();
    udpSend(&pwmRawLink, &pwmRawPkt, sizeof(pwmRawPkt));
}

static void* tcpThread(void* data)
{
    UNUSED(data);

    while (workerRunning) {
        dyad_update();
    }

    dyad_shutdown();
    printf("tcpThread end!!\n");
    return NULL;
}

// system
void systemInit(void)
{
    int ret;

    clock_gettime(CLOCK_MONOTONIC, &start_time);
    printf("[system]Init...\n");

    SystemCoreClock = 500 * 1e6; // virtual 500MHz

    if (pthread_mutex_init(&mainLoopLock, NULL) != 0) {
        printf("Create mainLoopLock error!\n");
        exit(1);
    }

    dyad_init();
    dyad_setTickInterval(0.2f);
    dyad_setUpdateTimeout(0.01f);

    ret = udpInit(&pwmLink, simulator_ip, PORT_PWM, false);
    printf("[SITL] init PwmOut UDP link to gazebo %s:%d...%d\n", simulator_ip, PORT_PWM, ret);

    ret = udpInit(&pwmRawLink, simulator_ip, PORT_PWM_RAW, false);
    printf("[SITL] init PwmOut UDP link to RF9 %s:%d...%d\n", simulator_ip, PORT_PWM_RAW, ret);

    ret = udpInit(&stateLink, NULL, PORT_STATE, true);
    printf("[SITL] start UDP server @%d...%d\n", PORT_STATE, ret);

    ret = udpInit(&rcLink, NULL, PORT_RC, true);
    printf("[SITL] start UDP server for RC input @%d...%d\n", PORT_RC, ret);

}

// Do not deliver packets until firmware has created its virtual sensor devices.
void targetReady(void)
{
    int ret = pthread_create(&tcpWorker, NULL, tcpThread, NULL);
    if (ret != 0) {
        fprintf(stderr, "Create tcpWorker error!\n");
        exit(1);
    }

}

void systemReset(void)
{
    printf("[system]Reset!\n");
    workerRunning = false;
    pthread_join(tcpWorker, NULL);
    exit(0);
}
void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    UNUSED(requestType);

    printf("[system]ResetToBootloader!\n");
    workerRunning = false;
    pthread_join(tcpWorker, NULL);
    exit(0);
}

void timerInit(void)
{
    printf("[timer]Init...\n");
}

void failureMode(failureMode_e mode)
{
    printf("[failureMode]!!! %d\n", mode);
    while (1);
}

void indicateFailure(failureMode_e mode, int repeatCount)
{
    UNUSED(repeatCount);
    printf("Failure LED flash for: [failureMode]!!! %d\n", mode);
}

// Time part
// Thanks ArduPilot
uint64_t nanos64_real(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec*1e9 + ts.tv_nsec) - (start_time.tv_sec*1e9 + start_time.tv_nsec);
}

uint64_t micros64_real(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e6*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec*1.0e-9)));
}

uint64_t millis64_real(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e3*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec*1.0e-9)));
}

uint64_t micros64(void)
{
    return virtualTimeUs;
}

uint64_t millis64(void)
{
    return virtualTimeUs / 1000;
}

uint32_t micros(void)
{
    return micros64() & 0xFFFFFFFF;
}

uint32_t millis(void)
{
    return millis64() & 0xFFFFFFFF;
}

int32_t clockCyclesToMicros(int32_t clockCycles)
{
    return clockCycles / 500;
}

int32_t clockCyclesTo10thMicros(int32_t clockCycles)
{
    return clockCycles / 50;
}

int32_t clockCyclesTo100thMicros(int32_t clockCycles)
{
    return clockCycles / 5;
}

uint32_t clockMicrosToCycles(uint32_t micros)
{
    return micros * 500U;
}
uint32_t getCycleCounter(void)
{
    return (uint32_t) (micros64() * 500U);
}

void microsleep(uint32_t usec)
{
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = usec*1000UL;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) ;
}

void delayMicroseconds(uint32_t us)
{
    virtualTimeUs += us;
}

void delayMicroseconds_real(uint32_t us)
{
    microsleep(us);
}

void delay(uint32_t ms)
{
    virtualTimeUs += (uint64_t)ms * 1000;
}

// Subtract the ‘struct timespec’ values X and Y,  storing the result in RESULT.
// Return 1 if the difference is negative, otherwise 0.
// result = x - y
// from: http://www.gnu.org/software/libc/manual/html_node/Elapsed-Time.html
int timeval_sub(struct timespec *result, struct timespec *x, struct timespec *y)
{
    unsigned int s_carry = 0;
    unsigned int ns_carry = 0;
    // Perform the carry for the later subtraction by updating y.
    if (x->tv_nsec < y->tv_nsec) {
        int nsec = (y->tv_nsec - x->tv_nsec) / 1000000000 + 1;
        ns_carry += 1000000000 * nsec;
        s_carry += nsec;
    }

    // Compute the time remaining to wait. tv_usec is certainly positive.
    result->tv_sec = x->tv_sec - y->tv_sec - s_carry;
    result->tv_nsec = x->tv_nsec - y->tv_nsec + ns_carry;

    // Return 1 if result is negative.
    return x->tv_sec < y->tv_sec;
}


// PWM part
pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];
static pwmOutputPort_t servos[MAX_SUPPORTED_SERVOS];

// real value to send
static int16_t motorsPwm[MAX_SUPPORTED_MOTORS];
static int16_t servosPwm[MAX_SUPPORTED_SERVOS];
static int16_t idlePulse;

void servoDevInit(const servoDevConfig_t *servoConfig)
{
    printf("[SITL] Init servos num %d rate %d center %d\n", MAX_SUPPORTED_SERVOS,
           servoConfig->servoPwmRate, servoConfig->servoCenterPulse);
    for (uint8_t servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
        servos[servoIndex].enabled = true;
    }
}

static motorDevice_t motorPwmDevice; // Forward

pwmOutputPort_t *pwmGetMotors(void)
{
    return motors;
}

static float pwmConvertFromExternal(uint16_t externalValue)
{
    return (float)externalValue;
}

static uint16_t pwmConvertToExternal(float motorValue)
{
    return (uint16_t)motorValue;
}

static void pwmDisableMotors(void)
{
    motorPwmDevice.enabled = false;
}

static bool pwmEnableMotors(void)
{
    motorPwmDevice.enabled = true;

    return true;
}

static void pwmWriteMotor(uint8_t index, float value)
{

    if (index < MAX_SUPPORTED_MOTORS) {
        motorsPwm[index] = value - idlePulse;
    }

    if (index < pwmRawPkt.motorCount) {
        pwmRawPkt.pwm_output_raw[index] = value;
    }

}

static void pwmWriteMotorInt(uint8_t index, uint16_t value)
{
    pwmWriteMotor(index, (float)value);
}

static void pwmShutdownPulsesForAllMotors(void)
{
    motorPwmDevice.enabled = false;
}

bool pwmIsMotorEnabled(uint8_t index)
{
    return motors[index].enabled;
}

static void pwmCompleteMotorUpdate(void)
{
    // send to simulator
    // for gazebo8 ArduCopterPlugin remap, normal range = [0.0, 1.0], 3D rang = [-1.0, 1.0]

    double outScale = 1000.0;
    if (featureIsEnabled(FEATURE_3D)) {
        outScale = 500.0;
    }

    pwmPkt.motor_speed[3] = motorsPwm[0] / outScale;
    pwmPkt.motor_speed[0] = motorsPwm[1] / outScale;
    pwmPkt.motor_speed[1] = motorsPwm[2] / outScale;
    pwmPkt.motor_speed[2] = motorsPwm[3] / outScale;

    // get one "fdm_packet" can only send one "servo_packet"!!
    // Delivery occurs once per accepted external timestamp in targetPoll().
//    printf("[pwm]%u:%u,%u,%u,%u\n", idlePulse, motorsPwm[0], motorsPwm[1], motorsPwm[2], motorsPwm[3]);

}

void pwmWriteServo(uint8_t index, float value)
{
    servosPwm[index] = value;
    if (index + pwmRawPkt.motorCount < SIMULATOR_MAX_PWM_CHANNELS) {
        // In pwmRawPkt, we put servo right after the motors.
        pwmRawPkt.pwm_output_raw[index + pwmRawPkt.motorCount] = value;
    }
}

static motorDevice_t motorPwmDevice = {
    .vTable = {
        .postInit = motorPostInitNull,
        .convertExternalToMotor = pwmConvertFromExternal,
        .convertMotorToExternal = pwmConvertToExternal,
        .enable = pwmEnableMotors,
        .disable = pwmDisableMotors,
        .isMotorEnabled = pwmIsMotorEnabled,
        .decodeTelemetry = motorDecodeTelemetryNull,
        .write = pwmWriteMotor,
        .writeInt = pwmWriteMotorInt,
        .updateComplete = pwmCompleteMotorUpdate,
        .shutdown = pwmShutdownPulsesForAllMotors,
    }
};

motorDevice_t *motorPwmDevInit(const motorDevConfig_t *motorConfig, uint16_t _idlePulse, uint8_t motorCount, bool useUnsyncedPwm)
{
    UNUSED(motorConfig);
    UNUSED(useUnsyncedPwm);

    printf("Initialized motor count %d\n", motorCount);
    pwmRawPkt.motorCount = motorCount;

    idlePulse = _idlePulse;

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
        motors[motorIndex].enabled = true;
    }
    motorPwmDevice.count = motorCount; // Never used, but seemingly a right thing to set it anyways.
    motorPwmDevice.initialized = true;
    motorPwmDevice.enabled = false;

    return &motorPwmDevice;
}

// ADC part
uint16_t adcGetChannel(uint8_t channel)
{
    if (channel != ADC_BATTERY || !atomicBatteryEnabled) { return 0; }
    /* Invert the configured divider into a quantized 12-bit ADC sample;
     * voltage.c still performs its native conversion and filtering. */
    const voltageSensorADCConfig_t *config = voltageSensorADCConfig(VOLTAGE_SENSOR_ADC_VBAT);
    const double scale = config->vbatscale * getVrefMv();
    if (scale <= 0 || config->vbatresdivval == 0 || config->vbatresdivmultiplier == 0) { return 0; }
    const double raw = simulatedBatteryVoltage * 100.0 * 4095.0 * 10.0 *
        config->vbatresdivval * config->vbatresdivmultiplier / scale;
    return (uint16_t)lrint(constrain(raw, 0, 4095));
}

// stack part
char _estack;
char _Min_Stack_Size;

// virtual EEPROM
static FILE *eepromFd = NULL;

void FLASH_Unlock(void)
{
    if (eepromFd != NULL) {
        fprintf(stderr, "[FLASH_Unlock] eepromFd != NULL\n");
        return;
    }

    // open or create
    eepromFd = fopen(EEPROM_FILENAME,"r+");
    if (eepromFd != NULL) {
        // obtain file size:
        fseek(eepromFd , 0 , SEEK_END);
        size_t lSize = ftell(eepromFd);
        rewind(eepromFd);

        size_t n = fread(eepromData, 1, sizeof(eepromData), eepromFd);
        if (n == lSize) {
            printf("[FLASH_Unlock] loaded '%s', size = %ld / %ld\n", EEPROM_FILENAME, lSize, sizeof(eepromData));
        } else {
            fprintf(stderr, "[FLASH_Unlock] failed to load '%s'\n", EEPROM_FILENAME);
            return;
        }
    } else {
        printf("[FLASH_Unlock] created '%s', size = %ld\n", EEPROM_FILENAME, sizeof(eepromData));
        if ((eepromFd = fopen(EEPROM_FILENAME, "w+")) == NULL) {
            fprintf(stderr, "[FLASH_Unlock] failed to create '%s'\n", EEPROM_FILENAME);
            return;
        }
        if (fwrite(eepromData, sizeof(eepromData), 1, eepromFd) != 1) {
            fprintf(stderr, "[FLASH_Unlock] write failed: %s\n", strerror(errno));
        }
    }
}

void FLASH_Lock(void)
{
    // flush & close
    if (eepromFd != NULL) {
        fseek(eepromFd, 0, SEEK_SET);
        fwrite(eepromData, 1, sizeof(eepromData), eepromFd);
        fclose(eepromFd);
        eepromFd = NULL;
        printf("[FLASH_Lock] saved '%s'\n", EEPROM_FILENAME);
    } else {
        fprintf(stderr, "[FLASH_Lock] eeprom is not unlocked\n");
    }
}

FLASH_Status FLASH_ErasePage(uintptr_t Page_Address)
{
    UNUSED(Page_Address);
//    printf("[FLASH_ErasePage]%x\n", Page_Address);
    return FLASH_COMPLETE;
}

FLASH_Status FLASH_ProgramWord(uintptr_t addr, uint32_t value)
{
    if ((addr >= (uintptr_t)eepromData) && (addr < (uintptr_t)ARRAYEND(eepromData))) {
        *((uint32_t*)addr) = value;
        printf("[FLASH_ProgramWord]%p = %08x\n", (void*)addr, *((uint32_t*)addr));
    } else {
            printf("[FLASH_ProgramWord]%p out of range!\n", (void*)addr);
    }
    return FLASH_COMPLETE;
}

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    UNUSED(io);
    UNUSED(cfg);
    printf("IOConfigGPIO\n");
}

void spektrumBind(rxConfig_t *rxConfig)
{
    UNUSED(rxConfig);
    printf("spektrumBind\n");
}

void debugInit(void)
{
    printf("debugInit\n");
}

void unusedPinsInit(void)
{
    printf("unusedPinsInit\n");
}
