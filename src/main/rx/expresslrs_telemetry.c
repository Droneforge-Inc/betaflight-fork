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

/*
 * Based on https://github.com/ExpressLRS/ExpressLRS
 * Thanks to AlessandroAU, original creator of the ExpressLRS project.
 *
 * Authors:
 * Phobos- - Original port.
 */

#include "platform.h"
#include <string.h>

#ifdef USE_RX_EXPRESSLRS

#include "common/maths.h"
#include "config/feature.h"
#include "fc/runtime_config.h"

#include "msp/msp_protocol.h"

#include "rx/crsf_protocol.h"
#include "rx/expresslrs_telemetry.h"

#include "telemetry/crsf.h"
#include "telemetry/telemetry.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"

static uint8_t tlmBuffer[CRSF_FRAME_SIZE_MAX];

#define ELRS_ASSUMED_LINK_SLOT_INTERVAL_US 5000U
#define ELRS_FLIGHT_MODE_PAYLOAD_ESTIMATE 6U
#define ELRS_MIN_OTHER_TELEMETRY_HZ 10U
#define ELRS_MAX_OTHER_TELEMETRY_PERIOD_US \
  (1000000U / ELRS_MIN_OTHER_TELEMETRY_HZ)

typedef enum {
#if defined(USE_GPS)
  CRSF_FRAME_GPS_INDEX = 0,
  CRSF_FRAME_BATTERY_SENSOR_INDEX,
#else
  CRSF_FRAME_BATTERY_SENSOR_INDEX = 0,
#endif
#if !defined(EKF_ONLY)
  CRSF_FRAME_ATTITUDE_INDEX,
#endif
#ifdef USE_EKF
  CRSF_FRAME_KINEMATIC_STATE_INDEX,
#endif
#if defined(SEND_IMU_TELEMETRY)
  CRSF_FRAME_RAW_IMU_INDEX,
#endif
#ifndef IGNORE_FLIGHT_MODE
  CRSF_FRAME_FLIGHT_MODE_INDEX,
#endif
#if defined(USE_VARIO) && !defined(EKF_ONLY)
  CRSF_FRAME_VARIO_SENSOR_INDEX,
#endif
#if defined(USE_BARO) && !defined(EKF_ONLY)
  CRSF_FRAME_BARO_ALTITUDE_INDEX,
#endif
#if defined(USE_RANGEFINDER_TF) && !defined(EKF_ONLY)
  CRSF_FRAME_RANGEFINDER_TF_INDEX,
#endif
#if defined(USE_RANGEFINDER_OPTFLOW_MTF) && !defined(EKF_ONLY)
  CRSF_FRAME_OPTRANGE_INDEX,
#endif
#if defined(SEND_MOTOR_TELEMETRY)
  CRSF_FRAME_MOTOR_RPM_INDEX,
#endif
  CRSF_FRAME_PAYLOAD_TYPES_COUNT // should be last
} frameTypeIndex_e;

static crsfFrameType_e payloadTypes[] = {
#if defined(USE_GPS)
    CRSF_FRAMETYPE_GPS,
#endif
    CRSF_FRAMETYPE_BATTERY_SENSOR,
#if !defined(EKF_ONLY)
    CRSF_FRAMETYPE_ATTITUDE,
#endif
#ifdef USE_EKF
    CRSF_FRAMETYPE_KINEMATIC_STATE,
#endif
#if defined(SEND_IMU_TELEMETRY)
    CRSF_FRAMETYPE_RAW_IMU,
#endif
#ifndef IGNORE_FLIGHT_MODE
    CRSF_FRAMETYPE_FLIGHT_MODE,
#endif
#if defined(USE_VARIO) && !defined(EKF_ONLY)
    CRSF_FRAMETYPE_VARIO_SENSOR,
#endif
#if defined(USE_BARO) && !defined(EKF_ONLY)
    CRSF_FRAMETYPE_BARO_ALTITUDE,
#endif
#if defined(USE_RANGEFINDER_TF) && !defined(EKF_ONLY)
    CRSF_FRAMETYPE_RANGEFINDER_TF,
#endif
#if defined(USE_RANGEFINDER_OPTFLOW_MTF) && !defined(EKF_ONLY)
    CRSF_FRAMETYPE_OPTRANGE,
#endif
#if defined(SEND_MOTOR_TELEMETRY)
    CRSF_FRAMETYPE_MOTOR_RPM,
#endif
};

#ifdef USE_EKF
STATIC_UNIT_TESTED uint32_t tlmSensors = 0;
#else
STATIC_UNIT_TESTED uint8_t tlmSensors = 0;
#endif
STATIC_UNIT_TESTED uint8_t currentPayloadIndex;
#if defined(EKF_ONLY) && defined(USE_EKF)
STATIC_UNIT_TESTED uint8_t ekfOnlyStatePerOtherBase;
STATIC_UNIT_TESTED uint8_t ekfOnlyStatePerOtherExtra;
STATIC_UNIT_TESTED uint8_t ekfOnlyOtherCount;
STATIC_UNIT_TESTED uint8_t ekfOnlyOtherRoundIndex;
STATIC_UNIT_TESTED uint8_t ekfOnlyStateSentForCurrentOther;
#endif

static uint8_t *data = NULL;
static uint8_t length = 0;
static uint8_t currentOffset;
static uint8_t bytesLastPayload;
static uint8_t currentPackage;
static elrsTelemetryPayloadType_e currentPayloadType = ELRS_PAYLOAD_NONE;
static bool currentPayloadIsUidMsp;
static bool waitUntilTelemetryConfirm;
static uint16_t waitCount;
static uint16_t maxWaitCount;
static volatile stubbornSenderState_e senderState;
#ifdef USE_MSP_OVER_TELEMETRY
static bool pendingUidMspReply;
static uint32_t uidRequestCount;
static uint32_t uidReplyCount;
#endif

#if defined(EKF_ONLY) && defined(USE_EKF)
static uint16_t getTelemetryFrameSize(const crsfFrameType_e frameType) {
  switch (frameType) {
  default:
  case CRSF_FRAMETYPE_ATTITUDE:
    return CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE;
  case CRSF_FRAMETYPE_BATTERY_SENSOR:
    return CRSF_FRAME_LENGTH_NON_PAYLOAD +
           CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE;
  case CRSF_FRAMETYPE_FLIGHT_MODE:
    return CRSF_FRAME_LENGTH_NON_PAYLOAD + ELRS_FLIGHT_MODE_PAYLOAD_ESTIMATE;
#if defined(USE_GPS)
  case CRSF_FRAMETYPE_GPS:
    return CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_GPS_PAYLOAD_SIZE;
#endif
#if defined(USE_VARIO)
  case CRSF_FRAMETYPE_VARIO_SENSOR:
    return CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_VARIO_SENSOR_PAYLOAD_SIZE;
#endif
#if defined(USE_BARO) && defined(USE_VARIO)
  case CRSF_FRAMETYPE_BARO_ALTITUDE:
    return CRSF_FRAME_LENGTH_NON_PAYLOAD +
           CRSF_FRAME_BARO_ALTITUDE_PAYLOAD_SIZE;
#endif
#if defined(SEND_IMU_TELEMETRY)
  case CRSF_FRAMETYPE_RAW_IMU:
    return CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_RAW_IMU_PAYLOAD_SIZE;
#endif
#ifdef USE_EKF
  case CRSF_FRAMETYPE_KINEMATIC_STATE:
    return CRSF_FRAME_LENGTH_NON_PAYLOAD +
           CRSF_FRAME_KINEMATIC_STATE_PAYLOAD_SIZE;
#endif
#if defined(USE_RANGEFINDER_TF)
  case CRSF_FRAMETYPE_RANGEFINDER_TF:
    return CRSF_FRAME_LENGTH_NON_PAYLOAD +
           CRSF_FRAME_RANGEFINDER_TF_PAYLOAD_SIZE;
#endif
#if defined(USE_RANGEFINDER_OPTFLOW_MTF)
  case CRSF_FRAMETYPE_OPTRANGE:
    return CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_OPTRANGE_PAYLOAD_SIZE;
#endif
#ifdef SEND_MOTOR_TELEMETRY
  case CRSF_FRAMETYPE_MOTOR_RPM:
    return CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_MOTOR_RPM_PAYLOAD_SIZE;
#endif
  }
}

static uint32_t getTelemetryFrameIntervalUs(const crsfFrameType_e frameType) {
  const uint32_t frameBytes = getTelemetryFrameSize(frameType);
  const uint32_t slotCount =
      (frameBytes + (ELRS_TELEMETRY_BYTES_PER_CALL - 1U)) /
      ELRS_TELEMETRY_BYTES_PER_CALL;
  return slotCount * ELRS_ASSUMED_LINK_SLOT_INTERVAL_US;
}
#endif

static void telemetrySenderResetState(void) {
  bytesLastPayload = 0;
  currentOffset = 0;
  currentPackage = 1;
  currentPayloadType = ELRS_PAYLOAD_NONE;
  currentPayloadIsUidMsp = false;
  waitUntilTelemetryConfirm = true;
  waitCount = 0;
  // 80 corresponds to UpdateTelemetryRate(ANY, 2, 1), which is what the TX uses
  // in boost mode
  maxWaitCount = 80;
  senderState = ELRS_SENDER_IDLE;
}

/***
 * Queues a message to send, will abort the current message if one is currently
 * being transmitted
 ***/
void setTelemetryDataToTransmit(const uint8_t lengthToTransmit,
                                uint8_t *dataToTransmit,
                                elrsTelemetryPayloadType_e payloadType) {
  length = lengthToTransmit;
  data = dataToTransmit;
  currentOffset = 0;
  currentPackage = 1;
  currentPayloadType = payloadType;
#ifdef USE_MSP_OVER_TELEMETRY
  currentPayloadIsUidMsp = (payloadType == ELRS_PAYLOAD_MSP) ? pendingUidMspReply : false;
  pendingUidMspReply = false;
#else
  currentPayloadIsUidMsp = false;
#endif
  waitCount = 0;
  senderState =
      (senderState == ELRS_SENDER_IDLE) ? ELRS_SENDING : ELRS_RESYNC_THEN_SEND;
}

bool isTelemetrySenderActive(void) { return senderState != ELRS_SENDER_IDLE; }

bool isRegularTelemetrySenderActive(void) {
  return isTelemetrySenderActive() && currentPayloadType == ELRS_PAYLOAD_REGULAR;
}

/***
 * Copy up to maxLen bytes from the current package to outData
 * packageIndex
 ***/
uint8_t getCurrentTelemetryPayload(uint8_t *outData) {
  uint8_t packageIndex;

  bytesLastPayload = 0;
  switch (senderState) {
  case ELRS_RESYNC:
  case ELRS_RESYNC_THEN_SEND:
    packageIndex = ELRS_TELEMETRY_MAX_PACKAGES;
    break;
  case ELRS_SENDING:
    bytesLastPayload =
        MIN((uint8_t)(length - currentOffset), ELRS_TELEMETRY_BYTES_PER_CALL);
    // If this is the last data chunk, and there has been at least one other
    // packet skip the blank packet needed for WAIT_UNTIL_NEXT_CONFIRM
    if (currentPackage > 1 && (currentOffset + bytesLastPayload) >= length) {
      packageIndex = 0;
    } else {
      packageIndex = currentPackage;
    }
    memcpy(outData, &data[currentOffset], bytesLastPayload);

    break;
  default:
    packageIndex = 0;
  }

  return packageIndex;
}

void confirmCurrentTelemetryPayload(const bool telemetryConfirmValue) {
  stubbornSenderState_e nextSenderState = senderState;

  switch (senderState) {
  case ELRS_SENDING:
    if (telemetryConfirmValue != waitUntilTelemetryConfirm) {
      waitCount++;
      if (waitCount > maxWaitCount) {
        waitUntilTelemetryConfirm = !telemetryConfirmValue;
        nextSenderState = ELRS_RESYNC;
      }
      break;
    }

    currentOffset += bytesLastPayload;
    if (currentOffset >= length) {
      // A 0th packet is always requred so the reciver can
      // differentiate a new send from a resend, if this is
      // the first packet acked, send another, else IDLE
      if (currentPackage == 1) {
        nextSenderState = ELRS_WAIT_UNTIL_NEXT_CONFIRM;
      } else {
        nextSenderState = ELRS_SENDER_IDLE;
      }
    }

    currentPackage++;
    waitUntilTelemetryConfirm = !waitUntilTelemetryConfirm;
    waitCount = 0;
    break;

  case ELRS_RESYNC:
  case ELRS_RESYNC_THEN_SEND:
  case ELRS_WAIT_UNTIL_NEXT_CONFIRM:
    if (telemetryConfirmValue == waitUntilTelemetryConfirm) {
      nextSenderState = (senderState == ELRS_RESYNC_THEN_SEND)
                            ? ELRS_SENDING
                            : ELRS_SENDER_IDLE;
      waitUntilTelemetryConfirm = !telemetryConfirmValue;
    } else if (senderState ==
               ELRS_WAIT_UNTIL_NEXT_CONFIRM) { // switch to resync if tx does
                                               // not confirm value fast enough
      waitCount++;
      if (waitCount > maxWaitCount) {
        waitUntilTelemetryConfirm = !telemetryConfirmValue;
        nextSenderState = ELRS_RESYNC;
      }
    }

    break;

  case ELRS_SENDER_IDLE:
    break;
  }

  senderState = nextSenderState;
  if (senderState == ELRS_SENDER_IDLE) {
#ifdef USE_MSP_OVER_TELEMETRY
    if (currentPayloadType == ELRS_PAYLOAD_MSP && currentPayloadIsUidMsp) {
      uidReplyCount++;
    }
#endif
    currentPayloadType = ELRS_PAYLOAD_NONE;
    currentPayloadIsUidMsp = false;
  }
}

#ifdef USE_MSP_OVER_TELEMETRY
static uint8_t *mspData = NULL;
static volatile bool finishedData;
static volatile uint8_t mspLength = 0;
static volatile uint8_t mspCurrentOffset;
static volatile uint8_t mspCurrentPackage;
static volatile bool mspConfirm;

STATIC_UNIT_TESTED volatile bool mspReplyPending;
STATIC_UNIT_TESTED volatile bool deviceInfoReplyPending;

bool hasPriorityTelemetryPending(void) {
  return mspReplyPending || deviceInfoReplyPending;
}

void mspReceiverResetState(void) {
  mspCurrentOffset = 0;
  mspCurrentPackage = 1;
  mspConfirm = false;
  mspReplyPending = false;
  pendingUidMspReply = false;
  deviceInfoReplyPending = false;
}

bool getCurrentMspConfirm(void) { return mspConfirm; }

void setMspDataToReceive(const uint8_t maxLength, uint8_t *dataToReceive) {
  mspLength = maxLength;
  mspData = dataToReceive;
  mspCurrentPackage = 1;
  mspCurrentOffset = 0;
  finishedData = false;
}

void receiveMspData(const uint8_t packageIndex,
                    const volatile uint8_t *const receiveData) {
  // Resync
  if (packageIndex == ELRS_MSP_MAX_PACKAGES) {
    mspConfirm = !mspConfirm;
    mspCurrentPackage = 1;
    mspCurrentOffset = 0;
    finishedData = false;
    return;
  }

  if (finishedData) {
    return;
  }

  bool acceptData = false;
  if (packageIndex == 0 && mspCurrentPackage > 1) {
    // PackageIndex 0 (the final packet) can also contain data
    acceptData = true;
    finishedData = true;
  } else if (packageIndex == mspCurrentPackage) {
    acceptData = true;
    mspCurrentPackage++;
  }

  if (acceptData && (receiveData != NULL)) {
    uint8_t len =
        MIN((uint8_t)(mspLength - mspCurrentOffset), ELRS_MSP_BYTES_PER_CALL);
    memcpy(&mspData[mspCurrentOffset], (const uint8_t *)receiveData, len);
    mspCurrentOffset += len;
    mspConfirm = !mspConfirm;
  }
}

bool hasFinishedMspData(void) { return finishedData; }

void mspReceiverUnlock(void) {
  if (finishedData) {
    mspCurrentPackage = 1;
    mspCurrentOffset = 0;
    finishedData = false;
  }
}

static uint8_t mspFrameSize = 0;
static uint8_t mspRequestOriginId = CRSF_ADDRESS_RADIO_TRANSMITTER;

static void bufferMspResponse(uint8_t *payload, const uint8_t payloadSize) {
  pendingUidMspReply = telemetryMspPayloadIsUidResponse(payload, payloadSize);
  mspFrameSize =
      getCrsfMspFrame(tlmBuffer, mspRequestOriginId, payload, payloadSize);
}

void processMspPacket(uint8_t *packet) {
  switch (packet[2]) {
  case CRSF_FRAMETYPE_DEVICE_PING:
    deviceInfoReplyPending = true;
    break;
  case CRSF_FRAMETYPE_MSP_REQ:
  case CRSF_FRAMETYPE_MSP_WRITE:
    mspRequestOriginId = packet[ELRS_MSP_ORIGIN_INDEX];
    if (telemetryMspPayloadIsUidRequest(&packet[ELRS_MSP_PACKET_OFFSET],
                                        CRSF_FRAME_RX_MSP_FRAME_SIZE)) {
      uidRequestCount++;
    }
    if (bufferCrsfMspFrame(&packet[ELRS_MSP_PACKET_OFFSET],
                           CRSF_FRAME_RX_MSP_FRAME_SIZE)) {
      handleCrsfMspFrameBuffer(&bufferMspResponse);
      mspReplyPending = true;
    }
    break;
  default:
    break;
  }
}

uint32_t expressLrsGetUidRequestCount(void) { return uidRequestCount; }

uint32_t expressLrsGetUidReplyCount(void) { return uidReplyCount; }
#endif

/*
 * Called when the telemetry ratio or air rate changes, calculate
 * the new threshold for how many times the telemetryConfirmValue
 * can be wrong in a row before giving up and going to RESYNC
 */
void updateTelemetryRate(const uint16_t airRate, const uint8_t tlmRatio,
                         const uint8_t tlmBurst) {
  // consipicuously unused airRate parameter, the wait count is strictly based
  // on number of packets, not time between the telemetry packets, or a wall
  // clock timeout
  UNUSED(airRate);
  // The expected number of packet periods between telemetry packets
  uint32_t packsBetween = tlmRatio * (1 + tlmBurst) / tlmBurst;
  maxWaitCount = packsBetween * ELRS_TELEMETRY_MAX_MISSED_PACKETS;
}

void initTelemetry(void) {
  tlmSensors = 0;
  currentPayloadIndex = 0;
#if defined(EKF_ONLY) && defined(USE_EKF)
  ekfOnlyStatePerOtherBase = 0;
  ekfOnlyStatePerOtherExtra = 0;
  ekfOnlyOtherCount = 0;
  ekfOnlyOtherRoundIndex = 0;
  ekfOnlyStateSentForCurrentOther = 0;
#endif

  telemetrySenderResetState();
#ifdef USE_MSP_OVER_TELEMETRY
  mspReceiverResetState();
#endif

  if (!featureIsEnabled(FEATURE_TELEMETRY)) {
    return;
  }

#if !defined(EKF_ONLY)
  if (sensors(SENSOR_ACC) &&
      telemetryIsSensorEnabled(SENSOR_PITCH | SENSOR_ROLL | SENSOR_HEADING)) {
    tlmSensors |= BIT(CRSF_FRAME_ATTITUDE_INDEX);
  }
#endif
#ifdef SEND_IMU_TELEMETRY
  if (sensors(SENSOR_ACC) &&
      telemetryIsSensorEnabled(SENSOR_PITCH | SENSOR_ROLL | SENSOR_HEADING)) {
    tlmSensors |= BIT(CRSF_FRAME_RAW_IMU_INDEX);
  }
#endif

#ifdef USE_EKF
  if (sensors(SENSOR_ACC) &&
      telemetryIsSensorEnabled(SENSOR_ALTITUDE | SENSOR_PITCH | SENSOR_ROLL |
                               SENSOR_HEADING)) {
    tlmSensors |= BIT(CRSF_FRAME_KINEMATIC_STATE_INDEX);
  }
#endif
#if defined(USE_BARO) && defined(USE_VARIO) && !defined(EKF_ONLY)
  if (telemetryIsSensorEnabled(SENSOR_ALTITUDE)) {
    tlmSensors |= BIT(CRSF_FRAME_BARO_ALTITUDE_INDEX);
  }
#endif
  if ((isBatteryVoltageConfigured() &&
       telemetryIsSensorEnabled(SENSOR_VOLTAGE)) ||
      (isAmperageConfigured() &&
       telemetryIsSensorEnabled(SENSOR_CURRENT | SENSOR_FUEL))) {
    tlmSensors |= BIT(CRSF_FRAME_BATTERY_SENSOR_INDEX);
  }
#ifndef IGNORE_FLIGHT_MODE
  if (telemetryIsSensorEnabled(SENSOR_MODE)) {
    tlmSensors |= BIT(CRSF_FRAME_FLIGHT_MODE_INDEX);
  }
#endif
#if defined(USE_VARIO) && !defined(EKF_ONLY)
  if ((sensors(SENSOR_BARO) || featureIsEnabled(FEATURE_GPS)) &&
      telemetryIsSensorEnabled(SENSOR_VARIO)) {
    tlmSensors |= BIT(CRSF_FRAME_VARIO_SENSOR_INDEX);
  }
#endif
#if defined(USE_RANGEFINDER_TF) && !defined(EKF_ONLY)
  if (sensors(SENSOR_SONAR) && telemetryIsSensorEnabled(SENSOR_LIDAR)) {
    tlmSensors |= BIT(CRSF_FRAME_RANGEFINDER_TF_INDEX);
  }
#endif
#if defined(USE_RANGEFINDER_OPTFLOW_MTF) && !defined(EKF_ONLY)
  if (sensors(SENSOR_OPTICALFLOW) &&
      telemetryIsSensorEnabled(SENSOR_OPTRANGE)) {
    tlmSensors |= BIT(CRSF_FRAME_OPTRANGE_INDEX);
  }
#endif

#if defined(SEND_MOTOR_TELEMETRY)
  // Always send motor telemetry if enabled
  tlmSensors |= BIT(CRSF_FRAME_MOTOR_RPM_INDEX);
#endif
#ifdef USE_GPS
  if (featureIsEnabled(FEATURE_GPS) &&
      telemetryIsSensorEnabled(SENSOR_ALTITUDE | SENSOR_LAT_LONG |
                               SENSOR_GROUND_SPEED | SENSOR_HEADING)) {
    tlmSensors |= BIT(CRSF_FRAME_GPS_INDEX);
  }
#endif

#if defined(EKF_ONLY) && defined(USE_EKF)
  if (tlmSensors & BIT(CRSF_FRAME_KINEMATIC_STATE_INDEX)) {
    uint8_t stateCount = 1;
    uint32_t otherCycleUs = 0;

    for (uint8_t i = 0; i < CRSF_FRAME_PAYLOAD_TYPES_COUNT; i++) {
      if (i == CRSF_FRAME_KINEMATIC_STATE_INDEX || !(tlmSensors & BIT(i))) {
        continue;
      }

      ekfOnlyOtherCount++;
      otherCycleUs += getTelemetryFrameIntervalUs(payloadTypes[i]);
    }

    if (ekfOnlyOtherCount > 0) {
      const uint32_t kinematicIntervalUs =
          getTelemetryFrameIntervalUs(CRSF_FRAMETYPE_KINEMATIC_STATE);
      const uint32_t weightedCycleUs =
          otherCycleUs + 2U * ekfOnlyOtherCount * kinematicIntervalUs;

      if (weightedCycleUs <= ELRS_MAX_OTHER_TELEMETRY_PERIOD_US) {
        stateCount = 2U * ekfOnlyOtherCount;
      } else if (otherCycleUs + kinematicIntervalUs <=
                 ELRS_MAX_OTHER_TELEMETRY_PERIOD_US) {
        stateCount =
            (uint8_t)((ELRS_MAX_OTHER_TELEMETRY_PERIOD_US - otherCycleUs) /
                      kinematicIntervalUs);
      }

      ekfOnlyStatePerOtherBase = stateCount / ekfOnlyOtherCount;
      ekfOnlyStatePerOtherExtra = stateCount % ekfOnlyOtherCount;
    }
  }
#endif
}

bool getNextTelemetryPayload(uint8_t *nextPayloadSize, uint8_t **payloadData,
                             elrsTelemetryPayloadType_e *payloadType) {
#ifdef USE_MSP_OVER_TELEMETRY
  if (mspReplyPending) {
    *nextPayloadSize = mspFrameSize;
    *payloadData = tlmBuffer;
    *payloadType = ELRS_PAYLOAD_MSP;
    mspReplyPending = false;
    return true;
  } else if (deviceInfoReplyPending) {
    *nextPayloadSize = getCrsfFrame(tlmBuffer, CRSF_FRAMETYPE_DEVICE_INFO);
    *payloadData = tlmBuffer;
    *payloadType = ELRS_PAYLOAD_DEVICE_INFO;
    deviceInfoReplyPending = false;
    return true;
  } else
#endif
#if defined(EKF_ONLY) && defined(USE_EKF)
      if (tlmSensors & BIT(CRSF_FRAME_KINEMATIC_STATE_INDEX)) {
    if (ekfOnlyOtherCount == 0) {
      *nextPayloadSize = getCrsfFrame(tlmBuffer, CRSF_FRAMETYPE_KINEMATIC_STATE);
      *payloadData = tlmBuffer;
      *payloadType = ELRS_PAYLOAD_REGULAR;
      return true;
    }

    const uint8_t stateQuota =
        ekfOnlyStatePerOtherBase +
        ((ekfOnlyOtherRoundIndex < ekfOnlyStatePerOtherExtra) ? 1U : 0U);

    if (ekfOnlyStateSentForCurrentOther < stateQuota) {
      *nextPayloadSize = getCrsfFrame(tlmBuffer, CRSF_FRAMETYPE_KINEMATIC_STATE);
      *payloadData = tlmBuffer;
      *payloadType = ELRS_PAYLOAD_REGULAR;
      ekfOnlyStateSentForCurrentOther++;
      return true;
    }

    for (uint8_t i = 0; i < CRSF_FRAME_PAYLOAD_TYPES_COUNT; i++) {
      const uint8_t payloadIndex = currentPayloadIndex;
      currentPayloadIndex =
          (currentPayloadIndex + 1) % CRSF_FRAME_PAYLOAD_TYPES_COUNT;

      if (payloadIndex == CRSF_FRAME_KINEMATIC_STATE_INDEX) {
        continue;
      }

      if (tlmSensors & BIT(payloadIndex)) {
        *nextPayloadSize = getCrsfFrame(tlmBuffer, payloadTypes[payloadIndex]);
        *payloadData = tlmBuffer;
        *payloadType = ELRS_PAYLOAD_REGULAR;
        ekfOnlyStateSentForCurrentOther = 0;
        ekfOnlyOtherRoundIndex =
            (ekfOnlyOtherRoundIndex + 1) % ekfOnlyOtherCount;
        return true;
      }
    }

    *nextPayloadSize = getCrsfFrame(tlmBuffer, CRSF_FRAMETYPE_KINEMATIC_STATE);
    *payloadData = tlmBuffer;
    *payloadType = ELRS_PAYLOAD_REGULAR;
    ekfOnlyOtherRoundIndex = 0;
    ekfOnlyStateSentForCurrentOther = 0;
    return true;
  } else
#endif
      if (tlmSensors & BIT(currentPayloadIndex)) {
    *nextPayloadSize =
        getCrsfFrame(tlmBuffer, payloadTypes[currentPayloadIndex]);
    *payloadData = tlmBuffer;
    *payloadType = ELRS_PAYLOAD_REGULAR;
    currentPayloadIndex =
        (currentPayloadIndex + 1) % CRSF_FRAME_PAYLOAD_TYPES_COUNT;
    return true;
  } else {
    currentPayloadIndex =
        (currentPayloadIndex + 1) % CRSF_FRAME_PAYLOAD_TYPES_COUNT;
    *nextPayloadSize = 0;
    *payloadData = 0;
    *payloadType = ELRS_PAYLOAD_NONE;
    return false;
  }
}

#endif
