#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_MIGHTYCAM

#include "build/build_config.h"
#include "build/debug.h"

#include "common/utils.h"

#include "drivers/mightycam/mightycam.h"
#include "drivers/time.h"

#include "io/serial.h"

#define MIGHTYCAM_FRAME_SYNC_BYTE 0xDF
#define MIGHTYCAM_BAUDRATE 115200
#define MIGHTYCAM_TIMEOUT_MS 500
#define MIGHTYCAM_MAX_PAYLOAD_LENGTH 255

typedef enum {
    MIGHTYCAM_FRAME_STATE_WAIT_START,
    MIGHTYCAM_FRAME_STATE_READING_TYPE,
    MIGHTYCAM_FRAME_STATE_READING_LENGTH,
    MIGHTYCAM_FRAME_STATE_READING_PAYLOAD,
    MIGHTYCAM_FRAME_STATE_WAIT_CKSUM,
} mightycamFrameState_e;

static serialPort_t *mightycamSerialPort = NULL;
static bool mightycamDetected = false;

static mightycamFrameState_e mightycamFrameState;
static uint8_t mightycamPacketType;
static uint8_t mightycamPayloadLength;
static uint8_t mightycamPayload[MIGHTYCAM_MAX_PAYLOAD_LENGTH + 1];
static uint8_t mightycamReceivePosition;

static mightycamPoseData_t mightycamPoseData;
static mightycamStateData_t mightycamStateData;
static mightycamRawPayload_t mightycamPoseRawPayload;
static mightycamRawPayload_t mightycamStateRawPayload;
static mightycamStats_t mightycamStats;
static timeMs_t mightycamLastFrameReceivedMs = 0;

static void mightycamResetFrame(void)
{
    mightycamFrameState = MIGHTYCAM_FRAME_STATE_WAIT_START;
    mightycamReceivePosition = 0;
    mightycamPayloadLength = 0;
}

static uint8_t mightycamChecksum(const uint8_t packetType, const uint8_t payloadLength, const uint8_t *payload)
{
    uint8_t checksum = packetType + payloadLength;

    for (int i = 0; i < payloadLength; i++) {
        checksum += payload[i];
    }

    return checksum;
}

static uint16_t mightycamReadU16Le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t mightycamReadI16Le(const uint8_t *data)
{
    return (int16_t)mightycamReadU16Le(data);
}

static uint32_t mightycamReadU32Le(const uint8_t *data)
{
    return (uint32_t)data[0] |
        ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[3] << 24);
}

static uint64_t mightycamReadU64Le(const uint8_t *data)
{
    return (uint64_t)mightycamReadU32Le(data) |
        ((uint64_t)mightycamReadU32Le(&data[4]) << 32);
}

static bool mightycamHandlePosePayload(uint8_t *payload, const uint8_t payloadLength, const timeMs_t timeNowMs)
{
    if (payloadLength != MIGHTYCAM_POSE_PAYLOAD_LENGTH) {
        return false;
    }

    mightycamPoseData_t data;
    data.timestampNs = mightycamReadU64Le(&payload[0]);
    data.valid = payload[8] != 0;
    data.confidence = payload[9];
    data.positionXMm = mightycamReadI16Le(&payload[10]);
    data.positionYMm = mightycamReadI16Le(&payload[12]);
    data.positionZMm = mightycamReadI16Le(&payload[14]);
    data.orientationXScaled = mightycamReadI16Le(&payload[16]);
    data.orientationYScaled = mightycamReadI16Le(&payload[18]);
    data.orientationZScaled = mightycamReadI16Le(&payload[20]);
    data.orientationWScaled = mightycamReadI16Le(&payload[22]);
    data.receivedMs = timeNowMs;
    data.packetCount = mightycamPoseData.packetCount + 1;

    mightycamPoseData = data;
    mightycamStats.poseFrameCount++;

    return true;
}

static bool mightycamHandleStatePayload(uint8_t *payload, const uint8_t payloadLength, const timeMs_t timeNowMs)
{
    if (payloadLength != MIGHTYCAM_STATE_PAYLOAD_LENGTH) {
        return false;
    }

    mightycamStateData_t data;
    const uint8_t flags = payload[9];

    data.timestampNs = mightycamReadU64Le(&payload[0]);
    data.vioState = payload[8] <= MIGHTYCAM_VIO_STATE_LOST ?
        (mightycamVioState_e)payload[8] :
        MIGHTYCAM_VIO_STATE_UNKNOWN;
    data.recording = (flags & MIGHTYCAM_STATE_FLAG_RECORDING) != 0;
    data.previewMode = (flags & MIGHTYCAM_STATE_FLAG_PREVIEW_MODE) != 0;
    data.initializing = (flags & MIGHTYCAM_STATE_FLAG_INITIALIZING) != 0;
    data.armConnected = (flags & MIGHTYCAM_STATE_FLAG_ARM_CONNECTED) != 0;
    data.receivedMs = timeNowMs;
    data.packetCount = mightycamStateData.packetCount + 1;

    mightycamStateData = data;
    mightycamStats.stateFrameCount++;

    return true;
}

static void mightycamHandlePacket(const uint8_t packetType, const uint8_t payloadLength, const timeMs_t timeNowMs)
{
    bool parsed = false;
    mightycamRawPayload_t *rawPayload = NULL;

    switch (packetType) {
    case MIGHTYCAM_PACKET_TYPE_POSE:
        parsed = mightycamHandlePosePayload(mightycamPayload, payloadLength, timeNowMs);
        rawPayload = &mightycamPoseRawPayload;
        break;

    case MIGHTYCAM_PACKET_TYPE_STATE:
        parsed = mightycamHandleStatePayload(mightycamPayload, payloadLength, timeNowMs);
        rawPayload = &mightycamStateRawPayload;
        break;

    default:
        mightycamStats.unknownTypeErrors++;
        break;
    }

    if (parsed) {
        rawPayload->packetType = packetType;
        rawPayload->payloadLength = payloadLength;
        memcpy(rawPayload->payload, mightycamPayload, payloadLength);
        rawPayload->receivedMs = timeNowMs;
        rawPayload->packetCount++;

        mightycamLastFrameReceivedMs = timeNowMs;
        mightycamStats.frameCount++;
        mightycamStats.lastPacketType = packetType;
        mightycamStats.lastPayloadLength = payloadLength;
    } else if (packetType == MIGHTYCAM_PACKET_TYPE_POSE || packetType == MIGHTYCAM_PACKET_TYPE_STATE) {
        mightycamStats.parseErrors++;
    }
}

bool mightycamInit(void)
{
    if (mightycamDetected) {
        return true;
    }

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_MIGHTYCAM);

    if (!portConfig) {
        return false;
    }

    mightycamSerialPort = openSerialPort(portConfig->identifier, FUNCTION_MIGHTYCAM, NULL, NULL, MIGHTYCAM_BAUDRATE, MODE_RX, 0);

    if (!mightycamSerialPort) {
        return false;
    }

    memset(&mightycamPoseData, 0, sizeof(mightycamPoseData));
    memset(&mightycamStateData, 0, sizeof(mightycamStateData));
    memset(&mightycamPoseRawPayload, 0, sizeof(mightycamPoseRawPayload));
    memset(&mightycamStateRawPayload, 0, sizeof(mightycamStateRawPayload));
    memset(&mightycamStats, 0, sizeof(mightycamStats));
    mightycamResetFrame();
    mightycamDetected = true;

    return true;
}

void mightycamUpdate(void)
{
    if (!mightycamSerialPort) {
        return;
    }

    const timeMs_t timeNowMs = millis();

    while (serialRxBytesWaiting(mightycamSerialPort)) {
        const uint8_t c = serialRead(mightycamSerialPort);
        mightycamStats.bytesReceived++;

        switch (mightycamFrameState) {
        case MIGHTYCAM_FRAME_STATE_WAIT_START:
            if (c == MIGHTYCAM_FRAME_SYNC_BYTE) {
                mightycamFrameState = MIGHTYCAM_FRAME_STATE_READING_TYPE;
            }
            break;

        case MIGHTYCAM_FRAME_STATE_READING_TYPE:
            mightycamPacketType = c;
            mightycamFrameState = MIGHTYCAM_FRAME_STATE_READING_LENGTH;
            break;

        case MIGHTYCAM_FRAME_STATE_READING_LENGTH:
            mightycamPayloadLength = c;
            mightycamReceivePosition = 0;
            mightycamFrameState = mightycamPayloadLength == 0 ?
                MIGHTYCAM_FRAME_STATE_WAIT_CKSUM :
                MIGHTYCAM_FRAME_STATE_READING_PAYLOAD;
            break;

        case MIGHTYCAM_FRAME_STATE_READING_PAYLOAD:
            mightycamPayload[mightycamReceivePosition++] = c;
            if (mightycamReceivePosition == mightycamPayloadLength) {
                mightycamFrameState = MIGHTYCAM_FRAME_STATE_WAIT_CKSUM;
            }
            break;

        case MIGHTYCAM_FRAME_STATE_WAIT_CKSUM:
            if (c == mightycamChecksum(mightycamPacketType, mightycamPayloadLength, mightycamPayload)) {
                mightycamHandlePacket(mightycamPacketType, mightycamPayloadLength, timeNowMs);
            } else {
                mightycamStats.checksumErrors++;
            }
            mightycamResetFrame();
            break;

        default:
            mightycamResetFrame();
            break;
        }
    }

    DEBUG_SET(DEBUG_MIGHTYCAM, 0, mightycamStats.frameCount);
    DEBUG_SET(DEBUG_MIGHTYCAM, 1, mightycamStats.poseFrameCount);
    DEBUG_SET(DEBUG_MIGHTYCAM, 2, mightycamStats.stateFrameCount);
    DEBUG_SET(DEBUG_MIGHTYCAM, 3, mightycamStats.checksumErrors);
    DEBUG_SET(DEBUG_MIGHTYCAM, 4, mightycamStats.parseErrors);
    DEBUG_SET(DEBUG_MIGHTYCAM, 5, mightycamStats.lastPacketType);
    DEBUG_SET(DEBUG_MIGHTYCAM, 6, mightycamStats.lastPayloadLength);
    DEBUG_SET(DEBUG_MIGHTYCAM, 7, mightycamIsHealthy());
}

bool mightycamIsDetected(void)
{
    return mightycamDetected;
}

bool mightycamIsHealthy(void)
{
    return mightycamDetected && mightycamLastFrameReceivedMs != 0 &&
        millis() - mightycamLastFrameReceivedMs <= MIGHTYCAM_TIMEOUT_MS;
}

bool mightycamGetLatestPose(mightycamPoseData_t *data)
{
    if (!data || mightycamPoseData.packetCount == 0) {
        return false;
    }

    *data = mightycamPoseData;
    return true;
}

bool mightycamGetLatestState(mightycamStateData_t *data)
{
    if (!data || mightycamStateData.packetCount == 0) {
        return false;
    }

    *data = mightycamStateData;
    return true;
}

bool mightycamGetLatestRawPayload(uint8_t packetType, mightycamRawPayload_t *payload)
{
    if (!payload) {
        return false;
    }

    const mightycamRawPayload_t *source = NULL;

    switch (packetType) {
    case MIGHTYCAM_PACKET_TYPE_POSE:
        source = &mightycamPoseRawPayload;
        break;

    case MIGHTYCAM_PACKET_TYPE_STATE:
        source = &mightycamStateRawPayload;
        break;

    default:
        return false;
    }

    if (source->packetCount == 0) {
        return false;
    }

    *payload = *source;
    return true;
}

void mightycamGetStats(mightycamStats_t *stats)
{
    if (!stats) {
        return;
    }

    *stats = mightycamStats;
}

#endif
