#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

#define MIGHTYCAM_TASK_PERIOD_MS 5
#define MIGHTYCAM_CONFIDENCE_SCALE 255
#define MIGHTYCAM_POSITION_SCALE_MM 1000
#define MIGHTYCAM_QUATERNION_SCALE 32767
#define MIGHTYCAM_POSE_PAYLOAD_LENGTH 24
#define MIGHTYCAM_STATE_PAYLOAD_LENGTH 10
#define MIGHTYCAM_CRSF_PAYLOAD_SIZE MIGHTYCAM_POSE_PAYLOAD_LENGTH

#define MIGHTYCAM_STATE_FLAG_RECORDING (1 << 0)
#define MIGHTYCAM_STATE_FLAG_PREVIEW_MODE (1 << 1)
#define MIGHTYCAM_STATE_FLAG_INITIALIZING (1 << 2)
#define MIGHTYCAM_STATE_FLAG_ARM_CONNECTED (1 << 3)

typedef enum {
    MIGHTYCAM_PACKET_TYPE_POSE = 0x01,
    MIGHTYCAM_PACKET_TYPE_STATE = 0x02,
} mightycamPacketType_e;

typedef enum {
    MIGHTYCAM_VIO_STATE_UNKNOWN = 0,
    MIGHTYCAM_VIO_STATE_OFF,
    MIGHTYCAM_VIO_STATE_INITIALIZING,
    MIGHTYCAM_VIO_STATE_TRACKING,
    MIGHTYCAM_VIO_STATE_DEGRADED,
    MIGHTYCAM_VIO_STATE_LOST,
} mightycamVioState_e;

typedef struct mightycamPoseData_s {
    uint64_t timestampNs;
    bool valid;
    uint8_t confidence;
    int16_t positionXMm;
    int16_t positionYMm;
    int16_t positionZMm;
    int16_t orientationXScaled;
    int16_t orientationYScaled;
    int16_t orientationZScaled;
    int16_t orientationWScaled;
    timeMs_t receivedMs;
    uint32_t packetCount;
} mightycamPoseData_t;

typedef struct mightycamStateData_s {
    uint64_t timestampNs;
    mightycamVioState_e vioState;
    bool recording;
    bool previewMode;
    bool initializing;
    bool armConnected;
    timeMs_t receivedMs;
    uint32_t packetCount;
} mightycamStateData_t;

typedef struct mightycamStats_s {
    uint32_t bytesReceived;
    uint32_t frameCount;
    uint32_t poseFrameCount;
    uint32_t stateFrameCount;
    uint16_t checksumErrors;
    uint16_t parseErrors;
    uint16_t unknownTypeErrors;
    uint8_t lastPacketType;
    uint8_t lastPayloadLength;
} mightycamStats_t;

typedef struct mightycamRawPayload_s {
    uint8_t packetType;
    uint8_t payloadLength;
    uint8_t payload[MIGHTYCAM_POSE_PAYLOAD_LENGTH];
    timeMs_t receivedMs;
    uint32_t packetCount;
} mightycamRawPayload_t;

bool mightycamInit(void);
void mightycamUpdate(void);
bool mightycamIsDetected(void);
bool mightycamIsHealthy(void);
bool mightycamGetLatestPose(mightycamPoseData_t *data);
bool mightycamGetLatestState(mightycamStateData_t *data);
bool mightycamGetLatestRawPayload(uint8_t packetType, mightycamRawPayload_t *payload);
void mightycamGetStats(mightycamStats_t *stats);
