#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_RANGEFINDER_OPTFLOW_MTF

#include "build/debug.h"
#include "build/build_config.h"

#include "io/serial.h"

#include "drivers/time.h"
#include "drivers/optrange/optrange_mtf.h"

#define MTF_DEVTYPE_NONE 0
#define MTF_DEVTYPE_02   1

static uint8_t mtfDevtype = MTF_DEVTYPE_NONE;

#define MTF_HEADER_LENGTH        5          // Excluding sync byte (0xEF): dev_id, sys_id, msg_id, seq, len
#define MTF_MAX_PAYLOAD_LENGTH   64         // Max payload per MICOLINK spec
#define MTF_FRAME_SYNC_BYTE     0xEF
#define MTF_TIMEOUT_MS      (100 * 2)
#define MTF_MSG_ID_RANGE_SENSOR  0x51

// Microlink MTF02 frame format (From Microlink MTF-02P)
// Byte Off Description
// 1    -   SYNC
// 2    -   DEV ID
// 3    -   SYS ID
// 4    -   MSG ID
// 5    -   SEQ
// 6    -   Payload length
// 7-10 0   System time
//11-14 4   Distance (mm)
// 15   8   Dist strength
// 16   9   Dist precision
// 17   10  Dist Status (0 is invalid, 1 is valid)
// 18   11  Reserved
//19-20 12  Vel X (cm/s)
//21-22 14  Vel Y (cm/s)
// 23   16  Flow quality
// 24   17  Flow status
//25-26 18  Reserved
//27    -  Checksum

#define MTF_02_RANGE_MIN 1 // 1mm
#define MTF_02_RANGE_MAX 6000 // 6m

#define MTF_DETECTION_CONE_DECIDEGREES 450 // CHECK: is ok?

static serialPort_t *mtfSerialPort = NULL;

typedef enum {
    MTF_FRAME_STATE_WAIT_START,
    MTF_FRAME_STATE_READING_HEADER,
    MTF_FRAME_STATE_READING_PAYLOAD,
    MTF_FRAME_STATE_WAIT_CKSUM,
} mtfFrameState_e;

static mtfFrameState_e mtfFrameState;
static uint8_t mtfHeader[MTF_HEADER_LENGTH];
static uint8_t mtfPayload[MTF_MAX_PAYLOAD_LENGTH];
static uint8_t mtfReceivePosition;
static uint8_t mtfPayloadLength;  // Actual payload length from header

static uint32_t mtfDistValue;
static uint8_t mtfDistStrength;
static uint8_t mtfDistPrecision;
static uint8_t mtfDistStatus;
static int16_t mtfVelX;
static int16_t mtfVelY;
static uint8_t mtfFlowQuality;
static uint8_t mtfFlowStatus;

static uint16_t mtferrors = 0;

void mtfInit(optrangeDev_t *dev)
{
    UNUSED(dev);

    mtfFrameState = MTF_FRAME_STATE_WAIT_START;
    mtfReceivePosition = 0;
}

void mtfUpdate(optrangeDev_t *dev)
{
    UNUSED(dev);
    static timeMs_t lastFrameReceivedMs = 0;
    const timeMs_t timeNowMs = millis();

    if (mtfSerialPort == NULL) {
        return;
    }

    while (serialRxBytesWaiting(mtfSerialPort)) {
        uint8_t c = serialRead(mtfSerialPort);
        
        switch (mtfFrameState) {
        case MTF_FRAME_STATE_WAIT_START:
            if (c == MTF_FRAME_SYNC_BYTE) {
                mtfFrameState = MTF_FRAME_STATE_READING_HEADER;
            }
            mtfDistValue = 5;
            break;

        case MTF_FRAME_STATE_READING_HEADER:
            mtfHeader[mtfReceivePosition++] = c;
            if (mtfReceivePosition == MTF_HEADER_LENGTH) {
                // Header complete, extract payload length
                mtfPayloadLength = mtfHeader[4];  // len field is 5th byte (index 4)
                
                // Validate payload length
                if (mtfPayloadLength > MTF_MAX_PAYLOAD_LENGTH) {
                    mtfFrameState = MTF_FRAME_STATE_WAIT_START;
                    mtfReceivePosition = 0;
                } else {
                    mtfFrameState = MTF_FRAME_STATE_READING_PAYLOAD;
                    mtfReceivePosition = 0;
                }
            }
            mtfDistValue = 50;
            break;

        case MTF_FRAME_STATE_READING_PAYLOAD:
            mtfPayload[mtfReceivePosition++] = c;
            if (mtfReceivePosition == mtfPayloadLength) {
                mtfFrameState = MTF_FRAME_STATE_WAIT_CKSUM;
            }
            mtfDistValue = 500;
            break;

        case MTF_FRAME_STATE_WAIT_CKSUM: 
            {
                uint8_t cksum = MTF_FRAME_SYNC_BYTE;
                for (int i = 0; i < MTF_HEADER_LENGTH; i++) {
                    cksum += mtfHeader[i];
                }
                for (int i = 0; i < mtfPayloadLength; i++) {
                    cksum += mtfPayload[i];
                }

                mtfDistValue = 1000;
                if (c == cksum) {
                    uint8_t msg_id = mtfHeader[2];  // msg_id is 3rd byte (index 2)
                    
                    // Only process range sensor messages
                    if (msg_id == MTF_MSG_ID_RANGE_SENSOR) {
                        switch (mtfDevtype) {
                        case MTF_DEVTYPE_02:
                            {
                                // Payload structure: 4 bytes time, 4 bytes dist, 1 strength, 1 precision, 1 status, 1 reserved, 2 velX, 2 velY, 1 quality, 1 flow_status, 2 reserved
                                // Indices: time=[0-3], dist=[4-7], strength=[8], precision=[9], dist_status=[10], reserved=[11], velX=[12-13], velY=[14-15], quality=[16], flow_status=[17]
                                
                                uint8_t distStatus = mtfPayload[10];
                                uint32_t distValue = mtfPayload[4] | (mtfPayload[5] << 8) | (mtfPayload[6] << 16) | (mtfPayload[7] << 24);

                                if (distStatus != 0 && distValue >= MTF_02_RANGE_MIN && distValue < MTF_02_RANGE_MAX) {
                                    mtfDistValue = distValue;
                                } else {
                                    mtfDistValue = UINT32_MAX;
                                }
                                mtfDistStrength = mtfPayload[8];
                                mtfDistPrecision = mtfPayload[9];
                                mtfDistStatus = distStatus;

                                mtfVelX = (int16_t)(mtfPayload[12] | (mtfPayload[13] << 8));
                                mtfVelY = (int16_t)(mtfPayload[14] | (mtfPayload[15] << 8));
                                mtfFlowQuality = mtfPayload[16];
                                mtfFlowStatus = mtfPayload[17];
                            }
                            break;
                        }
                    }

                    lastFrameReceivedMs = timeNowMs;
                } else {
                    mtferrors++;
                }
            }

            mtfFrameState = MTF_FRAME_STATE_WAIT_START;
            mtfReceivePosition = 0;

            break;
        }
    }

    if (timeNowMs - lastFrameReceivedMs > MTF_TIMEOUT_MS) {
        // who cares?
    }
}

optrangeRangefinderData_t mtfGetRangefinderData(optrangeDev_t *dev)
{
    UNUSED(dev);

    optrangeRangefinderData_t data;
    data.distValue = mtfDistValue;
    data.distStrength = mtfDistStrength;
    data.distPrecision = mtfDistPrecision;
    data.distStatus = mtfDistStatus;
    return data;
}

optrangeFlowData_t mtfGetFlowData(optrangeDev_t *dev)
{
    UNUSED(dev);

    optrangeFlowData_t data;
    data.velX = mtfVelX;
    data.velY = mtfVelY;
    data.flowQuality = mtfFlowQuality;
    data.flowStatus = mtfFlowStatus;
    return data;
}

static bool mtfDetect(optrangeDev_t *dev, uint8_t devType)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_OPTRANGE);

    if (!portConfig) {
        return false;
    }

    mtfSerialPort = openSerialPort(portConfig->identifier, FUNCTION_OPTRANGE, NULL, NULL, 115200, MODE_RXTX, 0);

    if (mtfSerialPort == NULL) {
        return false;
    }

    mtfDevtype = devType;

    dev->delayMs = 10;
    dev->maxRangeCm = MTF_02_RANGE_MAX / 10;
    dev->detectionConeDeciDegrees = MTF_DETECTION_CONE_DECIDEGREES;
    dev->detectionConeExtendedDeciDegrees = MTF_DETECTION_CONE_DECIDEGREES;

    dev->init = &mtfInit;
    dev->update = &mtfUpdate;
    dev->readRangefinder = &mtfGetRangefinderData;
    dev->readFlow = &mtfGetFlowData;

    return true;
}

bool mtf02Detect(optrangeDev_t *dev)
{
    return mtfDetect(dev, MTF_DEVTYPE_02);
}

#endif