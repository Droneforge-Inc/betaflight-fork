// Execute the real CRSF ISR/task dispatcher with the portable reference checker.
#include <cstring>
#include <vector>
#include "gtest/gtest.h"
extern "C" {
#include "platform.h"
#include "build/debug.h"
#include "common/crc.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"
#include "drivers/serial.h"
#include "io/serial.h"
#include "rx/rx.h"
#include "rx/crsf.h"
#include "flight/df3/df3_mlrs.h"
#include "msp/msp.h"
#include "msp/msp_protocol.h"
#include "telemetry/msp_shared.h"
void crsfDataReceive(uint16_t, void *);
uint8_t crsfFrameStatus(rxRuntimeState_t *);
extern uint32_t crsfChannelData[CRSF_MAX_CHANNEL];
PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
rssiSource_e rssiSource;
int16_t debug[DEBUG16_VALUE_COUNT];
}

static uint32_t nowUs, txFree, openedBaud;
static unsigned writes, acceptedControls, uidRequests, mspRequests;
static uint8_t mspReplyDestination;
static uint16_t mspCommand;
static std::vector<uint8_t> queuedMsp;
static bool profile, armed, autonomy, controlSeen;
static df3ReferenceReceiver_t reference;
static uint8_t previous[32];
static std::vector<uint8_t> transmitted;
static serialPort_t port;
static serialPortVTable vtable;
static serialPortConfig_t portConfig;

extern "C" {
uint32_t micros(void) { return nowUs; }
uint32_t microsISR(void) { return nowUs; }
serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr,
                            void *, uint32_t baud, portMode_e, portOptions_e) { openedBaud = baud; return &port; }
const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e) { return &portConfig; }
bool telemetryCheckRxPortShared(const serialPortConfig_t *) { return false; }
serialPort_t *telemetrySharedPort;
void crsfScheduleDeviceInfoResponse(void) {}
void crsfScheduleMspResponse(uint8_t destination) { mspReplyDestination = destination; }
void crsfRecordUidRequestReceived(void) { ++uidRequests; }
bool bufferCrsfMspFrame(uint8_t *p, int count) { queuedMsp.assign(p, p + count); return true; }
mspDescriptor_t mspDescriptorAlloc(void) { return 0; }
mspResult_e mspFcProcessCommand(mspDescriptor_t, mspPacket_t *cmd, mspPacket_t *reply, mspPostProcessFnPtr *)
{
    ++mspRequests;
    mspCommand = cmd->cmd;
    reply->cmd = cmd->cmd;
    return MSP_RESULT_ACK;
}
bool isBatteryVoltageAvailable(void) { return true; }
bool isAmperageAvailable(void) { return true; }
timeUs_t rxFrameTimeUs(void) { return 0; }
// FC control-task boundary. Codec/freshness acceptance below is production code;
// estimator/control dependencies are intentionally absent from a dispatcher test.
void df3BetaflightMlrsProfile(bool active, bool newSession)
{
    profile = active;
    if (newSession && !armed) {
        df3ReferenceReset(&reference);
        memset(previous, 0, sizeof(previous));
        autonomy = controlSeen = false;
    }
    if (!active && !armed) controlSeen = false;
}
bool df3BetaflightMlrsLegacyAllowed(void) { return !profile && !(armed && controlSeen); }
bool df3BetaflightMlrsControl(const uint8_t *p, uint32_t receivedUs)
{
    if (!profile || !df3MlrsReferenceAccept(&reference, previous, p, receivedUs, armed)) return false;
    autonomy = p[1] & DF3_MLRS_AUTONOMY;
    controlSeen = true;
    ++acceptedControls;
    return true;
}
void df3BetaflightReferenceFrame(const uint8_t *, uint32_t) {}
void df3BetaflightTelemetryQueued(uint32_t, bool) {}
void df3BetaflightUartSubmitted(uint32_t, uint8_t) {}
}

class MlrsDispatch : public ::testing::Test {
protected:
    rxRuntimeState_t runtime{};
    void SetUp() override {
        armed = false;
        nowUs += 2000000;
        crsfRxMlrsProfileActive(nowUs); // Expire the prior fixture's session.
        profile = autonomy = controlSeen = false;
        df3ReferenceReset(&reference);
        memset(previous, 0, sizeof(previous));
        acceptedControls = writes = uidRequests = mspRequests = 0;
        mspCommand = mspReplyDestination = 0;
        queuedMsp.clear();
        initSharedMsp();
        transmitted.clear();
        txFree = 256;
        vtable = {};
        vtable.serialTotalTxFree = [](const serialPort_t *) { return txFree; };
        vtable.writeBuf = [](serialPort_t *, const void *data, int count) {
            ++writes;
            const auto *bytes = static_cast<const uint8_t *>(data);
            transmitted.insert(transmitted.end(), bytes, bytes + count);
        };
        port.vTable = &vtable;
        ASSERT_TRUE(crsfRxInit(rxConfig(), &runtime));
        ASSERT_EQ(420000u, openedBaud);
    }
    void feed(uint8_t type, std::vector<uint8_t> payload, uint32_t gapUs = 2000, uint32_t byteUs = 0) {
        nowUs += gapUs;
        std::vector<uint8_t> frame{0xc8, uint8_t(payload.size() + 2), type};
        frame.insert(frame.end(), payload.begin(), payload.end());
        uint8_t crc = 0;
        for (size_t i = 2; i < frame.size(); ++i) crc = crc8_dvb_s2(crc, frame[i]);
        frame.push_back(crc);
        for (const auto byte : frame) {
            nowUs += byteUs;
            crsfDataReceive(byte, &runtime);
        }
    }
    void probe(uint8_t origin = 0xee, uint32_t nonce = 0x12345678) {
        std::vector<uint8_t> p{0xc8, origin, 1, 0, 1, 0, 0, 0, 0, 0};
        df3WriteU32Le(p.data() + 6, nonce);
        feed(0xe8, p);
        crsfFrameStatus(&runtime);
    }
    std::vector<uint8_t> control(uint16_t channel, uint8_t flags = 3, uint16_t seq = 1) {
        std::vector<uint8_t> p(56);
        p[0] = 0xc8; p[1] = 0xea;
        p[2] = channel; p[3] = channel >> 8;
        uint8_t *r = p.data() + 24;
        r[0] = 1; r[1] = flags;
        df3WriteU16Le(r + 2, 9); df3WriteU16Le(r + 4, seq);
        df3WriteU32Le(r + 6, nowUs / 1000);
        df3WriteU16Le(r + 30, 200);
        return p;
    }
};

TEST_F(MlrsDispatch, NegotiationRejectsWrongOriginZeroNonceAndExpires)
{
    probe(0xea); EXPECT_FALSE(profile);
    probe(0xee, 0); EXPECT_FALSE(profile);
    probe(); ASSERT_TRUE(profile);
    crsfRxMlrsProfileReply();
    ASSERT_EQ(14u, transmitted.size());
    EXPECT_EQ(0xee, transmitted[0]); EXPECT_EQ(0xee, transmitted[3]); EXPECT_EQ(0xc8, transmitted[4]);
    EXPECT_EQ(1, transmitted[6]);
    EXPECT_EQ(0x5678, df3ReadU16Le(transmitted.data() + 9));
    EXPECT_EQ(0x1234, df3ReadU16Le(transmitted.data() + 11));
    nowUs += 1500001;
    crsfFrameStatus(&runtime);
    EXPECT_FALSE(profile);
}

TEST_F(MlrsDispatch, CompositeCommitRejectAndRepeatDoNotRenewLease)
{
    probe();
    auto p = control(1024);
    feed(0xe9, p);
    EXPECT_EQ(RX_FRAME_COMPLETE, crsfFrameStatus(&runtime));
    ASSERT_EQ(1024u, crsfChannelData[0]); ASSERT_TRUE(autonomy);
    const uint32_t acceptedUs = runtime.lastRcFrameTimeUs;
    const uint64_t leaseUs = reference.receivedUs;
    auto malformed = p;
    malformed[24] = 2; malformed[2] = 99;
    feed(0xe9, malformed);
    EXPECT_EQ(RX_FRAME_PENDING, crsfFrameStatus(&runtime));
    EXPECT_EQ(acceptedUs, runtime.lastRcFrameTimeUs);
    EXPECT_EQ(1024u, crsfChannelData[0]); EXPECT_EQ(1u, acceptedControls);
    malformed = p; malformed[1] = 0xee; // Wrong origin cannot reach the FC adapter.
    feed(0xe9, malformed);
    EXPECT_EQ(RX_FRAME_PENDING, crsfFrameStatus(&runtime));
    malformed = p; malformed.pop_back(); // Even a CRC-valid short control is rejected.
    feed(0xe9, malformed);
    EXPECT_EQ(RX_FRAME_PENDING, crsfFrameStatus(&runtime));
    EXPECT_EQ(acceptedUs, runtime.lastRcFrameTimeUs);
    EXPECT_EQ(1u, acceptedControls);
    p[2] = 1300 & 255; p[3] = 1300 >> 8;
    feed(0xe9, p);
    EXPECT_EQ(RX_FRAME_COMPLETE, crsfFrameStatus(&runtime));
    EXPECT_EQ(1300u, crsfChannelData[0]);
    EXPECT_GT(runtime.lastRcFrameTimeUs, acceptedUs);
    EXPECT_EQ(leaseUs, reference.receivedUs);
    EXPECT_EQ(1u, reference.accepted);
}

TEST_F(MlrsDispatch, ExpiryCannotInjectManualOrLegacyRcWhileArmed)
{
    probe(); feed(0xe9, control(1024));
    ASSERT_EQ(RX_FRAME_COMPLETE, crsfFrameStatus(&runtime));
    armed = true;
    const uint32_t acceptedUs = runtime.lastRcFrameTimeUs;
    feed(0x16, std::vector<uint8_t>(22, 0));
    EXPECT_EQ(RX_FRAME_PENDING, crsfFrameStatus(&runtime));
    EXPECT_EQ(acceptedUs, runtime.lastRcFrameTimeUs);
    nowUs += 1500001;
    crsfFrameStatus(&runtime);
    ASSERT_FALSE(profile); ASSERT_TRUE(autonomy);
    feed(0x16, std::vector<uint8_t>(22, 0));
    EXPECT_EQ(RX_FRAME_PENDING, crsfFrameStatus(&runtime));
    EXPECT_EQ(acceptedUs, runtime.lastRcFrameTimeUs);
    EXPECT_EQ(1u, acceptedControls);
    EXPECT_TRUE(autonomy);
    probe(); feed(0xe9, control(1100, 0, 2));
    EXPECT_EQ(RX_FRAME_COMPLETE, crsfFrameStatus(&runtime));
    EXPECT_FALSE(autonomy); EXPECT_EQ(1100u, crsfChannelData[0]);
}

TEST_F(MlrsDispatch, UartAdmitsBothSnapshotPartsOrNeither)
{
    uint8_t snapshot[66] = {1}, frames[98];
    df3MlrsSnapshotFrames(snapshot, frames);
    txFree = 97;
    EXPECT_FALSE(crsfRxTryWriteTelemetry(frames, sizeof(frames)));
    EXPECT_EQ(0u, writes); EXPECT_TRUE(transmitted.empty());
    txFree = 98;
    EXPECT_TRUE(crsfRxTryWriteTelemetry(frames, sizeof(frames)));
    EXPECT_EQ(1u, writes); EXPECT_EQ(98u, transmitted.size());
}

TEST_F(MlrsDispatch, BackToBackControlAndUidAt420000Baud)
{
    probe();
    // The radio writes a 60-byte E9 then a 12-byte MSP UID with no gap.
    // 24 us per 10-bit UART byte rounds up the actual 420000 baud period.
    const auto p = control(1152, 0);
    const uint32_t started = nowUs;
    feed(0xe9, p, 0, 24);
    feed(0x7a, {0xc8, 0xea, 0x50, 0, MSP_UID, 0, 0, 0}, 0, 24);
    EXPECT_EQ(72u * 24, nowUs - started);
    EXPECT_EQ(1u, uidRequests);
    EXPECT_EQ(0xea, mspReplyDestination);
    const std::vector<uint8_t> expected{0x50, 0, MSP_UID, 0, 0, 0};
    ASSERT_EQ(expected, queuedMsp);
    // MSP must not overwrite the atomic RC/reference pending frame.
    EXPECT_EQ(RX_FRAME_COMPLETE, crsfFrameStatus(&runtime));
    EXPECT_EQ(1152u, crsfChannelData[0]);
    EXPECT_EQ(1u, acceptedControls);
    // The actual shared MSP decoder accepts the exact six-byte radio payload.
    ASSERT_TRUE(handleMspFrame(queuedMsp.data(), queuedMsp.size(), nullptr));
    EXPECT_EQ(1u, mspRequests);
    EXPECT_EQ(MSP_UID, mspCommand);
}
