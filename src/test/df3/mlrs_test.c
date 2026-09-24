#include "flight/df3/df3_mlrs.h"
#include "common/df_custom.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void command(uint8_t *p, uint8_t flags, uint16_t epoch, uint16_t sequence, uint32_t source)
{
    memset(p, 0, 32);
    p[0] = 1; p[1] = flags;
    df3WriteU16Le(p + 2, epoch);
    df3WriteU16Le(p + 4, sequence);
    df3WriteU32Le(p + 6, source);
    df3WriteU16Le(p + 30, 200);
}

int main(void)
{
    uint8_t capability[12];
    memset(capability, 0x5a, sizeof(capability));
    assert(!df3MlrsCapabilities(1, capability));
    assert(capability[0] == 0x5a);
    assert(!df3MlrsCapabilities(0, NULL));
    assert(df3MlrsCapabilities(0, capability));
    assert(capability[0] == 1 && capability[1] == 1 && capability[2] == 1 && capability[3] == 0);
    assert(df3ReadU16Le(capability + 4) == (FIRMWARE_VERSION_DF & 0xffff));
    assert(df3ReadU16Le(capability + 6) == (FIRMWARE_VERSION_DF >> 16));
    assert(df3ReadU16Le(capability + 8) == (HARDWARE_VERSION_DF & 0xffff));
    assert(df3ReadU16Le(capability + 10) == (HARDWARE_VERSION_DF >> 16));
    df3ReferenceReceiver_t receiver = {0};
    uint8_t p[32], previous[32] = {0}, legacy[32];
    for (unsigned flags = 0; flags < 4; ++flags) {
        command(p, flags, 0x1234, 0x5678, 0x12345678);
        assert(df3MlrsReferenceDecode(p, legacy));
        assert(legacy[1] == ((flags & 1) ? 0 : 1));
        assert(legacy[2] == 0x12 && legacy[3] == 0x34);
        assert(legacy[6] == 0x12 && legacy[7] == 0x34 && legacy[8] == 0x56 && legacy[9] == 0x78);
    }
    command(p, 3, 1, 65535, 1000);
    df3WriteU16Le(p + 10, (uint16_t)-1234);
    assert(df3MlrsReferenceAccept(&receiver, previous, p, 1000000, false));
    assert(receiver.reference.position[0] < -12.33f && receiver.reference.position[0] > -12.35f);
    assert(df3MlrsReferenceAccept(&receiver, previous, p, 1400000, true));
    assert(receiver.receivedUs == 1000000 && receiver.accepted == 1);
    assert(df3MlrsReferenceFresh(&receiver, 1200000));
    assert(!df3MlrsReferenceFresh(&receiver, 1200001));
    df3Reference_t sampled;
    assert(!df3ReferenceSample(&receiver, 1400000, &sampled));
    p[1] = 0; // Same sequence cannot change selection or its payload.
    assert(!df3MlrsReferenceAccept(&receiver, previous, p, 1400000, true));
    command(p, 0, 1, 0, 1410); // sequence wrap and explicit manual takeover
    assert(df3MlrsReferenceAccept(&receiver, previous, p, 1410000, true));
    assert(!receiver.reference.active);
    assert(df3MlrsReferenceFresh(&receiver, 1420000));
    command(p, 3, 2, 1, 1420);
    assert(!df3MlrsReferenceAccept(&receiver, previous, p, 1420000, true));
    assert(df3MlrsReferenceAccept(&receiver, previous, p, 1420000, false));
    command(p, 2, 2, 2, 1430);
    p[10] = 1;
    assert(!df3MlrsReferenceDecode(p, legacy));
    p[10] = 0; p[1] = 4;
    assert(!df3MlrsReferenceDecode(p, legacy));
    p[1] = 2; p[30] = 49;
    assert(!df3MlrsReferenceDecode(p, legacy));
    assert(df3MlrsAge10ms(0, true) == 0);
    assert(df3MlrsAge10ms(1, true) == 1);
    assert(df3MlrsAge10ms(2540000, true) == 254);
    assert(df3MlrsAge10ms(2540001, true) == 255);
    assert(df3MlrsAge10ms(0, false) == 255);

    uint32_t next = 1000;
    unsigned generated = 0;
    for (uint32_t now = 1000; now < 1001000; now += 2000) generated += df3MlrsSnapshotDue(now, &next);
    assert(generated == 50);
    assert(df3MlrsSnapshotDue(1101000, &next) && next == 1121000);
    assert(!df3MlrsSnapshotDue(1101000, &next));
    next = UINT32_MAX - 9999;
    assert(df3MlrsSnapshotDue(next, &next));
    assert(next == 10000 && !df3MlrsSnapshotDue(9999, &next));
    assert(df3MlrsSnapshotDue(10000, &next));

    uint8_t snapshot[66], frames[98];
    for (unsigned i = 0; i < sizeof(snapshot); ++i) snapshot[i] = i;
    df3MlrsSnapshotFrames(snapshot, frames);
    for (unsigned part = 0; part < 2; ++part) {
        const uint8_t *f = frames + 49 * part;
        assert(f[0] == 0xee && f[1] == 47 && f[2] == 0xea && f[3] == 0xee && f[4] == 0xc8);
        assert(f[5] == 1 && f[6] == part && !memcmp(f + 7, snapshot + 2, 8));
        assert(!memcmp(f + 15, snapshot + 33 * part, 33));
    }
    // Independent golden DVB-S2 CRC values for the exact byte ramp snapshot.
    assert(frames[48] == 0xbe && frames[97] == 0xe7);
    puts("DF3 mLRS: control, freshness, snapshot and 50 Hz scheduling tests passed");
    return 0;
}
