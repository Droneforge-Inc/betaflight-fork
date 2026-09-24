/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <string.h>

extern "C" {
#include "platform.h"
#include "drivers/compass/compass.h"
#include "drivers/compass/compass_lis2mdl.h"
}

#include "gtest/gtest.h"

static uint8_t sample[6];
static bool readStartsSuccessfully = true;

TEST(CompassLis2mdlTest, ConvertsSensorAxesAfterSuccessfulRead)
{
    magDev_t mag = {};
    mag.dev.bus = &mag.bus;
    mag.bus.busType = BUS_TYPE_I2C;
    ASSERT_TRUE(lis2mdlDetect(&mag));
    EXPECT_EQ(0x1e, mag.dev.busType_u.i2c.address);

    int16_t output[3] = {11, 22, 33};
    readStartsSuccessfully = false;
    EXPECT_FALSE(mag.read(&mag, output));
    EXPECT_EQ(11, output[0]);
    EXPECT_EQ(22, output[1]);
    EXPECT_EQ(33, output[2]);
    readStartsSuccessfully = true;

    const struct {
        int16_t sensor[3];
        int16_t expected[3];
    } cases[] = {
        {{1234, -2345, 3456}, {-1234, -2345, 3456}},
        {{-1234, 2345, -3456}, {1234, 2345, -3456}},
        {{0, 0, 0}, {0, 0, 0}},
        {{32767, -32768, 32767}, {-32767, -32768, 32767}},
        {{-32767, 32767, -32768}, {32767, 32767, -32768}},
    };

    for (const auto &testCase : cases) {
        for (unsigned axis = 0; axis < 3; axis++) {
            const uint16_t raw = static_cast<uint16_t>(testCase.sensor[axis]);
            sample[2 * axis] = raw & 0xff;
            sample[2 * axis + 1] = raw >> 8;
        }

        EXPECT_FALSE(mag.read(&mag, output));
        ASSERT_TRUE(mag.read(&mag, output));
        for (unsigned axis = 0; axis < 3; axis++) {
            EXPECT_EQ(testCase.expected[axis], output[axis]) << "axis " << axis;
        }
    }
}

extern "C" {

bool busReadRegisterBuffer(const extDevice_t *, uint8_t reg, uint8_t *data, uint8_t length)
{
    EXPECT_EQ(0x4f, reg);
    EXPECT_EQ(1, length);
    data[0] = 0x40;
    return true;
}

bool busReadRegisterBufferStart(const extDevice_t *, uint8_t reg, uint8_t *data, uint8_t length)
{
    EXPECT_EQ(0x68, reg);
    EXPECT_EQ(sizeof(sample), length);
    if (readStartsSuccessfully) {
        memcpy(data, sample, sizeof(sample));
    }
    return readStartsSuccessfully;
}

bool busWriteRegister(const extDevice_t *, uint8_t, uint8_t) { return true; }
void busDeviceRegister(const extDevice_t *) {}
void delay(uint32_t) {}

}
