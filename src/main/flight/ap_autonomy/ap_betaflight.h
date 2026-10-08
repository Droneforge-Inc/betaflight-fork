#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float force_mps2[3]; // Latest pre-output-filter specific force, body FRD.
    uint32_t sample_count;
    uint32_t clip_count; // Samples with any raw axis near the ADC rail.
    uint32_t gap_count; // Invalid, stale or discontinuous acquisition intervals.
    uint32_t last_interval_us;
} apImuDiagnostics_t;

void apAutonomyImuDiagnostics(apImuDiagnostics_t *output);
struct gyroDev_s;
void apAutonomyGyroSample(const struct gyroDev_s *gyro);

#ifdef SITL
struct dfsim_input_v6_s;
void apAutonomySitlAiding(const struct dfsim_input_v6_s *input);
#endif

// Autonomous targets are converted from FRD rad/s to native BF deg/s here.
float apAutonomyRateSetpoint(unsigned axis);
float apAutonomyMotorCommand(float thrust);
float apAutonomyMotorThrust(float actuator);
void apAutonomyMotorLimits(bool lower, bool upper, bool attitude, float achievedThrust);
bool apAutonomyUsesLocalHeading(void);
bool apAutonomyReady(void);
