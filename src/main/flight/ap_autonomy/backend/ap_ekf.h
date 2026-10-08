#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    AP_EKF_RANGE = 1u << 0,
    AP_EKF_FLOW = 1u << 1,
    AP_EKF_GPS = 1u << 2,
    AP_EKF_BARO = 1u << 3,
    AP_EKF_COMPASS = 1u << 4,
    AP_EKF_HEADING = 1u << 5,
};

typedef struct {
    uint64_t time_us;
    float dt_s; /* Acceleration integration interval and EKF update interval. */
    /* Gyro integration interval; zero means dt_s for equal-rate callers. */
    float delta_angle_dt_s;
    /* Integrated IMU increments, body forward/right/down. Specific force includes gravity. */
    float delta_angle_rad[3];
    float delta_velocity_mps[3];
    uint32_t fresh;
    uint8_t armed;
    uint8_t takeoff_expected;
    uint8_t flow_quality;
    uint8_t gps_fix;
    /* Downward slant range and raw angular flow (not gyro compensated). */
    float range_m;
    float sensor_position_body_m[3];
    float flow_rad_s[2];
    float flow_gyro_rad_s[2];
    /* Acquisition-window midpoint in the same clock as time_us, not receipt time. */
    uint32_t flow_time_ms;
    float baro_altitude_m;
    float magnetic_field_mgauss[3];
    int32_t latitude_e7;
    int32_t longitude_e7;
    int32_t altitude_cm;
    float gps_velocity_ned_mps[3];
    float gps_horizontal_accuracy_m;
    float gps_vertical_accuracy_m;
    float gps_speed_accuracy_mps;
    float gps_lag_s;
    uint8_t gps_satellites;
    /* Indoor bootstrap only: body yaw in the local horizontal/down frame, rad.
     * Acquired from BF attitude at heading_time_ms, with positive 1-sigma error.
     * This establishes a local heading; it is not a true-north measurement.
     * Heading packets are ignored after the first successful alignment.
     */
    float heading_rad;
    float heading_error_rad;
    uint32_t heading_time_ms;
    /* ArduCopter's !has_manual_throttle(): AP is selected to control collective,
     * distinct from armed (manual flight must not activate vibration fallback).
     */
    uint8_t automatic_throttle;
} ap_ekf_input_t;

typedef struct {
    uint64_t time_us;
    /* Body-to-navigation quaternion, scalar first. SI units, horizontal/down
     * axes: bootstrapped local frame indoors, true NED outdoors.
     */
    float quaternion[4];
    float position_ned_m[3];
    float velocity_ned_mps[3];
    float acceleration_ned_mps2[3];
    float accel_bias_body_mps2[3];
    float gyro_bias_body_rads[3];
    /* Genuine upstream complementary vertical rate, not velocity_ned_mps[2]. */
    float vertical_position_rate_mps;
    float height_innovation_m;
    float height_test_ratio; /* squared normalized innovation; >1 rejects */
    uint32_t height_acceptance_age_ms; /* last height pass/reset; UINT32_MAX before either */
    uint8_t vertical_degraded; /* ArduCopter vibration compensation is active */
    /* Custom terrain outputs stay zero when the policy is compiled out. */
    float terrain_offset_down_m; /* surface position in the EKF NED datum */
    uint8_t terrain_transition; /* current range classified as terrain, not vehicle motion */
    uint8_t terrain_expired; /* prolonged terrain ambiguity has exhausted the coast allowance */
    uint32_t terrain_transition_age_ms;
    uint32_t filter_status;
    uint16_t faults;
    uint8_t initialized;
    uint8_t healthy;
    uint8_t heading_aligned;
    /* Upstream reset event counters. No reset delta is implied. */
    uint16_t position_ne_reset_count;
    uint16_t position_d_reset_count;
    uint16_t yaw_reset_count;
} ap_ekf_output_t;

enum {
    AP_EKF_DIAG_ENABLED = 1u << 0,
    AP_EKF_DIAG_CORE_UPDATED = 1u << 1,
    AP_EKF_DIAG_HEIGHT_ATTEMPT = 1u << 2,
    AP_EKF_DIAG_HEIGHT_FUSED = 1u << 3,
    AP_EKF_DIAG_HEIGHT_EVENT = 1u << 4,
    AP_EKF_DIAG_BAD_IMU = 1u << 5,
    AP_EKF_DIAG_BIAS_INHIBITED = 1u << 6,
    AP_EKF_DIAG_BIAS_Z_INHIBITED = 1u << 7,
    AP_EKF_DIAG_TERRAIN = 1u << 8,
    AP_EKF_DIAG_PREDICTED = 1u << 9,
    AP_EKF_DIAG_HEIGHT_SAMPLE = 1u << 10,
    AP_EKF_DIAG_HEIGHT_RANGE = 1u << 11,
    AP_EKF_DIAG_EVENT_BAD_IMU = 1u << 16,
    AP_EKF_DIAG_EVENT_BIAS_INHIBITED = 1u << 17,
    AP_EKF_DIAG_EVENT_BIAS_Z_INHIBITED = 1u << 18,
    AP_EKF_DIAG_EVENT_TERRAIN = 1u << 19,
};

/* Optional diagnostics; no estimator input/output ABI changes. All signs use
 * navigation down.
 * Height fields describe the last successful scalar height fusion, held until
 * the next event. HEIGHT_EVENT validates its fusion horizon; HEIGHT_SAMPLE
 * validates the sensor timestamp (both in FC/DAL milliseconds). Sequence starts
 * at one after enabling/reset and wraps as uint32_t. EVENT_* flags are held with
 * that event; EVENT_BIAS_INHIBITED means its actual Z-bias Kalman gain was masked.
 * Other status flags describe the current completed core update. HEIGHT_ATTEMPT
 * is the innovation check, HEIGHT_FUSED requires successful FinishFusion.
 * TERRAIN includes transition or expired ambiguity. PREDICTED means the delayed
 * EKF equations ran; CORE_UPDATED means the current output propagation ran.
 * Output P/V deltas are this update's additive observer corrections, zero if
 * none; output_delta_velocity is the bias/gravity-corrected propagation increment.
 * Complementary acceleration is its total rate-feedback acceleration, including
 * the integrated position error. It is not the measured acceleration.
 */
typedef struct {
    uint32_t fusion_time_ms;
    uint32_t height_sample_time_ms;
    uint32_t height_fusion_sequence;
    uint32_t flags;
    float height_observation_down_m;
    float height_variance_m2;
    float output_position_delta_down_m;
    float output_velocity_delta_down_mps;
    float output_delta_velocity_down_mps;
    float complementary_accel_correction_mps2;
} ap_ekf_vertical_diagnostics_t;

/* Changing enable state invalidates the snapshot. Getter returns 1 only after
 * a completed update while enabled, otherwise 0 and leaves *output untouched.
 * Both functions are safe when diagnostics were compiled out (getter returns 0).
 * Calls share the estimator's single-threaded ownership contract.
 */
__attribute__((visibility("default"))) void ap_ekf_set_vertical_diagnostics(int enabled);
__attribute__((visibility("default"))) int ap_ekf_get_vertical_diagnostics(ap_ekf_vertical_diagnostics_t *output);

/* Single-threaded, one estimator per process. Calls return 0 on invalid input.
 * indoor != 0 selects flow/range and initial local heading; otherwise
 * GPS/barometer/compass in true NED. Indoor output uses the seeded local frame.
 * healthy is the upstream overall check; inspect filter_status for each required
 * navigation axis and heading_aligned. Do not fly on healthy alone. Position datum is EKF3's datum,
 * so a ground station local origin must be established from a valid state.
 */
__attribute__((visibility("default"))) int ap_ekf_init(unsigned imu_rate_hz, int indoor);
__attribute__((visibility("default"))) int ap_ekf_update(const ap_ekf_input_t *input, ap_ekf_output_t *output);

#ifdef __cplusplus
}
#endif
