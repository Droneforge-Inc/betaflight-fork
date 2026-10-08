#include "ap_ekf.h"
#include "ap_terrain.h"
#include "ap_vibration.h"
#include "runtime.h"
#include "profile.h"

#include <AP_DAL/AP_DAL.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_NavEKF/AP_Nav_Common.h>
#include <AP_Math/AP_Math.h>
#include <cstring>

static_assert(INS_MAX_INSTANCES == 1 && INS_AUX_INSTANCES == 0,
              "The Betaflight AP bridge supplies one IMU stream");

static NavEKF3 *ekf;
static bool initialized;
static bool indoor_mode;
static bool heading_submitted;
static bool local_yaw;
static unsigned imu_rate;
static uint64_t last_time_us;
static uint32_t sensors_seen;
static uint64_t range_received_us;
static APVibrationPolicy vibration_policy;
#if EK3_FEATURE_DF_DIAGNOSTICS
static bool vertical_diagnostics_enabled;
static bool vertical_diagnostics_updated;
#endif

void ap_ekf_set_vertical_diagnostics(int enabled)
{
#if EK3_FEATURE_DF_DIAGNOSTICS
    if (vertical_diagnostics_enabled != (enabled != 0)) {
        vertical_diagnostics_enabled = enabled != 0;
        vertical_diagnostics_updated = false;
        if (ekf) {
            ekf->setVerticalDiagnosticsEnabled(vertical_diagnostics_enabled);
        }
    }
#else
    (void)enabled;
#endif
}

int ap_ekf_get_vertical_diagnostics(ap_ekf_vertical_diagnostics_t *output)
{
#if EK3_FEATURE_DF_DIAGNOSTICS
    if (!output || !vertical_diagnostics_updated) {
        return 0;
    }
    const auto *diagnostics = ekf->getVerticalDiagnostics();
    if (!diagnostics) {
        return 0;
    }

    output->fusion_time_ms = diagnostics->fusion_time_ms;
    output->height_sample_time_ms = diagnostics->height_sample_time_ms;
    output->height_fusion_sequence = diagnostics->height_fusion_sequence;
    output->flags = diagnostics->flags;
    output->height_observation_down_m = diagnostics->height_observation_down_m;
    output->height_variance_m2 = diagnostics->height_variance_m2;
    output->output_position_delta_down_m = diagnostics->output_position_delta_down_m;
    output->output_velocity_delta_down_mps = diagnostics->output_velocity_delta_down_mps;
    output->output_delta_velocity_down_mps = diagnostics->output_delta_velocity_down_mps;
    output->complementary_accel_correction_mps2 = diagnostics->complementary_accel_correction_mps2;
    return 1;
#else
    (void)output;
    return 0;
#endif
}


static Vector3f vector3(const float value[3]) {
    return Vector3f(value[0], value[1], value[2]);
}

static void store3(float output[3], const Vector3f &value) {
    output[0] = value.x;
    output[1] = value.y;
    output[2] = value.z;
}

static bool set_parameter(void *object, const AP_Param::GroupInfo *group, const char *name, float value) {
    for (const auto *entry = group; entry->type != AP_PARAM_NONE; ++entry) {
        auto *address = static_cast<uint8_t *>(object) + entry->offset;
        if (entry->type == AP_PARAM_GROUP) {
            const size_t prefix = strlen(entry->name);
            if (strncmp(name, entry->name, prefix) == 0 &&
                set_parameter(address, entry->group_info, name + prefix, value)) {
                return true;
            }
        } else if (strcmp(name, entry->name) == 0) {
            AP_Param::set_value(ap_var_type(entry->type), address, value);
            return true;
        }
    }
    return false;
}

int ap_ekf_init(unsigned imu_rate_hz, int indoor) {
    if (ekf || imu_rate_hz < 100 || imu_rate_hz > 4000) {
        return 0;
    }
    ap_backend_runtime_init();
    imu_rate = imu_rate_hz;
    indoor_mode = indoor != 0;
    ekf = NEW_NOTHROW NavEKF3;
    if (!ekf) {
        return 0;
    }
#if EK3_FEATURE_DF_DIAGNOSTICS
    ekf->setVerticalDiagnosticsEnabled(vertical_diagnostics_enabled);
#endif
    const struct { const char *name; float value; } parameters[] = {
        {"IMU_MASK", 1},
        {"SRC1_POSXY", indoor_mode ? 0.0f : 3.0f},
        {"SRC1_VELXY", indoor_mode ? 5.0f : 3.0f},
        {"SRC1_POSZ", indoor_mode ? 2.0f : 1.0f},
        {"SRC1_VELZ", indoor_mode ? 0.0f : 3.0f},
        {"SRC1_YAW", indoor_mode ? 6.0f : 1.0f},
        // The opt-in timestamp path uses the supplied integration-window midpoint.
        {"FLOW_DELAY", 0},
        // Shared XYZ bias settings. The 0.04 process-noise trial exceeds the
        // documented 0.02 range; EKF3's computation permits it (clamp 0..1).
        {"ACC_BIAS_LIM", 2.0f},
        {"ABIAS_P_NSE", 0.04f},
    };
    for (const auto &parameter : parameters) {
        if (!set_parameter(ekf, NavEKF3::var_info, parameter.name, parameter.value)) {
            AP_HAL::panic("EKF parameter missing: %s", parameter.name);
        }
    }
    // Range-primary indoor flight tracks a local floor offset. Keep height aiding
    // credible under motor-on IMU disturbances: reducing range noise alone can
    // otherwise reject the good range against an overconfident inertial state.
    if (indoor_mode &&
        (!set_parameter(ekf, NavEKF3::var_info, "RNG_M_NSE", 0.1f) ||
         !set_parameter(ekf, NavEKF3::var_info, "ACC_P_NSE", 3.0f))) {
        AP_HAL::panic("Indoor EKF noise parameters missing");
    }
    return 1;
}

int ap_ekf_update(const ap_ekf_input_t *input, ap_ekf_output_t *output) {
#if EK3_FEATURE_DF_DIAGNOSTICS
    vertical_diagnostics_updated = false;
#endif
    if (!ekf || !input || !output || input->time_us <= last_time_us ||
        !std::isfinite(input->dt_s) || input->dt_s <= 0 || input->dt_s > 0.05f) {
        return 0;
    }
    const float angle_dt = input->delta_angle_dt_s == 0 ? input->dt_s : input->delta_angle_dt_s;
    if (!std::isfinite(angle_dt) || angle_dt <= 0 || angle_dt > 0.05f) {
        return 0;
    }
    for (unsigned axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(input->delta_angle_rad[axis]) || !std::isfinite(input->delta_velocity_mps[axis])) {
            return 0;
        }
    }
    const bool bootstrap_heading = indoor_mode && !input->armed && !heading_submitted && (input->fresh & AP_EKF_HEADING);
    if (bootstrap_heading && (!std::isfinite(input->heading_rad) ||
        !std::isfinite(input->heading_error_rad) || input->heading_error_rad <= 0 ||
        int32_t(uint32_t(input->time_us / 1000) - input->heading_time_ms) < 0 ||
        uint32_t(input->time_us / 1000) - input->heading_time_ms > 200)) {
        return 0;
    }
    const uint32_t prepStarted = apProfileBegin(AP_PROF_EKF_PREP);
    ap_backend_set_time_us(input->time_us);
    last_time_us = input->time_us;
    sensors_seen |= input->fresh;
    auto &dal = AP::dal();
    log_RFRH frame{};
    frame.time_us = input->time_us;
    dal.handle_message(frame);
    log_RFRN navigation{};
    navigation.available_memory = ap_backend_available_memory();
    navigation.EAS2TAS = 1;
    navigation.vehicle_class = uint8_t(AP_DAL::VehicleClass::COPTER);
    navigation.ekf_type = 3;
    navigation.armed = input->armed;
    navigation.takeoff_expected = input->takeoff_expected;
    navigation.opticalflow_enabled = indoor_mode;
    dal.handle_message(navigation);
    log_RISH imu_header{};
    imu_header.loop_rate_hz = imu_rate;
    imu_header.loop_delta_t = input->dt_s;
    imu_header.accel_count = imu_header.gyro_count = 1;
    dal.handle_message(imu_header);
    log_RISI imu{};
    imu.delta_angle = vector3(input->delta_angle_rad);
    imu.delta_velocity = vector3(input->delta_velocity_mps);
    imu.delta_angle_dt = angle_dt;
    imu.delta_velocity_dt = input->dt_s;
    imu.use_accel = imu.use_gyro = imu.get_delta_angle_ret = imu.get_delta_velocity_ret = true;
    dal.handle_message(imu);
    apAutonomyCheckpoint();

    if (input->fresh & AP_EKF_RANGE) {
        range_received_us = input->time_us;
        log_RRNH header{};
        header.ground_clearance = 0.065f;
        header.max_distance = 8.0f;
        header.num_sensors = 1;
        dal.handle_message(header);
        log_RRNI range{};
        range.pos_offset = vector3(input->sensor_position_body_m);
        range.distance = input->range_m;
        range.orientation = ROTATION_PITCH_270;
        range.status = uint8_t(std::isfinite(input->range_m) && input->range_m > 0
            ? RangeFinder::Status::Good : RangeFinder::Status::NoData);
        dal.handle_message(range);
    } else if (range_received_us && input->time_us - range_received_us > 500000) {
        log_RRNI range{};
        range.orientation = ROTATION_PITCH_270;
        range.status = uint8_t(RangeFinder::Status::NoData);
        dal.handle_message(range);
    }
    if (input->fresh & AP_EKF_BARO) {
        log_RBRH header{};
        header.num_instances = 1;
        dal.handle_message(header);
        log_RBRI baro{};
        baro.last_update_ms = input->time_us / 1000;
        baro.altitude = input->baro_altitude_m;
        baro.healthy = true;
        dal.handle_message(baro);
    }
    if (!indoor_mode && (input->fresh & AP_EKF_COMPASS)) {
        log_RMGH header{};
        header.count = header.num_enabled = 1;
        header.available = header.consistent = true;
        dal.handle_message(header);
        log_RMGI compass{};
        compass.field = vector3(input->magnetic_field_mgauss);
        compass.last_update_usec = input->time_us;
        compass.use_for_yaw = compass.healthy = compass.have_scale_factor = true;
        dal.handle_message(compass);
    }
    if (!indoor_mode && (input->fresh & AP_EKF_GPS)) {
        log_RGPH header{};
        header.num_sensors = 1;
        dal.handle_message(header);
        log_RGPI gps{};
        gps.status = input->gps_fix;
        gps.num_sats = input->gps_satellites;
        gps.lag_sec = input->gps_lag_s;
        gps.have_vertical_velocity = gps.horizontal_accuracy_returncode = gps.vertical_accuracy_returncode = true;
        gps.speed_accuracy_returncode = gps.get_lag_returncode = true;
        dal.handle_message(gps);
        log_RGPJ measurement{};
        measurement.last_message_time_ms = input->time_us / 1000;
        measurement.lat = input->latitude_e7;
        measurement.lng = input->longitude_e7;
        measurement.alt = input->altitude_cm;
        measurement.velocity = vector3(input->gps_velocity_ned_mps);
        measurement.hacc = input->gps_horizontal_accuracy_m;
        measurement.vacc = input->gps_vertical_accuracy_m;
        measurement.sacc = input->gps_speed_accuracy_mps;
        measurement.hdop = 100;
        dal.handle_message(measurement);
    }

    apAutonomyCheckpoint();

    // EKF3 allocates optional sensor buffers when its cores are first created.
    // Register all required DAL sensors before that one-time allocation.
    const uint32_t required = indoor_mode ? (AP_EKF_HEADING | AP_EKF_RANGE) :
        (AP_EKF_COMPASS | AP_EKF_GPS | AP_EKF_BARO);
    if (!initialized && (sensors_seen & required) == required) {
        initialized = ekf->InitialiseFilter();
        apAutonomyCheckpoint();
    }
    if (initialized) {
        const bool heading_was_aligned = ekf->yawAlignmentComplete();
        nav_filter_status current_status{};
        ekf->getFilterStatus(current_status);
        if (bootstrap_heading && current_status.flags.attitude) {
            Quaternion heading;
            heading.from_euler(0, 0, input->heading_rad);
            // Only EXTNAV yaw is selected. The required position argument is
            // unused by fusion; position/velocity continue to come from flow/GPS.
            // BF and EKF3 share a gyro: use BF only to initialize the local yaw,
            // never as a repeated independent heading observation.
            ekf->writeExtNavData(Vector3f{}, heading, 1.0f, input->heading_error_rad,
                input->heading_time_ms, 0, 0);
            heading_submitted = true;
        }
        if (input->fresh & AP_EKF_FLOW) {
            ekf->writeOptFlowMeas(input->flow_quality,
                Vector2f(input->flow_rad_s[0], input->flow_rad_s[1]),
                Vector2f(input->flow_gyro_rad_s[0], input->flow_gyro_rad_s[1]),
                input->flow_time_ms, vector3(input->sensor_position_body_m), 0);
        }
        apProfileEnd(AP_PROF_EKF_PREP, prepStarted);
        apAutonomyCheckpoint();
        const uint32_t coreStarted = apProfileBegin(AP_PROF_EKF_CORE);
        ekf->UpdateFilter();
#if EK3_FEATURE_DF_DIAGNOSTICS
        vertical_diagnostics_updated = vertical_diagnostics_enabled;
#endif
        apProfileEnd(AP_PROF_EKF_CORE, coreStarted);
        apAutonomyCheckpoint();
        // Upstream may fully reinitialize an unhealthy filter while disarmed.
        // A new local frame then needs a new genuine heading, never an in-flight
        // re-seed from BF's independently reset heading origin.
        if (indoor_mode && !input->armed && heading_was_aligned && !ekf->yawAlignmentComplete()) {
            heading_submitted = false;
            local_yaw = false;
            if (!set_parameter(ekf, NavEKF3::var_info, "SRC1_YAW", 6)) {
                AP_HAL::panic("Indoor yaw source missing");
            }
        } else if (indoor_mode && !local_yaw && ekf->yawAlignmentComplete()) {
            // BF supplies a local origin once, not a continuing external yaw
            // sensor. NONE enables upstream stationary gyro-bias learning and
            // zero-innovation covariance stabilization without reusing BF yaw.
            if (!set_parameter(ekf, NavEKF3::var_info, "SRC1_YAW", 0)) {
                AP_HAL::panic("Indoor yaw source missing");
            }
            local_yaw = true;
        }
    } else {
        apProfileEnd(AP_PROF_EKF_PREP, prepStarted);
    }

    const uint32_t readbackStarted = apProfileBegin(AP_PROF_EKF_READBACK);
    *output = {};
    output->height_acceptance_age_ms = UINT32_MAX;
    output->time_us = input->time_us;
    output->initialized = initialized;
    output->heading_aligned = initialized && ekf->yawAlignmentComplete();
    if (!initialized) {
        apProfileEnd(AP_PROF_EKF_READBACK, readbackStarted);
        return 1;
    }
    Quaternion quaternion;
    // Inputs are already in the vehicle body frame. Standalone DAL does not
    // populate AHRS's board-to-body rotation, so use the predictor quaternion
    // directly rather than applying that absent extra transform.
    ekf->getQuaternion(quaternion);
    output->quaternion[0] = quaternion.q1;
    output->quaternion[1] = quaternion.q2;
    output->quaternion[2] = quaternion.q3;
    output->quaternion[3] = quaternion.q4;
    Vector2p position_ne;
    postype_t position_d;
    ekf->getPosNE(position_ne);
    ekf->getPosD(position_d);
    output->position_ned_m[0] = position_ne.x;
    output->position_ned_m[1] = position_ne.y;
    output->position_ned_m[2] = position_d;
    Vector3f velocity, accel_bias, gyro_bias;
    ekf->getVelNED(velocity);
    ekf->getAccelBias(-1, accel_bias);
    ekf->getGyroBias(-1, gyro_bias);
    store3(output->accel_bias_body_mps2, accel_bias);
    store3(output->gyro_bias_body_rads, gyro_bias);
    output->vertical_position_rate_mps = ekf->getPosDownDerivative();
#if EK3_FEATURE_DF_TERRAIN
    ap_terrain_result_t terrain;
    ekf->getTerrainPolicy(output->terrain_offset_down_m, terrain);
    output->terrain_transition = indoor_mode && terrain.transition;
    output->terrain_expired = indoor_mode && terrain.expired;
    output->terrain_transition_age_ms = terrain.transition_age_ms;
#endif
    ekf->getHeightInnovation(output->height_innovation_m, output->height_test_ratio,
                            output->height_acceptance_age_ms);
    store3(output->velocity_ned_mps, velocity);
    apAutonomyCheckpoint();
    Matrix3f rotation;
    quaternion.rotation_matrix(rotation);
    Vector3f acceleration = rotation * (imu.delta_velocity / input->dt_s - accel_bias);
    acceleration.z += GRAVITY_MSS;
    store3(output->acceleration_ned_mps2, acceleration);
    apAutonomyCheckpoint();
    nav_filter_status status{};
    ekf->getFilterStatus(status);
    output->filter_status = status.value;
    ekf->getFilterFaults(output->faults);
    output->healthy = ekf->healthy();
    output->position_ne_reset_count = ekf->getPosNorthEastResetCount();
    output->position_d_reset_count = ekf->getPosDownResetCount();
    output->yaw_reset_count = ekf->getYawResetCount();
    if (vibration_policy.due(input->time_us)) {
        Vector3f vel_innovation{}, pos_innovation{}, mag_innovation{}, mag_variance{};
        Vector2f offset{};
        float tas_innovation = 0, yaw_innovation = 0;
        float position_variance = 0, height_variance = 0, tas_variance = 0;
        ap_vibration_input evidence{};
        evidence.innovations_valid = ekf->getInnovations(vel_innovation, pos_innovation,
            mag_innovation, tas_innovation, yaw_innovation);
        // A local indoor frame has no LLH origin. Heading alignment replaces
        // ArduCopter's get_origin() prerequisite for refreshing these ratios;
        // no alternate velocity-gap or height-innovation trigger is added.
        evidence.variances_valid = output->heading_aligned && ekf->getVariances(
            evidence.velocity_variance, position_variance, height_variance,
            mag_variance, tas_variance, offset);
        evidence.velocity_innovation_down = vel_innovation.z;
        evidence.position_innovation_down = pos_innovation.z;
        evidence.vibration_affected = ekf->isVibrationAffected();
        evidence.armed = input->armed;
        evidence.automatic_throttle = input->automatic_throttle;
        vibration_policy.update(input->time_us, evidence);
    }
    output->vertical_degraded = vibration_policy.active();
    apProfileEnd(AP_PROF_EKF_READBACK, readbackStarted);
    return 1;
}
