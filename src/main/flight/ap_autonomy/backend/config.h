#pragma once

// Load libc declarations before redirecting AP's allocation expressions. This
// keeps libstdc++'s <cstdlib> namespace imports intact.
#include <cstdlib>
#include <stdlib.h>
#include "runtime.h"
#include "checkpoint.h"
#ifndef AP_BACKEND_ALLOCATOR_IMPL
#define malloc ap_backend_malloc
#define calloc ap_backend_calloc
#define free ap_backend_free
#endif

// Upstream standalone DAL accepts Betaflight sensor snapshots. One coherent
// configuration is shared by the estimator and controller in the static build.
#define CONFIG_HAL_BOARD 99
#define HAL_PROGRAM_SIZE_LIMIT_KB 512
#define APM_BUILD_DIRECTORY 10
#define HAL_NAVEKF3_AVAILABLE 1
#define EK3_FEATURE_LOCAL_YAW_BOOTSTRAP 1
// Qualify the indoor port over flat floors before enabling custom terrain
// rejection again. Research builds may opt in explicitly.
#ifndef EK3_FEATURE_DF_TERRAIN
#define EK3_FEATURE_DF_TERRAIN 0
#endif
#if defined(USE_DF3_BLACKBOX) && !defined(EK3_FEATURE_DF_DIAGNOSTICS)
#define EK3_FEATURE_DF_DIAGNOSTICS 1
#endif
#define HAL_NAVEKF2_AVAILABLE 0
#define HAL_EKF_IMU_MASK_DEFAULT 1
#define INS_MAX_INSTANCES 1
#define INS_AUX_INSTANCES 0
#define HAL_WITH_EKF_DOUBLE 0
#define HAL_WITH_DSP 0
#define HAL_NUM_CAN_IFACES 0
#define HAL_GCS_ENABLED 0
#define HAL_LOGGING_ENABLED 0
#define AP_SCRIPTING_ENABLED 0
#define AP_SIM_ENABLED 0
#define AP_AIRSPEED_ENABLED 0
#define AP_BEACON_ENABLED 0
#define HAL_VISUALODOM_ENABLED 0
#define AP_OPTICALFLOW_ENABLED 1
#define AP_RANGEFINDER_ENABLED 1
#define AP_GPS_ENABLED 1
#define AP_COMPASS_ENABLED 1
#define AP_BARO_ENABLED 1
#define AP_VEHICLE_ENABLED 0
#define AP_TERRAIN_AVAILABLE 0
#define AP_FENCE_ENABLED 0
#define AP_MISSION_ENABLED 0
#define AP_RALLY_ENABLED 0
#define AP_CANMANAGER_ENABLED 0
#define AP_NETWORKING_ENABLED 0
#define AP_DDS_ENABLED 0
#define AP_FOLLOW_ENABLED 0
#define AP_MOUNT_ENABLED 0
#define AP_WINCH_ENABLED 0
#define HAL_MSP_ENABLED 0
#define AP_EXTERNAL_AHRS_ENABLED 0
#define AP_AHRS_ENABLED 0
#define AP_AHRS_DCM_ENABLED 0
#define AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED 0
#define AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED 0
#define AP_GPS_MB_YAW_OFFSET_ENABLED 0
#define EK3_FEATURE_BODY_ODOM 0
// Only the external yaw observation is used for the local heading bootstrap.
// Position/velocity remain optical-flow/range or GPS/barometer observations.
#define EK3_FEATURE_EXTERNAL_NAV 0
#define EK3_FEATURE_EXTERNAL_YAW 1
#define EK3_FEATURE_DRAG_FUSION 0
#define EK3_FEATURE_BEACON_FUSION 0
#define EK3_FEATURE_POSITION_RESET 0
#define EK3_FEATURE_OPTFLOW_SRTM 0
#define EK3_FEATURE_OPTFLOW_SAMPLE_TIMESTAMP 1
#define EK3_FEATURE_MOVING_BASELINE 0
#define AP_CUSTOMROTATIONS_ENABLED 0
#define AP_PARAM_MAX_EMBEDDED_PARAM 0
#define AP_FILTER_ENABLED 0
#define __AP_LINE__ __LINE__
#include <AP_Common/AP_Common.h>
#include <AP_HAL/Semaphores.h>
namespace Empty {
class Semaphore : public AP_HAL::Semaphore {
public:
    bool give() override { return true; }
    bool take(uint32_t) override { return true; }
    bool take_nonblocking() override { return true; }
    bool check_owner() const { return true; }
};
}
