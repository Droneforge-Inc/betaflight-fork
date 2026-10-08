/* Blackbox observations only. No logged value feeds back into DF3.
 * One field list defines the snapshot indices and Blackbox field names. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef USE_AP_AUTONOMY
// AP schema 5: SI P/V/A and radian angles/rates x1000; quaternion and normalized
// thrust/PID terms x1e6; ages in microseconds. See ap_autonomy/ap_blackbox.h.
#define DF3_BLACKBOX_FIELDS(F) \
    F(AP_SCHEMA, "apSchema", COMMON) \
    F(AP_SAMPLE, "apSample", COMMON) \
    F(AP_CONTROL_AGE, "apControlAgeUs", COMMON) \
    F(AP_STATE_AGE, "apStateAgeUs", COMMON) \
    F(AP_FILTER_FLAGS, "apFilterFlags", COMMON) \
    F(AP_ESTIMATE_VALID, "apEstimateValid", COMMON) \
    F(AP_MODE, "apMode", COMMON) \
    F(AP_ACTIVE, "apActive", COMMON) \
    F(AP_ARMED, "apArmed", COMMON) \
    F(AP_AUTHORITY, "apAuthority", COMMON) \
    F(AP_REFERENCE_VALID, "apRefValid", COMMON) \
    F(AP_REFERENCE_EPOCH, "apRefEpoch", COMMON) \
    F(AP_REFERENCE_SEQUENCE, "apRefSeq", COMMON) \
    F(AP_REFERENCE_AGE, "apRefAgeUs", COMMON) \
    F(AP_REFERENCE_RX_AGE, "apRefRxAgeUs", COMMON) \
    F(AP_IMU_AGE, "apImuAgeUs", COMMON) \
    F(AP_RANGE_AGE, "apRangeAgeUs", COMMON) \
    F(AP_FLOW_AGE, "apFlowAgeUs", COMMON) \
    F(AP_GPS_AGE, "apGpsAgeUs", COMMON) \
    F(AP_POSITION_X, "apPx", LATERAL) \
    F(AP_POSITION_Y, "apPy", LATERAL) \
    F(AP_POSITION_Z, "apPz", VERTICAL) \
    F(AP_VELOCITY_X, "apVx", LATERAL) \
    F(AP_VELOCITY_Y, "apVy", LATERAL) \
    F(AP_VELOCITY_Z, "apVz", VERTICAL) \
    F(AP_ACCELERATION_X, "apAx", LATERAL) \
    F(AP_ACCELERATION_Y, "apAy", LATERAL) \
    F(AP_ACCELERATION_Z, "apAz", VERTICAL) \
    F(AP_REFERENCE_P_X, "apRefPx", LATERAL) \
    F(AP_REFERENCE_P_Y, "apRefPy", LATERAL) \
    F(AP_REFERENCE_P_Z, "apRefPz", VERTICAL) \
    F(AP_REFERENCE_V_X, "apRefVx", LATERAL) \
    F(AP_REFERENCE_V_Y, "apRefVy", LATERAL) \
    F(AP_REFERENCE_V_Z, "apRefVz", VERTICAL) \
    F(AP_REFERENCE_A_X, "apRefAx", LATERAL) \
    F(AP_REFERENCE_A_Y, "apRefAy", LATERAL) \
    F(AP_REFERENCE_A_Z, "apRefAz", VERTICAL) \
    F(AP_REFERENCE_YAW, "apRefYaw", COMMON) \
    F(AP_QW, "apQw", COMMON) \
    F(AP_QX, "apQx", COMMON) \
    F(AP_QY, "apQy", COMMON) \
    F(AP_QZ, "apQz", COMMON) \
    F(AP_RATE_X, "apRateX", COMMON) \
    F(AP_RATE_Y, "apRateY", COMMON) \
    F(AP_RATE_Z, "apRateZ", COMMON) \
    F(AP_ACCEL_TARGET_X, "apAccelTargetX", LATERAL) \
    F(AP_ACCEL_TARGET_Y, "apAccelTargetY", LATERAL) \
    F(AP_ACCEL_TARGET_Z, "apAccelTargetZ", VERTICAL) \
    F(AP_THRUST, "apThrust", VERTICAL) \
    F(AP_HOVER, "apHover", VERTICAL) \
    F(AP_ACCEL_PID_P, "apAccelPidP", VERTICAL) \
    F(AP_ACCEL_PID_I, "apAccelPidI", VERTICAL) \
    F(AP_RAW_ACCEL_SUM_Z, "apRawAccelSumZ", VERTICAL) \
    F(AP_INPUT_TIME_SUM, "apInputTimeSumUs", VERTICAL) \
    F(AP_ACCEL_BIAS_X, "apAccelBiasX", LATERAL) \
    F(AP_ACCEL_BIAS_Y, "apAccelBiasY", LATERAL) \
    F(AP_ACCEL_BIAS_Z, "apAccelBiasZ", VERTICAL) \
    F(AP_PREFILTER_FORCE_X, "apPrefilterForceX", LATERAL) \
    F(AP_PREFILTER_FORCE_Y, "apPrefilterForceY", LATERAL) \
    F(AP_INPUT_DV_SUM_Z, "apInputDvSumZ", VERTICAL) \
    F(AP_VERTICAL_RATE, "apVerticalRate", VERTICAL) \
    F(AP_CONTROL_VERTICAL_RATE, "apControlVerticalRate", VERTICAL) \
    F(AP_HEIGHT_INNOVATION, "apHeightInnovation", VERTICAL) \
    F(AP_HEIGHT_TEST_RATIO, "apHeightTestRatio", VERTICAL) \
    F(AP_HEIGHT_ACCEPTANCE_AGE, "apHeightAcceptAgeUs", VERTICAL) \
    F(AP_RANGE, "apRange", VERTICAL) \
    F(AP_IMU_SAMPLES, "apImuSamples", COMMON) \
    F(AP_IMU_CLIPS, "apImuClips", COMMON) \
    F(AP_IMU_GAPS, "apImuGaps", COMMON) \
    F(AP_INPUT_ANGLE_DT, "apInputAngleDtUs", COMMON) \
    F(AP_VERTICAL_DEGRADED, "apVerticalDegraded", VERTICAL) \
    F(AP_EKF_VERTICAL_RATE, "apEkfVerticalRate", VERTICAL) \
    F(AP_INPUT_DT, "apInputDtUs", VERTICAL) \
    F(AP_INPUT_IMU_AGE, "apInputImuAgeUs", VERTICAL) \
    F(AP_INPUT_RANGE_AGE, "apInputRangeAgeUs", VERTICAL) \
    F(AP_INPUT_FRESH, "apInputFresh", VERTICAL) \
    F(AP_INPUT_DVX, "apInputDvx", VERTICAL) \
    F(AP_INPUT_DVY, "apInputDvy", VERTICAL) \
    F(AP_INPUT_DVZ, "apInputDvz", VERTICAL) \
    F(AP_INPUT_DAX, "apInputDax", VERTICAL) \
    F(AP_INPUT_DAY, "apInputDay", VERTICAL) \
    F(AP_INPUT_DAZ, "apInputDaz", VERTICAL) \
    F(AP_FUSION_AGE, "apFusionAgeUs", VERTICAL) \
    F(AP_HEIGHT_SAMPLE_AGE, "apHeightSampleAgeUs", VERTICAL) \
    F(AP_HEIGHT_FUSION_SEQUENCE, "apHeightFusionSeq", VERTICAL) \
    F(AP_DIAGNOSTIC_FLAGS, "apDiagFlags", VERTICAL) \
    F(AP_HEIGHT_OBSERVATION, "apHeightObsZ", VERTICAL) \
    F(AP_HEIGHT_VARIANCE, "apHeightObsVar", VERTICAL) \
    F(AP_OUTPUT_DV_SUM_Z, "apOutputDvSumZ", VERTICAL) \
    F(AP_OUTPUT_CORRECTION_SUM_Z, "apOutputCorrectionSumZ", VERTICAL) \
    F(AP_CONTROLLER_ACCEL, "apControllerAz", VERTICAL) \
    F(AP_IMU_SATURATION_COUNTS, "apImuSaturationCounts", VERTICAL) \
    F(AP_OUTPUT_DELTA_VELOCITY, "apOutputDvZ", VERTICAL) \
    F(AP_COMPLEMENTARY_ACCEL, "apCompAccelZ", VERTICAL)
#else
#define DF3_BLACKBOX_FIELDS(F) \
    F(SAMPLE, "df3Sample", COMMON) \
    F(AGE, "df3AgeUs", COMMON) \
    F(FLAGS, "df3Flags", COMMON) \
    F(REFERENCE_AGE, "df3RefAgeUs", COMMON) \
    F(REFERENCE_P, "df3RefPz", VERTICAL) \
    F(REFERENCE_V, "df3RefVz", VERTICAL) \
    F(REFERENCE_A, "df3RefAz", VERTICAL) \
    F(POSITION, "df3Pz", VERTICAL) \
    F(VELOCITY, "df3Vz", VERTICAL) \
    F(ACCELERATION, "df3Az", VERTICAL) \
    F(PROJECTED_P, "df3ProjectedPz", VERTICAL) \
    F(PROJECTED_V, "df3ProjectedVz", VERTICAL) \
    F(FUSION_AGE, "df3FusionAgeUs", COMMON) \
    F(QW, "df3Qw", COMMON) \
    F(QX, "df3Qx", COMMON) \
    F(QY, "df3Qy", COMMON) \
    F(QZ, "df3Qz", COMMON) \
    F(FORCE_X, "df3ForceX", COMMON) \
    F(FORCE_Y, "df3ForceY", COMMON) \
    F(FORCE_Z, "df3ForceZ", COMMON) \
    F(IMU_AGE, "df3ImuAgeUs", COMMON) \
    F(RANGE_RAW, "df3RangeRaw", COMMON) \
    F(RANGE_DOWN, "df3RangeDown", COMMON) \
    F(RANGE_STRENGTH, "df3RangeStrength", COMMON) \
    F(RANGE_AGE, "df3RangeAgeUs", COMMON) \
    F(RANGE_RECEIPT_AGE, "df3RangeRxAgeUs", COMMON) \
    F(TERRAIN, "df3Terrain", COMMON) \
    F(BIAS_X, "df3BiasX", COMMON) \
    F(BIAS_Y, "df3BiasY", COMMON) \
    F(BIAS_Z, "df3BiasZ", COMMON) \
    F(IMU_SEQUENCE, "df3ImuSeq", COMMON) \
    F(IMU_FUSION_AGE, "df3ImuFusionAgeUs", COMMON) \
    F(IMU_MEASUREMENT, "df3ImuZ", VERTICAL) \
    F(IMU_INNOVATION, "df3ImuInnovation", VERTICAL) \
    F(IMU_VARIANCE, "df3ImuR", VERTICAL) \
    F(IMU_DV, "df3ImuDv", VERTICAL) \
    F(IMU_DA, "df3ImuDa", VERTICAL) \
    F(IMU_STATUS, "df3ImuStatus", COMMON) \
    F(RANGE_SEQUENCE, "df3RangeSeq", VERTICAL) \
    F(RANGE_FUSION_AGE, "df3RangeFusionAgeUs", VERTICAL) \
    F(RANGE_MEASUREMENT, "df3RangeZ", VERTICAL) \
    F(RANGE_INNOVATION, "df3RangeInnovation", VERTICAL) \
    F(RANGE_VARIANCE, "df3RangeR", VERTICAL) \
    F(RANGE_DV, "df3RangeDv", VERTICAL) \
    F(RANGE_DA, "df3RangeDa", VERTICAL) \
    F(RANGE_STATUS, "df3RangeStatus", VERTICAL) \
    F(RANGE_FLAGS, "df3RangeFlags", VERTICAL) \
    F(FEEDFORWARD, "df3Ff", VERTICAL) \
    F(P_FEEDBACK, "df3P", VERTICAL) \
    F(V_FEEDBACK, "df3V", VERTICAL) \
    F(I_FEEDBACK, "df3I", VERTICAL) \
    F(ACCEL_REQUESTED, "df3AccelRequest", VERTICAL) \
    F(ACCEL_APPLIED, "df3AccelApplied", VERTICAL) \
    F(INTEGRAL, "df3Integral", VERTICAL) \
    F(HOVER, "df3Hover", VERTICAL) \
    F(THROTTLE_REQUESTED, "df3ThrottleRequest", VERTICAL) \
    F(THROTTLE_APPLIED, "df3ThrottleApplied", VERTICAL) \
    F(INTEGRATION_HELD, "df3IntegrationHeld", VERTICAL) \
    F(REFERENCE_SEQUENCE, "df3RefSeq", COMMON) \
    F(DIAGNOSTICS, "df3Diagnostics", COMMON) \
    F(LOG_DROPS, "df3LogDrops", COMMON) \
    F(IMU_ROUGHNESS, "df3ImuRoughness", VERTICAL) \
    F(EPOCH, "df3Epoch", COMMON) \
    F(QUEUE, "df3Queue", COMMON) \
    F(REFERENCE_P_X, "df3RefPx", LATERAL) \
    F(REFERENCE_V_X, "df3RefVx", LATERAL) \
    F(REFERENCE_A_X, "df3RefAx", LATERAL) \
    F(POSITION_X, "df3Px", LATERAL) \
    F(VELOCITY_X, "df3Vx", LATERAL) \
    F(ACCELERATION_X, "df3Ax", LATERAL) \
    F(PROJECTED_P_X, "df3ProjectedPx", LATERAL) \
    F(PROJECTED_V_X, "df3ProjectedVx", LATERAL) \
    F(FEEDFORWARD_X, "df3Ffx", LATERAL) \
    F(P_FEEDBACK_X, "df3Ptermx", LATERAL) \
    F(V_FEEDBACK_X, "df3Vtermx", LATERAL) \
    F(I_FEEDBACK_X, "df3Itermx", LATERAL) \
    F(ACCEL_REQUESTED_X, "df3AccelRequestx", LATERAL) \
    F(ACCEL_APPLIED_X, "df3AccelAppliedx", LATERAL) \
    F(INTEGRAL_X, "df3Integralx", LATERAL) \
    F(INTEGRATION_HELD_X, "df3IntegrationHeldx", LATERAL) \
    F(REFERENCE_P_Y, "df3RefPy", LATERAL) \
    F(REFERENCE_V_Y, "df3RefVy", LATERAL) \
    F(REFERENCE_A_Y, "df3RefAy", LATERAL) \
    F(POSITION_Y, "df3Py", LATERAL) \
    F(VELOCITY_Y, "df3Vy", LATERAL) \
    F(ACCELERATION_Y, "df3Ay", LATERAL) \
    F(PROJECTED_P_Y, "df3ProjectedPy", LATERAL) \
    F(PROJECTED_V_Y, "df3ProjectedVy", LATERAL) \
    F(FEEDFORWARD_Y, "df3Ffy", LATERAL) \
    F(P_FEEDBACK_Y, "df3Ptermy", LATERAL) \
    F(V_FEEDBACK_Y, "df3Vtermy", LATERAL) \
    F(I_FEEDBACK_Y, "df3Itermy", LATERAL) \
    F(ACCEL_REQUESTED_Y, "df3AccelRequesty", LATERAL) \
    F(ACCEL_APPLIED_Y, "df3AccelAppliedy", LATERAL) \
    F(INTEGRAL_Y, "df3Integraly", LATERAL) \
    F(INTEGRATION_HELD_Y, "df3IntegrationHeldy", LATERAL) \
    F(ROLL_REQUESTED, "df3RollRequest", LATERAL) \
    F(PITCH_REQUESTED, "df3PitchRequest", LATERAL) \
    F(IMU_MEASUREMENT_X, "df3ImuX", LATERAL) \
    F(IMU_INNOVATION_X, "df3ImuInnovationX", LATERAL) \
    F(IMU_VARIANCE_X, "df3ImuRX", LATERAL) \
    F(IMU_DV_X, "df3ImuDvX", LATERAL) \
    F(IMU_DA_X, "df3ImuDaX", LATERAL) \
    F(IMU_MEASUREMENT_Y, "df3ImuY", LATERAL) \
    F(IMU_INNOVATION_Y, "df3ImuInnovationY", LATERAL) \
    F(IMU_VARIANCE_Y, "df3ImuRY", LATERAL) \
    F(IMU_DV_Y, "df3ImuDvY", LATERAL) \
    F(IMU_DA_Y, "df3ImuDaY", LATERAL) \
    F(IMU_ROUGHNESS_XY, "df3ImuRoughnessXY", LATERAL) \
    F(FLOW_SEQUENCE, "df3FlowSeq", LATERAL) \
    F(FLOW_FUSION_AGE, "df3FlowFusionAgeUs", LATERAL) \
    F(FLOW_STATUS, "df3FlowStatus", LATERAL) \
    F(FLOW_MEASUREMENT_X, "df3FlowX", LATERAL) \
    F(FLOW_INNOVATION_X, "df3FlowInnovationX", LATERAL) \
    F(FLOW_VARIANCE_X, "df3FlowRX", LATERAL) \
    F(FLOW_DV_X, "df3FlowDvX", LATERAL) \
    F(FLOW_DA_X, "df3FlowDaX", LATERAL) \
    F(FLOW_MEASUREMENT_Y, "df3FlowY", LATERAL) \
    F(FLOW_INNOVATION_Y, "df3FlowInnovationY", LATERAL) \
    F(FLOW_VARIANCE_Y, "df3FlowRY", LATERAL) \
    F(FLOW_DV_Y, "df3FlowDvY", LATERAL) \
    F(FLOW_DA_Y, "df3FlowDaY", LATERAL) \
    F(FLOW_RAW_X, "df3FlowRawX", LATERAL) \
    F(FLOW_ROTATION_X, "df3FlowRotationX", LATERAL) \
    F(FLOW_LEVER_X, "df3FlowLeverX", LATERAL) \
    F(FLOW_RAW_Y, "df3FlowRawY", LATERAL) \
    F(FLOW_ROTATION_Y, "df3FlowRotationY", LATERAL) \
    F(FLOW_LEVER_Y, "df3FlowLeverY", LATERAL) \
    F(FLOW_AGE, "df3FlowAgeUs", LATERAL) \
    F(FLOW_INTERVAL, "df3FlowIntervalUs", LATERAL) \
    F(FLOW_QUALITY, "df3FlowQuality", LATERAL) \
    F(FLOW_CLEARANCE, "df3FlowClearance", LATERAL) \
    F(FLOW_ACCEPTED, "df3FlowAccepted", LATERAL) \
    F(FLOW_REJECTED, "df3FlowRejected", LATERAL) \
    F(FLOW_GYRO_P, "df3FlowGyroP", LATERAL) \
    F(FLOW_GYRO_Q, "df3FlowGyroQ", LATERAL) \
    F(NATIVE_YAW, "df3NativeYaw", COMMON) \
    F(YAW_REFERENCE, "df3YawReference", COMMON) \
    F(YAW, "df3Yaw", COMMON) \
    F(YAW_RATE, "df3YawRate", COMMON) \
    F(OUTPUT_REASON, "df3OutputReason", COMMON) \
    F(OUTPUT_FAULT_REASON, "df3OutputFaultReason", COMMON) \
    F(OUTPUT_FAULT_MS, "df3OutputFaultMs", COMMON) \
    F(OUTPUT_FAULT_NOMINAL_AGE, "df3OutputFaultNominalAgeUs", COMMON) \
    F(OUTPUT_FAULT_PHASE, "df3OutputFaultPhase", COMMON) \
    F(OUTPUT_FAULT_COUNT, "df3OutputFaultCount", COMMON) \
    F(CONTROL_FAULT_OUTPUT_REASON, "df3ControlFaultOutputReason", COMMON)
#endif

#define DF3_BLACKBOX_INDEX(id, name, group) DF3_BB_##id,
enum { DF3_BLACKBOX_FIELDS(DF3_BLACKBOX_INDEX) DF3_BLACKBOX_FIELD_COUNT };
#undef DF3_BLACKBOX_INDEX

// Control quantities are per local-NED axis. The candidate I contribution can
// differ from the retained integral when antiwindup prevents accumulation.
typedef struct {
    float reference[3], terms[4]; // P/V/A; FF, Kp*ep, Kv*ev, Ki*candidateIntegral
    float requestedAccel, integral;
    bool integrationHeld;
} df3ControlAxisTrace_t;

typedef struct {
    df3ControlAxisTrace_t axis[3];
    float requestedThrottle, yawReference;
} df3ControlTrace_t;

typedef struct {
    uint64_t sensorUs;
    uint32_t sequence;
    int status; // -1: policy skipped/not yet completed; otherwise df3Status_e
    // Measurement/residual/R use the observation frame; corrections use NED.
    float measurement[3], innovation[3], variance[3], deltaV[3], deltaA[3];
} df3FusionTraceSample_t;

typedef struct {
    // First invalid output after a valid armed output, retained until disarm.
    // Sampling Blackbox more slowly than DF3 must not erase a short failure.
    uint64_t firstFailureUs;
    int32_t nominalAgeUs;
    uint32_t reason, firstReason, failures, phase, controlFaultReason;
    bool sawValid, controlFault;
} df3OutputTrace_t;

typedef struct {
    df3FusionTraceSample_t imu, range, flow;
    float imuRoughness, imuRoughnessXY;
    uint32_t rangeFlags; // mode | hasPosition<<2 | plausible<<3 | surfaceLocked<<4
    df3OutputTrace_t output;
} df3FusionTrace_t;
