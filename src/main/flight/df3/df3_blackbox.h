/* Blackbox observations only. No logged value feeds back into DF3.
 * One field list defines the snapshot indices and Blackbox field names. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define DF3_BLACKBOX_FIELDS(F) \
    F(SAMPLE, "df3Sample") \
    F(AGE, "df3AgeUs") \
    F(FLAGS, "df3Flags") \
    F(REFERENCE_AGE, "df3RefAgeUs") \
    F(REFERENCE_P, "df3RefPz") \
    F(REFERENCE_V, "df3RefVz") \
    F(REFERENCE_A, "df3RefAz") \
    F(POSITION, "df3Pz") \
    F(VELOCITY, "df3Vz") \
    F(ACCELERATION, "df3Az") \
    F(PROJECTED_P, "df3ProjectedPz") \
    F(PROJECTED_V, "df3ProjectedVz") \
    F(FUSION_AGE, "df3FusionAgeUs") \
    F(QW, "df3Qw") \
    F(QX, "df3Qx") \
    F(QY, "df3Qy") \
    F(QZ, "df3Qz") \
    F(FORCE_X, "df3ForceX") \
    F(FORCE_Y, "df3ForceY") \
    F(FORCE_Z, "df3ForceZ") \
    F(IMU_AGE, "df3ImuAgeUs") \
    F(RANGE_RAW, "df3RangeRaw") \
    F(RANGE_DOWN, "df3RangeDown") \
    F(RANGE_STRENGTH, "df3RangeStrength") \
    F(RANGE_AGE, "df3RangeAgeUs") \
    F(RANGE_RECEIPT_AGE, "df3RangeRxAgeUs") \
    F(TERRAIN, "df3Terrain") \
    F(BIAS_X, "df3BiasX") \
    F(BIAS_Y, "df3BiasY") \
    F(BIAS_Z, "df3BiasZ") \
    F(IMU_SEQUENCE, "df3ImuSeq") \
    F(IMU_FUSION_AGE, "df3ImuFusionAgeUs") \
    F(IMU_MEASUREMENT, "df3ImuZ") \
    F(IMU_INNOVATION, "df3ImuInnovation") \
    F(IMU_VARIANCE, "df3ImuR") \
    F(IMU_DV, "df3ImuDv") \
    F(IMU_DA, "df3ImuDa") \
    F(IMU_STATUS, "df3ImuStatus") \
    F(RANGE_SEQUENCE, "df3RangeSeq") \
    F(RANGE_FUSION_AGE, "df3RangeFusionAgeUs") \
    F(RANGE_MEASUREMENT, "df3RangeZ") \
    F(RANGE_INNOVATION, "df3RangeInnovation") \
    F(RANGE_VARIANCE, "df3RangeR") \
    F(RANGE_DV, "df3RangeDv") \
    F(RANGE_DA, "df3RangeDa") \
    F(RANGE_STATUS, "df3RangeStatus") \
    F(RANGE_FLAGS, "df3RangeFlags") \
    F(FEEDFORWARD, "df3Ff") \
    F(P_FEEDBACK, "df3P") \
    F(V_FEEDBACK, "df3V") \
    F(I_FEEDBACK, "df3I") \
    F(ACCEL_REQUESTED, "df3AccelRequest") \
    F(ACCEL_APPLIED, "df3AccelApplied") \
    F(INTEGRAL, "df3Integral") \
    F(HOVER, "df3Hover") \
    F(THROTTLE_REQUESTED, "df3ThrottleRequest") \
    F(THROTTLE_APPLIED, "df3ThrottleApplied") \
    F(INTEGRATION_HELD, "df3IntegrationHeld") \
    F(REFERENCE_SEQUENCE, "df3RefSeq") \
    F(DIAGNOSTICS, "df3Diagnostics") \
    F(LOG_DROPS, "df3LogDrops") \
    F(IMU_ROUGHNESS, "df3ImuRoughness") \
    F(EPOCH, "df3Epoch") \
    F(QUEUE, "df3Queue") \
    F(REFERENCE_P_X, "df3RefPx") \
    F(REFERENCE_V_X, "df3RefVx") \
    F(REFERENCE_A_X, "df3RefAx") \
    F(POSITION_X, "df3Px") \
    F(VELOCITY_X, "df3Vx") \
    F(ACCELERATION_X, "df3Ax") \
    F(PROJECTED_P_X, "df3ProjectedPx") \
    F(PROJECTED_V_X, "df3ProjectedVx") \
    F(FEEDFORWARD_X, "df3Ffx") \
    F(P_FEEDBACK_X, "df3Ptermx") \
    F(V_FEEDBACK_X, "df3Vtermx") \
    F(I_FEEDBACK_X, "df3Itermx") \
    F(ACCEL_REQUESTED_X, "df3AccelRequestx") \
    F(ACCEL_APPLIED_X, "df3AccelAppliedx") \
    F(INTEGRAL_X, "df3Integralx") \
    F(INTEGRATION_HELD_X, "df3IntegrationHeldx") \
    F(REFERENCE_P_Y, "df3RefPy") \
    F(REFERENCE_V_Y, "df3RefVy") \
    F(REFERENCE_A_Y, "df3RefAy") \
    F(POSITION_Y, "df3Py") \
    F(VELOCITY_Y, "df3Vy") \
    F(ACCELERATION_Y, "df3Ay") \
    F(PROJECTED_P_Y, "df3ProjectedPy") \
    F(PROJECTED_V_Y, "df3ProjectedVy") \
    F(FEEDFORWARD_Y, "df3Ffy") \
    F(P_FEEDBACK_Y, "df3Ptermy") \
    F(V_FEEDBACK_Y, "df3Vtermy") \
    F(I_FEEDBACK_Y, "df3Itermy") \
    F(ACCEL_REQUESTED_Y, "df3AccelRequesty") \
    F(ACCEL_APPLIED_Y, "df3AccelAppliedy") \
    F(INTEGRAL_Y, "df3Integraly") \
    F(INTEGRATION_HELD_Y, "df3IntegrationHeldy") \
    F(ROLL_REQUESTED, "df3RollRequest") \
    F(PITCH_REQUESTED, "df3PitchRequest") \
    F(IMU_MEASUREMENT_X, "df3ImuX") \
    F(IMU_INNOVATION_X, "df3ImuInnovationX") \
    F(IMU_VARIANCE_X, "df3ImuRX") \
    F(IMU_DV_X, "df3ImuDvX") \
    F(IMU_DA_X, "df3ImuDaX") \
    F(IMU_MEASUREMENT_Y, "df3ImuY") \
    F(IMU_INNOVATION_Y, "df3ImuInnovationY") \
    F(IMU_VARIANCE_Y, "df3ImuRY") \
    F(IMU_DV_Y, "df3ImuDvY") \
    F(IMU_DA_Y, "df3ImuDaY") \
    F(IMU_ROUGHNESS_XY, "df3ImuRoughnessXY") \
    F(FLOW_SEQUENCE, "df3FlowSeq") \
    F(FLOW_FUSION_AGE, "df3FlowFusionAgeUs") \
    F(FLOW_STATUS, "df3FlowStatus") \
    F(FLOW_MEASUREMENT_X, "df3FlowX") \
    F(FLOW_INNOVATION_X, "df3FlowInnovationX") \
    F(FLOW_VARIANCE_X, "df3FlowRX") \
    F(FLOW_DV_X, "df3FlowDvX") \
    F(FLOW_DA_X, "df3FlowDaX") \
    F(FLOW_MEASUREMENT_Y, "df3FlowY") \
    F(FLOW_INNOVATION_Y, "df3FlowInnovationY") \
    F(FLOW_VARIANCE_Y, "df3FlowRY") \
    F(FLOW_DV_Y, "df3FlowDvY") \
    F(FLOW_DA_Y, "df3FlowDaY") \
    F(FLOW_RAW_X, "df3FlowRawX") \
    F(FLOW_ROTATION_X, "df3FlowRotationX") \
    F(FLOW_LEVER_X, "df3FlowLeverX") \
    F(FLOW_RAW_Y, "df3FlowRawY") \
    F(FLOW_ROTATION_Y, "df3FlowRotationY") \
    F(FLOW_LEVER_Y, "df3FlowLeverY") \
    F(FLOW_AGE, "df3FlowAgeUs") \
    F(FLOW_INTERVAL, "df3FlowIntervalUs") \
    F(FLOW_QUALITY, "df3FlowQuality") \
    F(FLOW_CLEARANCE, "df3FlowClearance") \
    F(FLOW_ACCEPTED, "df3FlowAccepted") \
    F(FLOW_REJECTED, "df3FlowRejected") \
    F(FLOW_GYRO_P, "df3FlowGyroP") \
    F(FLOW_GYRO_Q, "df3FlowGyroQ")

#define DF3_BLACKBOX_INDEX(id, name) DF3_BB_##id,
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
    float requestedThrottle;
} df3ControlTrace_t;

typedef struct {
    uint64_t sensorUs;
    uint32_t sequence;
    int status; // -1: policy skipped/not yet completed; otherwise df3Status_e
    // Measurement/residual/R use the observation frame; corrections use NED.
    float measurement[3], innovation[3], variance[3], deltaV[3], deltaA[3];
} df3FusionTraceSample_t;

typedef struct {
    df3FusionTraceSample_t imu, range, flow;
    float imuRoughness, imuRoughnessXY;
    uint32_t rangeFlags; // mode | hasPosition<<2 | plausible<<3 | surfaceLocked<<4
} df3FusionTrace_t;
