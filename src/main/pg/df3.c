#include "platform.h"
#ifdef USE_DF3
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "df3.h"
PG_REGISTER_WITH_RESET_TEMPLATE(df3Config_t, df3Config, PG_DF3_CONFIG, 0);
PG_REGISTER_WITH_RESET_TEMPLATE(df3CalibrationConfig_t, df3CalibrationConfig, PG_DF3_CALIBRATION_CONFIG, 0);
PG_RESET_TEMPLATE(df3CalibrationConfig_t, df3CalibrationConfig, .enabled = 0);
PG_REGISTER_WITH_RESET_TEMPLATE(df3FlowConfig_t, df3FlowConfig, PG_DF3_FLOW_CONFIG, 0);
// Preserve legacy hardware calibration until measured for that sensor/mount.
// The rendered simulator uses rotationScale=1000 and sensorOffset={0,0,35}.
PG_RESET_TEMPLATE(df3FlowConfig_t, df3FlowConfig, .rotationScale = 758, .sensorOffset = {0, 0, 0});
PG_RESET_TEMPLATE(df3Config_t, df3Config,
                  // No motor authority until the aircraft's collective calibration is set.
                  .hover = 0, .accelToThrottle = 264,
                  // Fixed Riccati gains restored from the FC 1.3.22 flight profile; SI units * 1000.
                  .kp = {1208, 1208, 2987}, .kv = {1641, 1641, 2642}, .ki = {105, 105, 501}, .maxTiltDeg = 20);
#endif
