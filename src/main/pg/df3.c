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
#ifdef USE_DF3_BLACKBOX
PG_REGISTER_WITH_RESET_TEMPLATE(df3BlackboxConfig_t, df3BlackboxConfig, PG_DF3_BLACKBOX_CONFIG, 0);
PG_RESET_TEMPLATE(df3BlackboxConfig_t, df3BlackboxConfig, .axes = 0);
#endif
PG_RESET_TEMPLATE(df3FlowConfig_t, df3FlowConfig, .rotationScale = 758, .sensorOffset = {0, 0, 0});
PG_RESET_TEMPLATE(df3Config_t, df3Config,
                  // No motor authority until the aircraft's collective calibration is set.
                  .hover = 0, .accelToThrottle = 264,
                  // Fixed Riccati costs: XY Q=(0.064,0.016,0.00064), R=0.0285714285714.
                  // Z Q=(0.06,0.03,1/90), R=0.07375. Gains stored in SI units * 1000.
                  .kp = {1677, 1677, 1493}, .kv = {1977, 1977, 1841}, .ki = {149, 149, 387}, .maxTiltDeg = 20);
#endif
