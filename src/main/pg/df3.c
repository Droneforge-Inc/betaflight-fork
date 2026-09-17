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
                  .hover = 0, .accelToThrottle = 264, .kp = {4000, 4000, 14720}, .kv = {3600, 3600, 7256},
                  .ki = {400, 400, 5781}, .maxTiltDeg = 20);
#endif
