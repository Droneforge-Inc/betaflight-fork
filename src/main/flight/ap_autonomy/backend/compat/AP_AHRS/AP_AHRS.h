#pragma once

// The static backend exposes one AHRS snapshot type. Keeping the upstream
// frontend declaration here would give AP::ahrs() two incompatible return types
// across estimator and controller translation units.
#include "AP_AHRS_View.h"
