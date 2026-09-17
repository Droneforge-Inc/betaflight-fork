#pragma once

/* Internal reset kernel: disjoint, 4-byte-aligned P[18*18], J[3*3], tmp[18*18].
 * Preserves the C reset operation order and its exact scratch write footprint.
 * Cortex-M4F/AAPCS only; caller supplies the existing covariance workspace. */
void df3ResetCovarianceM4f(float P[18 * 18], const float J[9], float tmp[18 * 18]);
