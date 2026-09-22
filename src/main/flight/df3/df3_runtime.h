/* Production DF3 runtime contract, included by mk/df3.mk for C and ARM kernels.
 * USE_DF3 selects the complete runtime. Optional USE_DF3_PROFILE adds timing
 * diagnostics; USE_DF3_BLACKBOX adds controller/fusion traces to Blackbox.
 * Both diagnostics are off by default.
 * The internal definitions below keep historical implementations available to
 * standalone comparison tests and research.mk, which do not include this file.
 * They are not independent production options.
 */
#pragma once

#if defined(USE_DF3_BLACKBOX) && !defined(USE_DF3)
#error "USE_DF3_BLACKBOX requires USE_DF3"
#endif

#ifdef USE_DF3
#define USE_DF3_RESUMABLE
#define USE_DF3_MULTIRATE
#define USE_DF3_BUDGETED_WORKER
#define DF3_FUSION_HZ 30

#ifndef SITL
#define USE_DF3_BUDGET_CYCLES
#endif

#ifdef USE_DF3_PROFILE
#define USE_DF3_PROFILE_AUTOSTART
#endif
#endif
