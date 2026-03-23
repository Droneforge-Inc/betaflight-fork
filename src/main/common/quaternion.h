#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef union {
    float raw[4];
    struct {
        float w;
        float x;
        float y;
        float z;
    };
} quaternion_t;

#define QUATERNION_INITIALIZE  {.w=1, .x=0, .y=0, .z=0}

#ifdef __cplusplus
}
#endif
