#pragma once

typedef struct // size: (3 words = 12 bytes = 96 bits)
{
    unsigned int path_type : 5; // 32 types of movement
    unsigned int dir_x : 1;     // 0 is CCW, 1 is CW
    unsigned int dir_y : 1;     // 0 is CCW, 1 is CW
    unsigned int dir_z : 1;     // 0 is CCW, 1 is CW
    unsigned int time : 22;     // (1/4) ms, max 17 min for a single movement
    unsigned int : 0;           // 2 bit padding
    unsigned int pos_x : 16;    // steps, max 65535 steps or 320 revs
    unsigned int pos_y : 16;    // steps, max 65535 steps or 320 revs
    unsigned int pos_z : 16;    // steps, max 65535 steps or 320 revs
    unsigned int : 0;           // 16 bit padding
} path_params_t;


