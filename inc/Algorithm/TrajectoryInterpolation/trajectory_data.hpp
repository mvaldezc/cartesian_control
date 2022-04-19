/***********************************************************************
 * @file	:	trajectory_data.hpp
 * @brief 	:	Trajectory Data Library
 * 				Library with message types used to transmit trajectory data
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

#pragma once

// Machine step size: 0.025 mm (1/40 mm)

typedef struct // size: (4 words = 16 bytes = 128 bits)
{
    unsigned int path_type : 5; // 32 types of movement
    unsigned int dir_x : 1;     // 0 is CCW, 1 is CW
    unsigned int dir_y : 1;     // 0 is CCW, 1 is CW
    unsigned int dir_z : 1;     // 0 is CCW, 1 is CW
    unsigned int time : 20;     // (1/4) ms, max 4 min for a single movement
    unsigned int : 0;           // 4 bit padding
    unsigned int pos_x : 16;    // machine steps, max 1600 mm / 64000 steps / 320 revs
    unsigned int pos_y : 16;    // machine steps, max 1600 mm / 64000 steps / 320 revs
    unsigned int pos_z : 16;    // machine steps, max 1600 mm / 64000 steps / 320 revs
    unsigned int vel_x_f : 16;  // (1/20) step/sec, max 3200 steps/sec
    unsigned int vel_y_f : 16;  // (1/20) step/sec, max 3200 steps/sec
    unsigned int vel_z_f : 16;  // (1/20) step/sec, max 3200 steps/sec
} path_params_t;
