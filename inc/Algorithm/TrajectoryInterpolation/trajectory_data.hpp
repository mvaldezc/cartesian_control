/***********************************************************************
 * @file	:	trajectory_data.hpp
 * @brief 	:	Trajectory Data Library
 * 				Library with message types used to transmit trajectory data
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

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

typedef struct // size: (6 words = 24 bytes = 112 bits)
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
    unsigned int vel_x_0 : 16;  // (1/20) step/sec, max 3200 steps/sec
    unsigned int vel_x_f : 16;  // (1/20) step/sec, max 3200 steps/sec
    unsigned int vel_y_0 : 16;  // (1/20) step/sec, max 3200 steps/sec
    unsigned int vel_y_f : 16;  // (1/20) step/sec, max 3200 steps/sec
    unsigned int vel_z_0 : 16;  // (1/20) step/sec, max 3200 steps/sec
    unsigned int vel_z_f : 16;  // (1/20) step/sec, max 3200 steps/sec
} path_params_extended_t;
