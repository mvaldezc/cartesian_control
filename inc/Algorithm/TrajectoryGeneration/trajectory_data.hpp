/***********************************************************************
 * @file	:	trajectory_data.hpp
 * @brief 	:	Trajectory Data Types
 * 				Defines data types containing trajectory specifications.
 * @author	:	Marco Valdez @marcovc41
 *
 ***********************************************************************/

#pragma once
#include <list>
#include <memory>

namespace Algorithm::TrajectoryGeneration {

// Machine step size: 0.025 mm (1/40 mm) (1/200 rev)

struct path_params_t// size: (2.5 words = 10 bytes = 80 bits)
{
    unsigned int path_type : 5; // 32 types of movement/interpolations
    unsigned int dir_x : 1;     // 0 is CCW, 1 is CW
    unsigned int dir_y : 1;     // 0 is CCW, 1 is CW
    unsigned int dir_z : 1;     // 0 is CCW, 1 is CW
    unsigned int time : 22;     // (1/4) ms, max 17 min for a single movement
    unsigned int : 0;           // 2 bit padding
    unsigned int pos_x : 16;    // machine steps, max 1600 mm / 64000 steps / 320 revs
    unsigned int pos_y : 16;    // machine steps, max 1600 mm / 64000 steps / 320 revs
    unsigned int pos_z : 16;    // machine steps, max 1600 mm / 64000 steps / 320 revs
};

} // namespace Algorithm::TrajectoryGeneration

using path_list_t = std::shared_ptr<std::list<Algorithm::TrajectoryGeneration::path_params_t>>;
