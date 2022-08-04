#include <cstdio>
#include <memory>
#include "pico/stdlib.h"
#include "i2c_slave.hpp"
#include "communication_handler.hpp"
#include "cartesian_robot.hpp"
#include "state_manager.hpp"
#include "atomic.hpp"
#include "pico/multicore.h"

//================ Global variables definition ================

using namespace Algorithm::TrajectoryGeneration;
using namespace Communication;
using namespace Motor;
using namespace Sampler;
using namespace System::Robot;

void core1_resume();
void core1_start();

int main()
{
    //================= Serial initialization ==================
    stdio_init_all();
    printf("Uart init completed\n");
    
    //============== State Manager initialization ==============
    StateManager * stateManager = StateManager::getInstance();
    printf("State Machine init completed\n");
    
    //============== Data Structure instantiation ==============
    path_list_t via_points = createPathList();
    installDataContainer(via_points);
    printf("Path container init completed\n");

    //==================== Motor definition ====================
    std::shared_ptr<IMotor> motor_x(new Stepper(200, 8, 7, 6));
    std::shared_ptr<IMotor> motor_y(new Stepper(200, 7, 8, 9));
    std::shared_ptr<IMotor> motor_z(new Stepper(200, 10, 11, 12));
    printf("Motor def completed\n");

    //==================== Timer definition ====================
    constexpr uint64_t sampling_period_us = 250; 
    TimerIsrSampler motor_sampler(sampling_period_us);
    printf("Timer def completed\n");

    //============= Cartesian Robot initialization =============
    CartesianRobotClient welding_system(motor_x, motor_y, motor_z, motor_sampler);
    //CartesianRobotClient welding_system(motor_x, nullptr, nullptr, motor_sampler);
    printf("Robot init completed\n");
    
    //=================== I2C initialization ===================
    I2CSlave::init(&rxCallback, &txCallback);
    printf("I2C init completed\n");
    
    printf("System Fully Initialized\n");

    /*
    via_points->push_back(
        {.path_type = static_cast<char>(InterpolationType::AdvancedPoly),
         .dir_x = static_cast<bool>(MotorDirection::Clockwise),
         .dir_y = static_cast<bool>(MotorDirection::CounterClockwise),
         .dir_z = static_cast<bool>(MotorDirection::Clockwise),
         .time = 5 * 4000,
         .pos_x = 2000,
         .pos_y = 0,
         .pos_z = 0});
    via_points->push_back(
        {.path_type = static_cast<char>(InterpolationType::AdvancedPoly),
         .dir_x = static_cast<bool>(MotorDirection::CounterClockwise),
         .dir_y = static_cast<bool>(MotorDirection::Clockwise),
         .dir_z = static_cast<bool>(MotorDirection::Clockwise),
         .time = 5 * 4000,
         .pos_x = 4000,
         .pos_y = 0,
         .pos_z = 0});
    via_points->push_back(
        {.path_type = static_cast<char>(InterpolationType::AdvancedPoly),
         .dir_x = static_cast<bool>(MotorDirection::Clockwise),
         .dir_y = static_cast<bool>(MotorDirection::Clockwise),
         .dir_z = static_cast<bool>(MotorDirection::Clockwise),
         .time = 10 * 4000,
         .pos_x = 14000,
         .pos_y = 0,
         .pos_z = 0});
    via_points->push_back(
        {.path_type = static_cast<char>(InterpolationType::AdvancedPoly),
         .dir_x = static_cast<bool>(MotorDirection::CounterClockwise),
         .dir_y = static_cast<bool>(MotorDirection::Clockwise),
         .dir_z = static_cast<bool>(MotorDirection::Clockwise),
         .time = 2 * 4000,
         .pos_x = 100,
         .pos_y = 0,
         .pos_z = 0});
    */

    atomic_bool stop = false;
    bool started = false;
        
    while(true){   
        busy_wait_ms(200);
        switch (stateManager->getMachineState())
        {
            case MachineState::ExecuteProgram:
                if(started && stop)
                {
                    stop = false;
                    multicore_reset_core1();
                    multicore_launch_core1(core1_resume);
                    multicore_fifo_push_blocking((uint32_t) &welding_system);
                }
                else if(!started && stop)
                {
                    stop = false;
                    started = true;
                    multicore_reset_core1();
                    multicore_launch_core1(core1_start);
                    multicore_fifo_push_blocking((uint32_t) &welding_system);
                    multicore_fifo_push_blocking((uint32_t) &via_points);
                    multicore_fifo_push_blocking((uint32_t) &stop);
                }
                break;
            case MachineState::ProgramStop:
                stop = true;
                break;
            case MachineState::Off:
                started = false;
                stop = true;
                break;
            case MachineState::EmergencyStop:
                started = false;
                stop = true;
                welding_system.disable_motors();
                break;
            default:
                break;
        }
    }
    return 0;
}

void core1_start()
{
    // CartesianRobotClient and its parameters are passed to us via the FIFO
    // We get the stateManager singleton
    // We execute the fiven routine
    // Once it stops, we check if it executed entire routine
    CartesianRobotClient * welding = (CartesianRobotClient *) multicore_fifo_pop_blocking();
    path_list_t * via_points = (path_list_t *) multicore_fifo_pop_blocking();
    atomic_bool * stop = (atomic_bool *) multicore_fifo_pop_blocking();
    StateManager * stateManager = StateManager::getInstance();
    welding->execute_routine(*via_points, [stop](){if(*stop) return true; return false;});
    if(welding->routine_finished)
    {
        stateManager->setAction(Action::Done);
        stateManager->machineProcess();
    }
}

void core1_resume()
{
    // CartesianRobotClient is passed to us via the FIFO
    // We get the stateManager singleton
    // We execute the fiven routine
    // Once it stops, we check if it executed entire routine
    CartesianRobotClient * welding = (CartesianRobotClient *) multicore_fifo_pop_blocking();
    StateManager * stateManager = StateManager::getInstance();
    welding->resume_execution();
    if(welding->routine_finished)
    {
        stateManager->setAction(Action::Done);
        stateManager->machineProcess();
    }
}
