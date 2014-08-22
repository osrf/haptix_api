/*
  HAPTIX User API, Part 1

  This file defines data structures and API functions
  available both in simulation and on physical devices.
*/

#pragma once


//-------------------------------- constants --------------------------------------------

// maximum array sizes for static allocation
#define hxMAXMOTOR              15
#define hxMAXJOINT              25
#define hxMAXCONTACTSENSOR      10
#define hxMAXIMU                10
#define hxMAXCONTACT            30
#define hxMAXBODY               30


// API return codes
enum hxResult
{
    hxOK = 0,                               // success
    hxBAD,                                  // a bad thing
    hxHORRIBLE                              // another bad thing
};


// communication targets
enum hxTarget
{
    hxDEKA = 0,                             // DEKA physical arm
    hxMPL,                                  // MPL physical arm
    hxGAZEBO,                               // Gazebo simulator
    hxMUJOCO                                // MuJoCo simulator
};



//-------------------------------- data structures --------------------------------------

// robot info
struct _hxRobotInfo
{
    // array sizes
    int nmotor;                             // number of motors
    int njoint;                             // number of hinge joints
    int ncontactsensor;                     // number of contact sensors
    int nIMU;                               // number of IMUs

    // joint limits
    float limit[hxMAXJOINT][2];             // minimum and maximum joint angles (rad)

    // Anything else we should provide here, as opposed to letting the user extract
    // the data they need from the XML model or robot specs?
};


// sensor data
struct _hxSensor
{
    // motor data
    float motor_pos[hxMAXMOTOR];            // motor position (rad)
    float motor_vel[hxMAXMOTOR];            // motor velocity (rad/s)
    float motor_torque[hxMAXMOTOR];         // torque applied by embedded controller (Nm)

    // joint data
    // Question: how is this expected to be used?
    // Question: having both motor + joint could potentially violate transmission?
    float joint_pos[hxMAXJOINT];            // joint position (rad)
    float joint_vel[hxMAXJOINT];            // joint velocity (rad/s)

    // contact data
    float contact[hxMAXCONTACTSENSOR];      // contact normal force (N)

    // IMU data
    float IMU_linacc[hxMAXIMU][3];          // 3D linear acceleration (m/s^2)
    float IMU_angvel[hxMAXIMU][3];          // 3D angular velocity (rad/s)
    float IMU_orientation[hxMAXIMU][4];     // 3D orientation quaternion
};


// motor commands
struct _hxCommand
{
    // PD controller data
    float ref_pos[hxMAXMOTOR];              // reference positions
    float ref_vel[hxMAXMOTOR];              // reference velocities

    float gain_pos_p[hxMAXMOTOR];           // position feedback p-gains
    float gain_pos_i[hxMAXMOTOR];           // position feedback i-gains
    float gain_pos_d[hxMAXMOTOR];           // position feedback d-gains
    float gain_pos_imax[hxMAXMOTOR];        // position feedback i-max
    float gain_pos_imin[hxMAXMOTOR];        // position feedback i-min
    float gain_vel_p[hxMAXMOTOR];           // velocity feedback p-gains

    // Do the robots accept any other commands?
};


// type names
typedef struct _hxInfo hxInfo;
typedef struct _hxSensor hxSensor;
typedef struct _hxCommand hxCommand;



//-------------------------------- API functions ----------------------------------------

// connect to specified device/simulator target;
//  multile calls to this function are allowed with different targets
hxResult hx_connect(int target);


// close connection to specified device/simulator target
hxResult hx_close(int target);


// get info for specified device/simulator target
hxResult hx_getrobotinfo(int target, hxRobotInfo* robotinfo);


// synchronous update at the rate supported by the device:
//   1. set motor command
//   2. advance simulation state and sleep for remainder of update step,
//      or wait for physical device to finish update
//   3. return simulated or physical sensor data
hxResult hx_update(int target, const hxCommand* command, hxSensor* sensor);
