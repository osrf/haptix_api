/*
  HAPTIX User API, Part 1

  This file defines data structures and API functions
  available both in simulation and on physical robots.
*/

#pragma once
/// \file haptix.h
/// \brief Structures and functions for the primary HAPTIX API.

#ifndef __HAPTIX_COMM_HAPTIX_H__
#define __HAPTIX_COMM_HAPTIX_H__

#ifdef __cplusplus
extern "C" {
#endif

// ---------- constants ----------

/// \brief Maximum number of motors.
/// Defines the maximum number of motors across any particular robot.
/// It is used when allocating sensor and command objects related to the motors.
/// The number of motors for a particular robot is defined in #_hxRobotInfo.
#define hxMAXMOTOR              32

/// \brief Maximum number of joints.
/// Defines the maximum number of joints across any particular robot.
/// It is used when allocating sensor properties related to the joints.
/// number of joints for a particular robot is defined in #_hxRobotInfo.
#define hxMAXJOINT              32

/// \brief Maximum number of contact sensors.
/// Defines the maximum number of contact sensors across any particular
///r obot.
/// It is used when allocating an hxSensor.contact object.
/// The number of contact sensors for a particular robot is defined in
/// #_hxRobotInfo.
#define hxMAXCONTACTSENSOR      32

/// \brief Maximum number of IMUs.
/// Defines the maximum number of inertial measurement units across any
/// particular robot.
/// It is used when allocating sensor objects related to IMUs.
/// The number of IMUs for a particular robot is defined in #_hxRobotInfo.
#define hxMAXIMU                32

/// \brief API return codes.
/// These return codes are used to specify the outcome of a haptix-comm
/// function (e.g. success or failure).
typedef enum
{
  /// success
  hxOK = 0,
  /// failure
  hxERROR
} hxResult;

/// \brief Communication targets.
/// This enumeration specifies the possible robots (physical or simulated)
/// that communicate over the HAPTIX API. Specifying the target may be important
/// in case a controller has different behavior across different robots.
typedef enum
{
  /// \brief DEKA physical arm
  hxDEKA = 0,
  /// \brief JHU APL MPL physical arm
  hxMPL,
  /// \brief Gazebo simulator
  hxGAZEBO,
  /// \brief MuJoCo simulator
  hxMUJOCO
} hxTarget;

// ---------- data structures ----------

/// \brief A representation of time
struct _hxTime
{
  /// Seconds
  int sec;

  /// Nanoseconds
  int nsec;
};

/// \brief Device information.
/// This data structure specifies inherent properties of the robot that
/// do not change during simulation (for
/// example, the number of joints in the robot arm).
///
/// It can be retrieved from a communication target by calling
/// hx_robotinfo(int, hxRobotInfo*).
struct _hxRobotInfo
{
  /// \brief Number of motors.
  /// Motors are commanded through filling an hxCommand struct and calling
  /// hx_update(int, const hxCommand*, hxSensor*, hxTime*).
  ///
  /// The number of motors is less than or equal to the number of
  /// joints. For example, one motor may control several joints through
  /// kinematic joint coupling.
  int nmotor;

  /// \brief Number of hinge joints.
  /// The joints are passive and are moved as a side effect of commanding
  /// the motors. A joint may correspond directly to the movement of a motor,
  /// or it may be commanded indirectly through joint coupling.
  ///
  /// The number of joints is greater than or equal to the number of
  /// motors.
  int njoint;

  /// \brief Number of contact sensors.
  /// A contact sensor measures the magnitude of the force on that sensor.
  int ncontactsensor;

  /// \brief Number of IMUs (inertial measurement units).
  /// An IMU or inertial measurement unit measures the 3-dimensional linear
  /// acceleration vector and the 3-dimensional angular velocity vector
  /// experienced by the sensor.
  int nIMU;

  /// \brief Minimum and maximum motor angles (rad).
  /// An m by 2 array representing the angular limits of each motor in the
  /// robot, where m is the maximum number of motors. Each 1x2 row of the array
  /// corresponds to a motor. The first entry in the row is the lower limit
  /// of the motor. The second entry is the upper limit of the motor.
  float motorlimits[hxMAXMOTOR][2];

  /// \brief Minimum and maximum joint angles (rad).
  /// An n by 2 array representing the angular limits of each joint in the
  /// robot, where n is the maximum number of joints. Each 1x2 row of the array
  /// corresponds to a joint. The first entry in the row is the lower limit
  /// of the joint. The second entry is the upper limit of the joint.
  float jointlimits[hxMAXJOINT][2];

  /// \brief Hz rate at which the robot will update.
  float updateRate;
};

/// \brief Sensor data.
/// This data structure specifies the sensor information gained in a simulation
/// update.
/// It is an output of the function
/// hx_update(int, const hxCommand*, hxSensor*, hxTime*).
struct _hxSensor
{
  /// \brief Timestamp.
  /// Time at which the sensor reading was taken. See #hxTime for data format.
  hxTime timestamp;

  /// \brief Motor position (rad).
  /// An array of floats of size #hxMAXMOTOR. Entries 0 through
  /// hxRobotInfo::nmotors-1 contain the angular positions for each motor.
  /// The ordering of these motor values is consistent across the different
  /// motor-related properties of #hxSensor.
  ///
  /// These values cannot exceed the minimum and maximum values specified in
  /// hxRobotInfo::limit.
  float motor_pos[hxMAXMOTOR];

  /// \brief Motor velocity (rad/s).
  /// An array of floats of size #hxMAXMOTOR. Entries 0 through
  /// hxRobotInfo::nmotors-1 contain the angular velocity for
  /// each motor. The ordering of
  /// these motor values is consistent across the different motor-related
  /// properties of hxSensor.
  float motor_vel[hxMAXMOTOR];

  /// \brief Torque applied by embedded controller (Nm).
  /// An array of floats of size #hxMAXMOTOR. Entries 0 through
  /// hxRobotInfo::nmotors-1 contain the torque for each motor.
  /// The ordering of
  /// these motor values is consistent across the different motor-related
  /// properties of #hxSensor.
  float motor_torque[hxMAXMOTOR];

  /// \brief Joint position (rad).
  /// An array of floats of size #hxMAXJOINT. Entries 0 through
  /// hxRobotInfo::njoint-1 contain the angular position for each joint.
  /// The ordering of these joint values is consistent with
  /// hxSensor::joint_vel.
  float joint_pos[hxMAXJOINT];

  /// \brief Joint velocity (rad/s).
  /// An array of floats of size #hxMAXJOINT.
  /// Entries 0 through hxRobotInfo::njoint-1 contain the angular position
  /// for each joint.
  /// The ordering of these joint values is consistent with
  /// hxSensor::joint_pos.
  float joint_vel[hxMAXJOINT];

  /// \brief Contact normal force (N).
  /// An array of floats of size #hxMAXCONTACTSENSOR. Entries 0 through
  /// hxRobotInfo::ncontactsensor contain the contact magnitude for each
  /// contact sensor.
  float contact[hxMAXCONTACTSENSOR];

  /// \brief 3D linear acceleration (m/s^2).
  /// An array of floats of size #hxMAXIMUx3 where each row is a 3-dimensional
  /// linear acceleration vector. The entries of each row are measured in
  /// meters per second squared and ordered (x, y, z).
  /// Entries 0 through hxRobotInfo::nimu-1 contain the acceleration vectors
  /// for each IMU.\n
  /// The ordering of these IMU values is consistent with hxSensor::IMU_angvel.
  float IMU_linacc[hxMAXIMU][3];

  /// \brief 3D angular velocity (rad/s).
  /// An array of floats of size #hxMAXIMUx3 where each row is a 3-dimensional
  /// angular velocity vector. The entries of each row are measured in
  /// radians per second and ordered (x, y, z).
  /// Entries 0 through hxRobotInfo::nimu-1 contain the velocity vectors
  /// for each IMU.
  ///
  /// The ordering of these IMU values is consistent with hxSensor::IMU_linacc.
  float IMU_angvel[hxMAXIMU][3];

   /// \brief 3D orientation quaternion
   float IMU_orientation[hxMAXIMU][4];
};

/// \brief Motor command data.
/// This data structure specifies the next request to be send to the simulated
/// limb model.
///
/// It is an input of the function
/// hx_update(int, const hxCommand*, hxSensor*, hxTime*).
struct _hxCommand
{
  /// \brief Timestamp.
  /// The time at which the command was sent. See #hxTime for data format.
  hxTime timestamp;

  /// \brief Target reference positions (rad).
  /// An array of floats of size #hxMAXMOTOR. Entries 0 through
  /// hxRobotInfo::nmotors-1 contain the desired angular positions for each
  /// motor.
  float ref_pos[hxMAXMOTOR];

  /// \brief Target reference velocities (rad/s).
  /// An array of floats of size #hxMAXMOTOR. Entries 0 through
  /// hxRobotInfo::nmotors-1 contain the desired angular velocities for each
  /// motor.
  float ref_vel[hxMAXMOTOR];

  /// \brief Target position feedback gains.
  /// An array of floats of size #hxMAXMOTOR. Entries 0 through
  /// hxRobotInfo::nmotors-1 contain the position gain that will be
  /// applied during the update phase of the model controller.
  float gain_pos[hxMAXMOTOR];

  /// \brief Target velocity feedback gains.
  /// An array of floats of size #hxMAXMOTOR. Entries 0 through
  /// hxRobotInfo::nmotors-1 contain the velocity gain that will be
  /// applied during the update phase of the model controller.
  float gain_vel[hxMAXMOTOR];
};

/// \def hxTime
/// \brief Time representation.
typedef struct _hxTime hxTime;

/// \def hxRobotInfo
/// \brief Robot information.
typedef struct _hxRobotInfo hxRobotInfo;

/// \def hxSensor
/// \brief Sensor data.
typedef struct _hxSensor hxSensor;

/// \def hxCommand
/// \brief Motor commands.
typedef struct _hxCommand hxCommand;

// ---------- API functions ----------

/// \brief Connect to specified robot/simulator target.
/// Multiple calls to this function are allowed with different targets.
///
/// This function is not needed for use with Gazebo but added for code
/// compatibility with other simulators.
/// \param[in] _target Device to be connected. The valid targets are defined in
/// #hxTarget.
/// \param[in] _host When connecting to a simulator, use _host to specify
/// the machine that is running the simulator. The default value is blank,
/// which indicates that automatic detection of the robot/simulator
/// should be attempted.
/// \param[in] _port When connecting to a simulator, use _port to specify
/// what port the simulator is running on. This may be used in conjunction
/// with the _host paramter. Leave undefined of zero for automatic
/// detection.
/// \return 'hxOK' if the connection succeed or an error code otherwise.
hxResult hx_connect(int _target, const char *_host = "", int _port = 0);

/// \brief Close connection to specified robot/simulator target.
///
/// This function is not needed for use with Gazebo but added for code
/// compatibility with other simulators.
/// \param[in] _target Device to be disconnected. The valid targets are defined
/// in #hxTarget.
/// \return 'hxOK' if the disconnection succeed or an error code otherwise.
hxResult hx_close(int _target);

/// \brief Get information for a specified robot/simulator target.
/// \param[in] _target Requested robot. The valid targets are defined in
/// #hxTarget.
/// \param[out] _robotInfo Device information requested. See #_hxRobotInfo
/// for a list of available fields.
/// \return 'hxOK' if the operation succeed or an error code otherwise.
hxResult hx_robotinfo(int _target, hxRobotInfo *_robotinfo);

/// \brief Asynchronous command update at the rate supported by the robot:
///   1. Set the new motor command.
///   2. Return simulated or physical sensor data.
/// \param[in] _target Device to update. The valid targets are defined in
/// #hxTarget.
/// \param[in] _command New command to be sent. See #_hxCommand for the full
/// description of fields contained in a command request.
/// \param[out] _sensor Sensor data received after the update. See #_hxSensor
/// for the full description of fields contained the state response.
/// \param[out] _timestamp The timestamp associated with the sensor data.
/// \return 'hxOK' if the operation succeed or an error code otherwise.
hxResult hx_update(int _target,
                   const hxCommand *_command,
                   hxSensor *_sensor,
                   hxTime *_timestamp);

/// \brief Synchronous read-only update supported by the robot.
/// Advances simulation state and sleep for remainder of update step,
/// or wait for physical robot to finish update.
/// Return sensor data.
/// \param[in] _target Device to update.
/// \param[out] _sensor Sensor data received after the update.
/// \param[out] _timestamp The timestamp associated with the sensor data.
/// \return 'hxOK' if the operation succeed or an error code otherwise.
hxResult hx_readsensors(int _target, hxSensor *_sensor, hxTime *_timestamp);

/// \brief Return a string that describes the last result.
/// \sa hxResult.
/// \param[in] _target Target robot.
/// \return String that describes the last result.
const char *hx_lastresult(hxTarget _target);

#ifdef __cplusplus
}
#endif
#endif
