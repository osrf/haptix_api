/*
  HAPTIX User API, Part 1

  This file defines data structures and API functions that are
  supported both in simulation and on physical devices.

  See haptix_extra.h for simulation-only functionality.
*/


//-------------------------------- constants --------------------------------------------

// maximum array sizes for static allocation
#define hxMAXMOTOR		15
#define hxMAXJOINT		25
#define hxMAXCONTACT	10
#define hxMAXIMU		10


// API return codes
enum hxResult
{
	hxOK = 0,								// success
	hxBAD,									// a bad thing
	hxHORRIBLE								// another bad thing
};


// communication targets
enum hxTarget
{
	hxDEKA = 0,								// DEKA physical arm
	hxMPL,									// MPL physical arm
	hxGAZEBO,								// Gazebo simulator
	hxMUJOCO								// MuJoCo simulator
};



//-------------------------------- data structures --------------------------------------

// robot info
struct _hxInfo
{
	// array sizes
	int nmotor;								// number of motors
	int njoint;								// number of hinge joints
	int ncontact;							// number of contact sensors
	int nIMU;								// number of IMUs

	// Anything else we should provide here, as opposed to letting the user extract the
	//  data they need from the XML model or robot specs? The more info we include here,
	//  the closer we get to replicating the entire XML model file.
};


// sensor data
struct _hxSensor
{
	// motor data
	float motor_pos[hxMAXMOTOR];			// motor position (rad)
	float motor_vel[hxMAXMOTOR];			// motor velocity (rad/s)
	float motor_torque[hxMAXMOTOR];			// torque applied by embedded controller (Nm)

	// joint data
	float joint_pos[hxMAXJOINT];			// joint position (rad)
	float joint_vel[hxMAXJOINT];			// joint velocity (rad/s)

	// contact data
	float contact[hxMAXCONTACT];			// contact normal force (N)

	// IMU data
	float IMU_linacc[hxMAXIMU][3];			// 3D linear acceleration (m/s^2)
	float IMU_angvel[hxMAXIMU][3];			// 3D angular velocity (rad/s)
};


// motor commands
struct _hxCommand
{
	// PD controller data
	float ref_pos[hxMAXMOTOR];				// reference positions
	float ref_vel[hxMAXMOTOR];				// reference velocities
	float gain_pos[hxMAXMOTOR];				// position feedback gains
	float gain_vel[hxMAXMOTOR];				// velocity feedback gains

	// Do the robots accept any other commands?
};


// civilized type names
typedef struct _hxInfo hxInfo;
typedef struct _hxSensor hxSensor;
typedef struct _hxCommand hxCommand;



//-------------------------------- API functions ----------------------------------------

// connect to specified device/simulator target;
// multile calls to this function are allowed with different targets
hxResult hxConnect(int target);


// close connection to specified device/simulator target
hxResult hxClose(int target);


// get info for specified device/simulator target
hxResult hxGetInfo(int target, hxInfo* info);


// set motor commands and receive sensor data from specified device/simulator target
hxResult hxUpdate(int target, const hxCommand* command, hxSensor* sensor);
