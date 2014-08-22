/*
  HAPTIX User API, Part 2

  This file defines data structures and API functions
  available only in simulation.
*/

#pragma once

#include "haptix.h"


//-------------------------------- data structures --------------------------------------

// simulation info, beyond the information provided in hxRobotInfo
struct _hxSimInfo
{
    // array sizes
    int ndynamic;                           // number of dynamic objects
    int nstatic;                            // number of static objects

    // Anything else we should provide here?
};


// information about bodies, in the following order:
//   robot floating base
//   robot links, in the same order as the joints
//   dynamic objects
//   static objects
struct _hxBody
{
    float pos[hxMAXBODY][3];                // position
    float quat[hxMAXBODY][4];               // orientation (unit quaterion)
    float linvel[hxMAXBODY][3];             // linear velocity
    float angvel[hxMAXBODY][3];             // angular velocity (expmap representation)
    float linacc[hxMAXBODY][3];             // linear acceleration
    float angacc[hxMAXBODY][3];             // angular acceleration
};


// information about joints
struct _hxJoint
{
    float pos[hxMAXJOINT];                  // position
    float vel[hxMAXJOINT];                  // velocity
    float acc[hxMAXJOINT];                  // acceleration
    float torque_motor[hxMAXJOINT];         // torque due to actuation
    float torque_passive[hxMAXJOINT];       // torque due to limits, damping, friction
};


// information about contacts
struct _hxContact
{
    int   ncontact;                         // number of currently active contacts

    // contact descriptor
    int   body1[hxMAXCONTACT];              // contacting body 1
    int   body2[hxMAXCONTACT];              // contacting body 2

    // contact frame relative to global frame
    float point[hxMAXCONTACT][3];           // contact point
    float normal[hxMAXCONTACT][3];          // contact normal (unit vector)
    float tangent1[hxMAXCONTACT][3];        // first tangent (unit vector)
    float tangent2[hxMAXCONTACT][3];        // second tangent (unit vector)

    // data in contact frame, with axis order (normal, tangent1, tangent2)
    float distance[hxMAXCONTACT];           // normal distance
    float velocity[hxMAXCONTACT][3];        // relative velocity
    float force[hxMAXCONTACT][3];           // contact force
};


// type names
typedef struct _hxWorldInfo hxWorldInfo;
typedef struct _hxBody hxBody;
typedef struct _hxJoint hxJoint;
typedef struct _hxContact hxContact;



//-------------------------------- API functions ----------------------------------------

// get sim info
hxResult hx_sim_getinfo(int target, hxSimInfo* siminfo);


// get information about bodies
hxResult hx_sim_getbody(int target, hxBody* body);


// get information about joints
hxResult hx_sim_getjoint(int target, hxJoint* joint);


// get information about active contacts
hxResult hx_sim_getcontact(int target, hxContact* contact);


// get Jacobian of global point attached to robot link (index between 1 and njoint-1)
//  size of Jacobian matrix is 3-by-njoint, in row-major format
hxResult hx_sim_getjacobian(int target, int link, const float* point, float* jacobian);


// set simulation state (position and velocity) as follows:
//   the robot base and objects are set from hxBody
//   the robot links are set from hxJoint via forward kinematics
//   the robot link data in hxBody, and all acceleration and torque data are ignored
hxResult hx_sim_setstate(int target, const hxBody* body, const hxJoint* joint);


// synchronous update with direct torque control:
//   1. set joint torques ignoring actuator model
//   2. advance simulation state, sleep for remainder of update step only if requested
//   3. return simulated sensor data
hxResult hx_sim_updatedirect(int target, const float* torque, hxSensor* sensor, int flg_sleep);
