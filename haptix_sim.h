/*
   HAPTIX User API, Part 2

   This file defines data structures and API functions
   available only in simulation.
*/

#pragma once

#include "haptix.h"

// ---------- data structures ----------

/// \brief Simulation information
struct _hxSimInfo
{
  /// \brief Number of models in simulation.
  /// This defines the range of elements in the "models" array.
  int modelCount;

  /// \brief Array of models in simulation.
  /// \sa modelCount
  hxModel **models;
};

/// \brief Information about a model
struct _hxModel
{
  /// \brief True if the model is static
  bool isStatic;

  /// \brief Number of links in the model.
  /// This defines the range of elements in the "links" array.
  int linkCount;

  /// \brief Array of links in the model.
  /// \sa linkCount
  hxLink **links;
}

struct _hxLink
{
  /// \brief Position of the link
  float pos[3];

  /// \brief orientation (unit quaterion)
  float quat[4];

  /// \brief linear velocity
  float linvel[hxMAXBODY][3];

  /// \brief angular velocity (expmap representation)
  float angvel[hxMAXBODY][3];

  /// \brief linear acceleration
  float linacc[hxMAXBODY][3];

  /// \brief angular acceleration
  float angacc[hxMAXBODY][3];
};

/// \brief information about joints
struct _hxJoint
{
  /// \brief position
  float pos[hxMAXJOINT];

  /// \brief  velocity
  float vel[hxMAXJOINT];

  /// \brief acceleration
  float acc[hxMAXJOINT];

  /// \brief torque due to actuation
  float torque_motor[hxMAXJOINT];

  /// \brief  torque due to limits, damping, friction
  float torque_passive[hxMAXJOINT];
};

/// \brief information about contacts
struct _hxContact
{
  /// \brief number of currently active contacts
  int ncontact;

  /// \brief contact descriptor for contacting body 1
  int body1[hxMAXCONTACT];

  /// \brief contact descriptor for contacting body 2
  int body2[hxMAXCONTACT];

  /// \brief contact frame relative to global frame for the contact point
  float point[hxMAXCONTACT][3];

  /// \brief contact frame relative to global frame for the contact normal
  /// (unit vector)
  float normal[hxMAXCONTACT][3];

  /// \brief contact frame relative to global frame for the first tangent
  /// (unit vector)
  float tangent1[hxMAXCONTACT][3];

  /// \brief contact frame relative to global frame for the second tangent
  /// (unit vector)
  float tangent2[hxMAXCONTACT][3];

  /// \brief Normal distance in contact frame,
  /// with axis order (normal, tangent1, tangent2)
  float distance[hxMAXCONTACT];

  /// \brief Relative velocity in contact frame,
  /// with axis order (normal, tangent1, tangent2)
  float velocity[hxMAXCONTACT][3];

  /// \brief Contact force in contact frame,
  /// with axis order (normal, tangent1, tangent2)
  float force[hxMAXCONTACT][3];
};

/// \def hxSimInfo
/// \brief Information about the simulation world.
typedef struct _hxSimInfo hxSimInfo;

/// \def hxBody
/// \brief Information about simulated bodies.
typedef struct _hxBody hxBody;

/// \def hxJoint
/// \brief Information about simulated joints.
typedef struct _hxJoint hxJoint;

/// \def hxContact
/// \brief Information about contacts.
typedef struct _hxContact hxContact;


// ---------- API functions ----------

/// \brief Get simulation information.
/// \param[in] _target Device to be connected.
/// The valid targets are defined in #hxTarget.
/// \param[out] _siminfo Simulation information requested.
/// \sa _hxSimInfo
/// \return 'hxOK' if the connection succeed or an error code otherwise.
hxResult hxs_siminfo(int _target, hxSimInfo *_siminfo);

/// \brief Get information about bodies
/// \param[in] _target Device to be connected.
/// The valid targets are defined in #hxTarget.
/// \param[in] _index Index of the body.
/// \param[out] _body Body information requested.
/// \return 'hxOK' if the connection succeed or an error code otherwise.
hxResult hxs_body(int _target, int _index, hxBody *_body);

/// \brief Get information about joints
/// \param[in] _target Device to be connected. The valid targets are defined in
/// #hxTarget.
/// \return 'hxOK' if the connection succeed or an error code otherwise.
hxResult hxs_getjoint(int _target, hxJoint *_joint);

/// \brief Get information about active contacts
/// \param[in] _target Device to be connected. The valid targets are defined in
/// #hxTarget.
/// \return 'hxOK' if the connection succeed or an error code otherwise.
hxResult hxs_getcontact(int _target, hxContact *_contact);

/// \brief Get Jacobian of global point attached to robot link
/// (index between 1 and njoint-1) size of Jacobian matrix is
/// 3-by-njoint, in row-major format.
/// \param[in] _target Device to be connected. The valid targets are defined in
/// #hxTarget.
/// \return 'hxOK' if the connection succeed or an error code otherwise.
hxResult hxs_getjacobian(int _target, int _link,
    const float *_point, float *_jacobian);

/// \brief set simulation state (position and velocity) as follows:
///   the robot base and objects are set from hxBody
///   the robot links are set from hxJoint via forward kinematics
///   the robot link data in hxBody, and all acceleration and torque
///   data are ignored.
/// \param[in] _target Device to be connected. The valid targets are defined in
/// #hxTarget.
/// \return 'hxOK' if the connection succeed or an error code otherwise.
hxResult hxs_setstate(int _target, const hxBody *_body, const hxJoint *_joint);

/// synchronous update with direct torque control:
///   1. set joint torques ignoring actuator model
///   2. advance simulation state, sleep for remainder of update step
///      only if requested
///   3. return simulated sensor data
/// \param[in] _target Device to be connected. The valid targets are defined in
/// #hxTarget.
/// \return 'hxOK' if the connection succeed or an error code otherwise.
hxResult hxs_updatedirect(int _target, const float *_torque,
    hxSensor *_sensor, int _flagSleep);
