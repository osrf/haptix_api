/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/// \file haptix_sim.h
/// \brief This file defines data structures and API functions
/// available only in simulation.

#ifndef __HAPTIX_SIM_API_HAPTIX_H__
#define __HAPTIX_SIM_API_HAPTIX_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "haptix.h"

// ---------- constants ----------

/// \brief Maximum number of contacts.
#define hxMAXCONTACT 200

/// \brief Maximum number of links per model.
#define hxMAXLINKS 100

/// \brief Maximum number of links per model.
#define hxMAXJOINTS 50

/// \brief Maximum number of models per simulation.
#define hxMAXMODELS 50

// ---------- data structures ----------

/// \brief A three-tuple vector.
struct _hxVector3
{
  float x;
  float y;
  float z;
};

/// \def hxVector3
/// \brief A three-tuple that is commonly used to represent a position or
/// translation.
typedef struct _hxVector3 hxVector3;

/// \brief A quaternion.
struct _hxQuaternion
{
  float w;
  float x;
  float y;
  float z;
};

/// \def hxQuaternion
/// \brief A quaternion that is used to represent a rotation or
/// orientation.
typedef struct _hxQuaternion hxQuaternion;

/// \brief A translation and orientation constructed of a hxVector3 and
/// hxQuaternion.
struct _hxTransform
{
  hxVector3 pos;
  hxQuaternion orient;
};

/// \def hxTransform
/// \brief A transformation that is combination of a position and
/// orientation.
typedef struct _hxTransform hxTransform;

/// \brief information about a joint. A joint is a component of a model.
struct _hxJoint
{
  /// \brief Joint name.
  char *name;

  /// \brief Position (radians).
  float pos;

  /// \brief Velocity (rad/s).
  float vel;

  /// \brief acceleration (rad/s/s).
  float acc;

  /// \brief Torque due to actuation (N-m).
  float torque_motor;

  /// \brief torque due to limits, damping, friction (N-m).
  float torque_passive;
};

/// \def hxJoint
/// \brief Information about a joint.
typedef struct _hxJoint hxJoint;

/// \brief Information about a link. A link is a component of a model.
struct _hxLink
{
  /// \brief Link name.
  char *name;

  /// \brief The position and orientation of the link, relative to the
  /// model. Position is in meters.
  hxTransform transform;

  /// \brief Linear velocity (m/s).
  hxVector3 linvel;

  /// \brief Angular velocity (rad/s).
  hxVector3 angvel;

  /// \brief Linear acceleration (m/s/s).
  hxVector3 linacc;

  /// \brief Angular acceleration (rad/s/s).
  hxVector3 angacc;
};
/// \def hxLink
/// \brief Information about a link.
typedef struct _hxLink hxLink;

/// \brief Information about a model.
struct _hxModel {
  /// \brief Model name.
  char *name;

  /// \brief The position and orientation of the model, relative to the
  /// global coordinate frame.
  hxTransform transform;

  /// \brief Unique id of the model. This number is generated by simulation.
  int id;

  /// \brief Number of links in the model.
  /// This defines the range of elements in the "links" array.
  int link_count;

  /// \brief Array of links in the model.
  /// \sa link_count
  hxLink links[hxMAXLINKS];

  /// \brief Number of joints in the model.
  /// This defines the range of elements in the "joints" array.
  int joint_count;

  /// \brief Array of joints in the model.
  /// \sa joint_count
  hxJoint joints[hxMAXJOINTS];
};

/// \def hxModel
/// \brief Information about simulated models.
typedef struct _hxModel hxModel;

/// \brief Information about a contact point.
struct _hxContactPoint
{
  /// \brief contact descriptor for contacting link 1.
  int link1;

  /// \brief contact descriptor for contacting link 2.
  int link2;

  /// \brief Description of contact frame relative to global frame:
  /// origin of frame.
  hxVector3 point;

  /// \brief Description of contact frame relative to global frame:
  /// normal direction (unit vector).
  hxVector3 normal;

  /// \brief Description of contact frame relative to global frame:
  /// first tangent direction (unit vector).
  hxVector3 tangent1;

  /// \brief Description of contact frame relative to global frame:
  /// second tangent direction (unit vector).
  hxVector3 tangent2;

  /// \brief Normal distance (penetration depth) in contact frame (m).
  float distance;

  /// \brief Relative velocity in contact frame (m/s),
  /// with axis order (normal, tangent1, tangent2).
  hxVector3 velocity;

  /// \brief Contact force in contact frame (N),
  /// with axis order (normal, tangent1, tangent2).
  hxVector3 force;
};

/// \def hxContact
/// \brief Information about a contact point.
typedef struct _hxContactPoint hxContactPoint;

/// \brief Information about contacts.
struct _hxContactPoints
{
  /// \brief Number of currently active contacts.
  int contactCount;

  /// \brief Description of contacts.
  hxContactPoint contacts[hxMAXCONTACT];
};

/// \def hxContact
/// \brief Information about contacts.
typedef struct _hxContactPoints hxContactPoints;

/// \brief Simulation information.
struct _hxSimInfo
{
  /// \brief Number of models in simulation.
  /// This defines the range of elements in the "models" array.
  int modelCount;

  /// \brief Array of models in simulation.
  /// \sa modelCount
  hxModel models[hxMAXMODELS];

  /// \brief Information about the camera.
  /// \sahxs_get_camera_transform 
  hxTransform cameraTransform;
};

/// \def hxSimInfo
/// \brief Information about the simulation world.
typedef struct _hxSimInfo hxSimInfo;

// ---------- API functions ----------

/// \brief Get simulation information.
/// \param[out] _siminfo Simulation information requested.
/// \sa _hxSimInfo
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_siminfo(hxSimInfo *_siminfo);

/// \brief Get information about the simulation camera.
/// \param[out] _camera Information about the simulation camera.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_get_camera_transform(hxTransform *_camera_transform);

/// \brief Set camera transform.
/// \param[in] _transform New camera transform.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_camera_transform(const hxTransform *_transform);

/// \brief Get information about active contacts.
/// \param[out] _contact The latest contact information.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_contacts(hxContactPoints *_contact);

/// \brief Set simulation state (position and velocity) as follows:
///   the robot base and objects are set from hxModel
///   the robot links are set from hxJoint via forward kinematics
///   the robot link data in hxModel, and all acceleration and torque
///   data are ignored.
/// \param[in] _model Model information to set.
/// \param[in] _joint Joint information to set.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_state(const char *_modelName, const hxJoint *_joint);

/// \brief Add model during runtime.
/// \param[in] _urdf URDF xml description of the model.
/// \param[in] _name Model name.
/// \param[in] _x X position in global frame (m).
/// \param[in] _y Y position in global frame (m).
/// \param[in] _z Z position in global frame (m).
/// \param[in] _roll Roll in global frame (radians).
/// \param[in] _pitch Pitch in global frame (radians).
/// \param[in] _yaw Yaw in global frame (radians).
/// \param[out] _model Pointer to the new model.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_add_model(const char *_urdf, const char *_name,
  float _x, float _y, float _z, float _roll, float _pitch, float _yaw,
  hxModel *_model);

/// \brief Remove model.
/// \param[in] _id Id of the model.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_remove_model_id(int _id);

/// \brief Set model pose.
/// \param[in] _id Id of the model.
/// \param[in] _transform New model transform.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_model_transform(int _id, const hxTransform *_transform);

/// \brief Set the linear velocity of a model.
/// \param[in] _id Id of the model.
/// \param[in] _velocity Velocity (m/s).
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_linear_velocity(int _id, const hxVector3 *_velocity);

/// \brief Set the angular velocity of a model.
/// \param[in] _id Id of the model.
/// \param[in] _velocity Velocity (rad/s).
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_angular_velocity(int _id, const hxVector3 *_velocity);

/// \brief Apply force to a link.
/// \param[in] _modelName Name of the model containing the link.
/// \param[in] _linkName Name of the link.
/// \param[in] _force Force (N).
/// \param[in] _double Duration of the force application in seconds. Set to 0
/// for persistent duration.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_force(const char *_modelName, const char *_linkName,
    const hxVector3 *_force, const double _duration);

/// \brief Apply torque to a link.
/// \param[in] _modelName Name of the model containing the link.
/// \param[in] _linkName Name of the link.
/// \param[in] _torque Torque (N-m).
/// \param[in] _double Duration of the torque application in seconds. Set to 0
/// for persistent duration.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_torque(const char *_modelName, const char *_linkName,
    const hxVector3 *_torque, const double _duration);

/// \brief Send world reset command/Carry over limb pose between world reset.
/// \param[in] _resetLimbPose Non-zero to reset the pose of the limb.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_reset(int _resetLimbPose);

/// \brief Reset on-screen timer.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_reset_timer();

/// \brief Start on-screen timer.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_start_timer();

/// \brief Stop on-screen timer.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_stop_timer();

/// \brief Get the state of the on-screen timer.
/// \param[out] _time The time represented by the on-screen timer.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_get_timer(hxTime *_time);

/// \brief Start recording log file. Only one log file may be recorded at
/// a time.
/// \param[in] _filename Name of the file to log information into.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_start_logging(const char *_filename);

/// \brief Determine if logging is running.
/// \param[out] _result 1 if logging is running.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_is_logging(int *_result);

/// \brief Stop recording log file.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_stop_logging();

#ifdef __cplusplus
}
#endif

#endif
