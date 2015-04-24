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
#define hxsMAXCONTACT 200

/// \brief Maximum number of links per model.
#define hxsMAXLINKS 100

/// \brief Maximum number of joints per model.
#define hxsMAXJOINTS 50

/// \brief Maximum number of models per simulation.
#define hxsMAXMODELS 50

/// \brief Maximum number of characters allowed per name.
#define hxsMAXNAMESIZE 100

/// \def hxCollisionMode
/// \sa hxs_set_model_collide_mode
/// \sa hxs_model_collide_mode
/// \brief The different collision modes for simulation objects.
/// NO_COLLIDE means the object will pass through other objects, and the
/// simulation does not know if this event occurs. hxs_contacts will not
/// generate contact points.
/// DETECTION_ONLY means that the object will pass through other objects, and
/// the simulation will detect when the object collides. hxs_contacts will
/// generate contact points when this happens, but the force and torque values
/// of the hxContactPoint struct will be invalid.
/// COLLIDE means that the object will obey the laws of physics and the
/// simulation will generate forces when it collides with other objects.
typedef enum {NO_COLLIDE, DETECTION_ONLY, COLLIDE} hxCollisionMode;

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

/// \brief A 4-tupe representing a color in RGBA space.
struct _hxColor
{
  float r;
  float g;
  float b;
  float alpha;
};

/// \def hxColor
/// \brief A 4-tupe representing a color in RGBA space.
/// r, g, b are numbers between 0 and 1 representing the red, green, and blue
/// levels, and alpha is a number between 0 and 1 representing the transparency
/// (0 is invisible, 1 is opaque).
typedef struct _hxColor hxColor;

/// \brief A quaternion.
struct _hxQuaternion
{
  float w;
  float x;
  float y;
  float z;
};

/// \def hxQuaternion
/// \brief A quaternion that is used to represent a rotation or orientation.
typedef struct _hxQuaternion hxQuaternion;

/// \brief A translation and orientation constructed of a hxVector3 and
/// hxQuaternion.
struct _hxTransform
{
  hxVector3 pos;
  hxQuaternion orient;
};

/// \def hxTransform
/// \brief A transformation that is combination of a position and orientation.
typedef struct _hxTransform hxTransform;

/// \def hxWrench
/// \brief A force-torque pair, usually applied at a joint.
struct _hxWrench
{
  /// \brief 3-dimensional force vector (Newtons).
  hxVector3 force;

  /// \brief 3-dimensional torque vector. The magnitude of the vector
  /// represents the magnitude of the torque (in Newton meters). The direction
  /// is normal to the plane of rotation and obeys the right-hand rule.
  hxVector3 torque;
};

/// \def hxWrench
/// \brief A force-torque pair.
typedef struct _hxWrench hxWrench;

/// \brief Information about a joint. A joint is a component of a model.
struct _hxJoint
{
  /// \brief Joint name.
  char name[hxsMAXNAMESIZE];

  /// \brief Position (radians).
  float pos;

  /// \brief Velocity (rad/s).
  float vel;

  /// \brief Acceleration (rad/s/s).
  float acc;

  /// \brief Torque due to actuation (N-m).
  float torque_motor;

  /// \brief Force/torque pair due to external disturbances.
  hxWrench wrench_reactive;
};

/// \def hxJoint
/// \brief Information about a joint.
typedef struct _hxJoint hxJoint;

/// \brief Information about a link. A link is a component of a model.
struct _hxLink
{
  /// \brief Link name.
  char name[hxsMAXNAMESIZE];

  /// \brief The position and orientation of the link, relative to the model.
  /// Position is in meters.
  hxTransform transform;

  /// \brief Linear velocity (m/s).
  hxVector3 lin_vel;

  /// \brief Angular velocity (rad/s).
  hxVector3 ang_vel;

  /// \brief Linear acceleration (m/s/s).
  hxVector3 lin_acc;

  /// \brief Angular acceleration (rad/s/s).
  hxVector3 ang_acc;
};

/// \def hxLink
/// \brief Information about a link.
typedef struct _hxLink hxLink;

/// \brief Information about a model.
struct _hxModel
{
  /// \brief Model name.
  char name[hxsMAXNAMESIZE];

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
  hxLink links[hxsMAXLINKS];

  /// \brief Number of joints in the model.
  /// This defines the range of elements in the "joints" array.
  int joint_count;

  /// \brief Array of joints in the model.
  /// \sa joint_count
  hxJoint joints[hxsMAXJOINTS];

  /// \brief 1 if the model is affected by gravity, 0 otherwise.
  int gravity_mode;
};

/// \def hxModel
/// \brief Information about a model.
typedef struct _hxModel hxModel;

/// \brief Information about a contact point.
struct _hxContactPoint
{
  /// \brief Name of the first contacting link.
  char link1[hxsMAXNAMESIZE];

  /// \brief Name of the second contacting link.
  char link2[hxsMAXNAMESIZE];

  /// \brief Description of contact frame relative to link 1 frame:
  /// origin of contact on link 1.
  hxVector3 point;

  /// \brief Description of contact frame relative to link 1 frame:
  /// normal direction (unit vector) of contact force on link 1.
  hxVector3 normal;

  /// \brief Normal distance (penetration depth) in link 1 frame (m).
  float distance;

  /// \brief Contact force/torque pair in link 1 frame at "point".
  hxWrench wrench;
};

/// \def hxContactPoint
/// \brief Information about a contact point.
typedef struct _hxContactPoint hxContactPoint;

/// \brief Information about contacts.
struct _hxContactPoints
{
  /// \brief Number of currently active contacts.
  int contact_count;

  /// \brief Description of contacts.
  hxContactPoint contacts[hxsMAXCONTACT];
};

/// \def hxContactPoints
/// \brief Information about contacts.
typedef struct _hxContactPoints hxContactPoints;

/// \brief Simulation information.
struct _hxSimInfo
{
  /// \brief Number of models in simulation.
  /// This defines the range of elements in the "models" array.
  int model_count;

  /// \brief Array of models in simulation.
  /// \sa modelCount
  hxModel models[hxsMAXMODELS];

  /// \brief Information about the camera.
  /// \sa hxs_get_camera_transform
  hxTransform camera_transform;
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
hxResult hxs_camera_transform(hxTransform *_transform);

/// \brief Set camera transform.
/// \param[in] _transform New camera transform.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_set_camera_transform(const hxTransform *_transform);

/// \brief Get information about active contacts for a model.
/// \param[in] _model The name of the model to query.
/// \param[out] _contact The latest contact information.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_contacts(const char *_model, hxContactPoints *_contact);

/// \brief Set simulation state (position and velocity) of joint named "_joint"
/// in model "_model" to the desired position and velocity. The acceleration,
/// torque, and reaction wrench of the joint may change based on the constraints
/// of model's dynamic system.
/// \param[in] _model Name of the model to set.
/// \param[in] _joint Name of the joint to set.
/// \param[in] _pos Desired position of the joint.
/// \param[in] _vel Desired velocity of the joint.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_set_model_joint_state(const char *_model, const char *_joint,
    float _pos, float _vel);

/// \brief Set simulation state (position and velocity) of link named "_link"
/// in model "_model" to the desired position and velocity. The link
/// acceleration may change based on the constraints of model's dynamic system.
/// \param[in] _model Name of the model to set.
/// \param[in] _link Name of the link to set.
/// \param[in] _transform Desired position and orientation of the link.
/// \param[in] _lin_vel Desired linear velocity of the link.
/// \param[in] _ang_vel Desired angular velocity of the link.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_set_model_link_state(const char *_model, const char *_link,
    const hxTransform *_transform, const hxVector3 *_lin_vel,
    const hxVector3 *_ang_vel);

/// \brief Add model during runtime.
/// \param[in] _sdf SDF xml description of the model.
/// \param[in] _name Model name.
/// \param[in] _x X position in global frame (m).
/// \param[in] _y Y position in global frame (m).
/// \param[in] _z Z position in global frame (m).
/// \param[in] _roll Roll in global frame (radians).
/// \param[in] _pitch Pitch in global frame (radians).
/// \param[in] _yaw Yaw in global frame (radians).
/// \param[in] _gravity_mode 1 if the model is affected by gravity, 0 otherwise.
/// \param[out] _model Pointer to the new model.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_add_model(const char *_sdf, const char *_name,
  float _x, float _y, float _z, float _roll, float _pitch, float _yaw,
  int _gravity_mode, hxModel *_model);

/// \brief Remove model.
/// \param[in] _name Name of the model.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_remove_model(const char *_name);

/// \brief Set model pose.
/// \param[in] _name Name of the model.
/// \param[in] _transform New model transform.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_model_transform(const char *_name, const hxTransform *_transform);

/// \brief Get whether or not this model is affected by gravity.
/// \param[in] _name Name of the model.
/// \param[in] _gravity If 1, the model is affected by gravity. If 0,
/// the model is free-floating.
hxResult hxs_model_gravity_mode(const char *_name, int *_gravity_mode);

/// \brief Set whether or not this model is affected by gravity.
/// \param[in] _name Name of the model.
/// \param[in] _gravity If 1, the model is affected by gravity. If 0,
/// the model is free-floating.
hxResult hxs_set_model_gravity_mode(const char *_name, const int _gravity_mode);

/// \brief Set the linear velocity of a model.
/// \param[in] _name Name of the model.
/// \param[in] _velocity Velocity (m/s).
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_linear_velocity(const char *_name, const hxVector3 *_velocity);

/// \brief Set the angular velocity of a model.
/// \param[in] _name Name of the model.
/// \param[in] _velocity Velocity (rad/s).
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_angular_velocity(const char *_name, const hxVector3 *_velocity);

/// \brief Apply force to a link.
/// \param[in] _modelName Name of the model containing the link.
/// \param[in] _linkName Name of the link.
/// \param[in] _force Force (N).
/// \param[in] _duration Duration of the force application in seconds. Set to 0
/// for persistent duration.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_force(const char *_modelName, const char *_linkName,
    const hxVector3 *_force, const float _duration);

/// \brief Apply torque to a link.
/// \param[in] _modelName Name of the model containing the link.
/// \param[in] _linkName Name of the link.
/// \param[in] _torque Torque (N-m).
/// \param[in] _duration Duration of the torque application in seconds. Set to 0
/// for persistent duration.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_torque(const char *_modelName, const char *_linkName,
    const hxVector3 *_torque, const float _duration);

/// \brief Apply a wrench to a link.
/// \param[in] _modelName Name of the model containing the link.
/// \param[in] _linkName Name of the link.
/// \param[in] _wrench Wrench to apply.
/// \param[in] _duration Duration of the torque application in seconds. Set to 0
/// for persistent duration.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_wrench(const char *_modelName, const char *_linkName,
    const hxWrench *_wrench, const float _duration);

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
/// \sa hxTime
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_timer(hxTime *_time);

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

/// \brief Set the color of the model.
/// \param[in] _model Name of the model.
/// \param[in] _color The color to set.
/// \sa hxColor
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_set_model_color(const char *_model, const hxColor *_color);

/// \brief Get the color of the model.
/// \param[in] _model Name of the model.
/// \param[out] _color The color of the model.
/// \sa hxColor
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_model_color(const char *_model, hxColor *_color);

/// \brief Set the collide mode of the object.
/// \param[in] _model Name of the model.
/// \param[in] _collide_mode The collide mode of the object.
/// \sa hxCollisionMode
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_set_model_collide_mode(const char *_model,
    const hxCollisionMode *_collide_mode);

/// \brief Get the collide mode of the object.
/// \param[in] _model Name of the model.
/// \param[out] _collide_mode The collide mode of the object.
/// \sa hxCollisionMode
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_model_collide_mode(const char *_model,
    hxCollisionMode *_collide_mode);

#ifdef __cplusplus
}
#endif

#endif
