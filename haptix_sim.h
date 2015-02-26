/*
   HAPTIX User API, Part 2

   This file defines data structures and API functions
   available only in simulation.
*/

#pragma once

#include "haptix.h"

// ---------- data structures ----------

/// \brief A three-tuple vector
struct _hxVector3
{
  float x;
  float y;
  float z;
};

/// \def hxVector3
/// \brief A three-tuple that is commonly used to represent a position or
/// translation.
typedef _hxVector3 hxVector3;

/// \brief A quaternion
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
typedef _hxQuaternion hxQuaternion;

/// \brief A translation and orientation constructed of a hxVector3 and
/// hxQuaternion.
struct _hxTransform
{
  hxVector3 pos;
  hxQuaternion orient;
};

/// \def hxTransform
/// \brief A transormation that is combination of a position and
/// orientation.
typedef _hxTransform hxTransform;

/// \brief Information about a model
struct _hxModel
{
  /// \brief The position and orientation of the model, relative the
  /// global coordiate frame.
  hxTransform transform;

  /// \brief True if the model is static (immovable).
  bool isStatic;

  /// \brief Unique id of the model. This number is generated by simulation.
  int id;

  /// \brief Number of links in the model.
  /// This defines the range of elements in the "links" array.
  int linkCount;

  /// \brief Array of links in the model.
  /// \sa linkCount
  hxLink **links;

  /// \brief Number of joints in the model.
  /// This defines the range of elements in the "joints" array.
  int linkCount;

  /// \brief Array of joints in the model.
  /// \sa jointCount
  hxJoint **joints;
};

/// \brief Information about a link. A link is a component of a model
struct _hxLink
{
  /// \brief The position and orientation of the link, relative the
  /// model. Position is in meters.
  hxTransform transform;

  /// \brief Linear velocity (m/s)
  hxVector3 linvel;

  /// \brief Angular velocity (rad/s)
  hxVector3 angvel;

  /// \brief Linear acceleration (m/s/s)
  hxVector3 linacc;

  /// \brief Angular acceleration (rad/s/s)
  hxVector3 angacc;
};

/// \brief information about a joint. A joint is a component of a model
struct _hxJoint
{
  /// \brief Position (radians)
  float pos;

  /// \brief Velocity (rad/s)
  float vel;

  /// \brief acceleration (rad/s/s)
  float acc;

  /// \brief Torque due to actuation (N-m)
  float torque_motor;

  /// \brief torque due to limits, damping, friction (N-m)
  float torque_passive;
};

/// \brief Information about contacts
struct _hxContact
{
  /// \brief Number of currently active contacts
  int contactCount;

  /// \brief contact descriptor for contacting body 1
  int body1[hxMAXCONTACT];

  /// \brief contact descriptor for contacting body 2
  int body2[hxMAXCONTACT];

  /// \brief Description of contact frame relative to global frame:
  /// origin of frame
  hxVector3 point[hxMAXCONTACT];

  /// \brief Description of contact frame relative to global frame:
  /// normal direction (unit vector)
  hxVector3 normal[hxMAXCONTACT];

  /// \brief Description of contact frame relative to global frame:
  /// first tangent direction (unit vector)
  hxVector3 tangent1[hxMAXCONTACT];

  /// \brief Description of contact frame relative to global frame:
  /// second tangent direction (unit vector)
  hxVector3 tangent2[hxMAXCONTACT];

  /// \brief Normal distance (penetration depth) in contact frame (m).
  float distance[hxMAXCONTACT];

  /// \brief Relative velocity in contact frame (m/s),
  /// with axis order (normal, tangent1, tangent2)
  hxVector3 velocity[hxMAXCONTACT];

  /// \brief Contact force in contact frame (N),
  /// with axis order (normal, tangent1, tangent2)
  hxVector3 force[hxMAXCONTACT];
};

/// \brief Information about the simulation camera. This is the camera
/// that generates the user's view.
struct _hxCamera
{
  /// \brief The position and orientation of the camera, relative the
  /// global coordiate frame.
  hxTransform transform;
};

/// \brief Simulation information
struct _hxSimInfo
{
  /// \brief Number of models in simulation.
  /// This defines the range of elements in the "models" array.
  int modelCount;

  /// \brief Array of models in simulation.
  /// \sa modelCount
  hxModel **models;

  /// \brief Information about the camera.
  /// \sa hxCamera
  hxCamera *camera;
};

/// \def hxCamera
/// \brief Information about the simulation camera.
typedef struct _hxCamera hxCamera;

/// \def hxSimInfo
/// \brief Information about the simulation world.
typedef struct _hxSimInfo hxSimInfo;

/// \def hxModel
/// \brief Information about simulated models.
typedef struct _hxModel hxBody;

/// \def hxLink
/// \brief Information about a link.
typedef struct _hxLink hxLink;

/// \def hxJoint
/// \brief Information about a joint.
typedef struct _hxJoint hxJoint;

/// \def hxContact
/// \brief Information about contacts.
typedef struct _hxContact hxContact;

// ---------- API functions ----------

/// \brief Get simulation information.
/// \param[out] _siminfo Simulation information requested.
/// \sa _hxSimInfo
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_siminfo(hxSimInfo *_siminfo);

/// \brief Get information about the simulation camera.
/// \param[out[ _camera Information about the simulation camera.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_camera(hxCamera *_camera);

/// \brief Set camera transform.
/// \param[in] _transform New camera transform
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_camera_transform(hxTransform _transform);

/// \brief Get information about active contacts.
/// \param[out] _contact The latest contact information.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_contacts(hxContact *_contact);

/// \brief Get Jacobian of global point attached to robot link
/// (index between 1 and njoint-1) size of Jacobian matrix is
/// 3-by-njoint, in row-major format.
/// \param[in] _link Model link.
/// \param[in] _point Point on the link.
/// \param[out] _jacobian Resulting jacobian matrix.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_jacobian(const hxLink *_link, const float *_point,
                      float *_jacobian);

/// \brief Set simulation state (position and velocity) as follows:
///   the robot base and objects are set from hxModel
///   the robot links are set from hxJoint via forward kinematics
///   the robot link data in hxBody, and all acceleration and torque
///   data are ignored.
/// \param[in] _model Model information to set.
/// \param[in] _joint Joint information to set.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_state(const hxModel *_model, const hxJoint *_joint);

/// \brief Add model during runtime.
/// \param[in] _urdf URDF xml description of the model.
/// \param[in] _x X position in global frame (m)
/// \param[in] _y Y position in global frame (m)
/// \param[in] _z Z position in global frame (m)
/// \param[in] _roll Roll in global frame (radians)
/// \param[in] _pitch Pitch in global frame (radians)
/// \param[in] _yaw Yaw in global frame (radians)
/// \return Pointer to the new model.
hxModel *hxs_add_model(const char *_urdf, float _x, float _y, float _z,
                       float _roll, float _pitch, float _yaw);

/// \brief Remove model.
/// \param[in] _id Id of the model.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_remove_model(int _id);

/// \brief Remove model.
/// \param[in] _model Pointer to the model.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_remove_model(const hxModel *_model);

/// \brief Set model pose.
/// \param[in] _id Id of the model.
/// \param[in] _transform New model transform
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_modeltransform(int _id, const hxTransform &_transform);

/// \brief Set the linear velocity of a model.
/// \param[in] _id Id of the model.
/// \param[in] _velocity Velocity (m/s)
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_linear_velocity(int _id, const hxVector3 &_velocity);

/// \brief Set the angular velocity of a model.
/// \param[in] _id Id of the model.
/// \param[in] _velocity Velocity (rad/s)
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_angular_velocity(int _id, const hxVector3 &_velocity);

/// \brief Set the linear acceleration on a model.
/// \param[in] _id Id of the model.
/// \param[in] _accel Acceleration (m/s/s)
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_linear_accel(int _id, const hxVector3 &_accel);

/// \brief Set the angular acceleration on a model.
/// \param[in] _id Id of the model.
/// \param[in] _accel Acceleration (rad/s/s)
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_angular_accel(int _id, const hxVector3 &_accel);

/// \brief Apply force to a link.
/// \param[in] _link Pointer to the link.
/// \param[in] _force Force (N)
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_force(const hxLink *_link, const hxVector3 &_force);

/// \brief Apply torque to a link.
/// \param[in] _link Pointer to the link.
/// \param[in] _torque Torque (N-m)
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_torque(const hxLink *_link, const hxVector3 &_torque);

/// \brief Send world reset command/Carry over limb pose between world reset.
/// \param[in] _rsetLimbPose True to reset the post of the limb.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_reset(bool _resetLimbPose);

/// \brief Reset on-screen timer.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_reset_timer();

/// \brief Start on-screen timer.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_start_timer();

/// \brief Stop on-screen timer.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_stop_timer();

/// \brief Start recording log file. Only one log file may be recorded at
/// a time
/// \param[in] _filename Name of the file to log information into.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_start_logging(const char *_filename);

/// \brief Determine if logging is running.
/// \return True if logging is running.
bool hxs_is_logging();

/// \brief Stop recording log file.
/// \return 'hxOK' if the function succeed or an error code otherwise.
hxResult hxs_stop_logging();
