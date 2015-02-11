%HX_UPDATE Send motor command and receive sensor state.
%
% A non-blocking function that sends a command to the robot and receives
% the latest sensor information. This function may be used as frequently as
% desired, but the simulator/robot is only guaranteed to update at the rate
% returned from HX_ROBOT_INFO.
%
% HX_CONNECT should have been called first.
%
% sensor = hx_update(command)
%
% Parameters:
%   command: Structure with the following named fields:
%     ref_pos (float array) : An N by 1 array of desired positions for the
%       motors (rad).
%     ref_pos_enabled (bool) : If true, then the values in ref_pos should be
%       used; otherwise, they should be ignored.
%     ref_vel_max (float array) : An N by 1 array of desired maximum angular
%       velocities for the motors (rad/s).
%     ref_vel_max_enabled (bool) : If true, then the values in ref_vel_max
%       should be used; otherwise, they should be ignored.
%     gain_pos (float array) : An N by 1 array of position gains to be applied
%       to the internal controller (Nm/rad).
%     gain_pos_enabled (bool) : If true, then the values in gain_pos
%       should be used; otherwise, they should be ignored.
%     gain_vel (float array) : An N by 1 array of velocity gains to be applied
%       to the internal controller (Nms/rad).
%     gain_vel_enabled (bool) : If true, then the values in gain_vel
%       should be used; otherwise, they should be ignored.
%     
%     
% Return values:
%   sensor: Structure with the following named fields:
%     time_stamp (float) : Time at which the sensor reading was taken (s).
%     motor_pos (float array) : An N by 1 array of motor positions (rad).
%     motor_vel (float array) : An N by 1 array of motor velocities (rad/s).
%     motor_torque (float array) : An N by 1 array of motor torques (Nm).
%     joint_pos (float array) : An M by 1 array of joint positions (rad).
%     joint_vel (float array) : An M by 1 array of joint velocities (rad/s).
%     contact (float array) : An L by 1 array of contact force magnitudes (N).
%     imu_linear_acc (float array) : A K by 3 array of IMU accelerometer
%       data (m/s^2).  Each row is a 3-dimensional vector of accelerometer
%       output, which comprises the vector difference (a-g), where a is the
%       linear acceleration and g is the gravity vector.  This measurement
%       is expressed in a body-fixed frame.  The entries of each row are
%       measured in meters per second squared and ordered (x, y, z).
%     imu_angular_vel (float array) : A K by 3 array of IMU gyro data (rad/s).
%       Each row is a 3-dimensional angular velocity vector.  This measurement
%       is expressed in a body-fixed frame.  The entries of each row are
%       measured in radians per second and ordered (x, y, z).
%     imu_orientation (float array) : A K by 4 array of IMU orientation data
%       (quaternion).  Each row provides a 4-element quaternion representation
%       of the estimated orientation of the corresponding IMU.  A given
%       simulator/robot might not support this functionality.
%
% Throws an error if something failed.
