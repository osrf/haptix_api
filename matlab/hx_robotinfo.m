%HX_ROBOTINFO Get information for a specified robot/simulator target.
%
% [info, result] = hx_robotinfo(target)
%
% Parameters:
%   target : The device to be connected, which should be one of the following
%     integers:
%       0 : DEKA limb
%       1 : MPL limb
%       2 : Gazebo simulation
%       3 : MuJoCo simulation
%
% Return values:
%   info: Structure with the following named fields:
%     nmotor : Number of motors.  The number of motors is less than or equal
%       to the number of joints. For example, one motor may control
%       several joints through kinematic joint coupling.
%     njoint : Number of joints.  The number of joints is greater than or equal
%       to the number of motors.
%     ncontactsensor : Number of contact sensors.  A contact sensor measures
%       the magnitude of the force on that sensor.
%     nIMU : Number of IMUs (inertial measurement units).  An IMU or inertial
%       measurement unit measures the 3-dimensional linear acceleration
%       vector and the 3-dimensional angular velocity vector experienced by
%       the sensor.
%     motorlimits : An m by 2 array representing the angular limits of each
%       motor in the robot, where m is the maximum number of motors. Each
%       1x2 row of the array corresponds to a motor. The first entry in the
%       row is the lower limit of the motor. The second entry is the upper
%       limit of the motor.
%     jointlimits : Minimum and maximum joint angles (rad).  An n by 2
%       array representing the angular limits of each joint in the robot,
%       where n is the maximum number of joints. Each 1x2 row of the array
%       corresponds to a joint. The first entry in the row is the lower
%       limit of the joint. The second entry is the upper limit of the joint.
%
%   result: 0 if the call was successful, 1 otherwise
