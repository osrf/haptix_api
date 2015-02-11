%HX_READ_SENSORS Read sensor data from simulator/robot.
%
% Similar to HX_UPDATE, this function reads the latest sensor information
% from the simulator/robot in a non-blocking manner. Use this function when
% you want to read the sensor data without sending a new command.  This
% function may be called as often as desired, but the robot is only
% guaranteed to update at the rate returned from HX_ROBOT_INFO.
%
% HX_CONNECT should have been called first.
%
% sensor = hx_read_sensors()
%
% Return values: 
%   sensor: See documentation for HX_UPDATE
%
% Throws an error if something failed.

