%HX_CONNECT_HOST Connect to specified robot/simulator target with host and port
%
% result = hx_connect(target, host, port)
%
% This function should be called before any other HAPTIX-related functions.
%
% Parameters:
%   target : The device to be connected, which should be one of the following
%     integers:
%       0 : DEKA limb
%       1 : MPL limb
%       2 : Gazebo simulation
%       3 : MuJoCo simulation
%   host: When connecting to a simulator, use host to specify the machine
%     that is running the simulator.
%   port: When connecting to a simulator, use _port to specify
%     what port the simulator is running on.
%
% Return values:
%   result: 0 if the call was successful, 1 otherwise
