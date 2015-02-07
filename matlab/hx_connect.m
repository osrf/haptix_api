%HX_CONNECT Connect to specified robot/simulator target
%
% result = hx_connect(target)
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
%
% Return values:
%   result: 0 if the call was successful, 1 otherwise
