%HX_CLOSE Disconnect from specified robot/simulator target
%
% result = hx_close(target)
%
% HX_CONNECT should have been called first.  After this call, do not call
% any other HAPTIX-related functions (except for HX_CONNECT).
%
% Parameters:
%   target : The device to be disconnected, which should be one of the following
%     integers:
%       0 : DEKA limb
%       1 : MPL limb
%       2 : Gazebo simulation
%       3 : MuJoCo simulation
%
% Return values:
%   result: 0 if the call was successful, 1 otherwise
