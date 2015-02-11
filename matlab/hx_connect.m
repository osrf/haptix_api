%HX_CONNECT Connect to robot/simulator
%
% hx_connect(host, port)
%
% This function should be called before any other HAPTIX-related functions.
%
% Parameters:
%   host (string) : The name/address of the simulator/robot to use.
%   port (int) : The TCP/UDP port of the simulator/robot to use.
%
% Not all simulators/robots require the host and/or port parameters.  Use
% empty/zero values to use automatic discovery.
%
% Throws an error if something failed.
