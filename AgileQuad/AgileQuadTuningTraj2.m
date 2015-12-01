% AgileQuadTuningTrajectory2 generates a 'benchmark' trajectory to use for
% tuning the position controller of the AgileQuad
%
%   AUTHOR: Ross Allen
%   DATE:   July 22, 2015
%
%   NOTES:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc

addpath([pwd, '/../DblIntQuad/']);

mpinfo.comms.commfile = '../../AgileQuad/ViconWifiComm/Debug/trajectory.txt';

keys =[ 2.06    3.17    -1.25;...
        1.76    2.98    -1.25;...
        0.35    2.02    -1.25;...
        1.14    1.64    -1.25;...
        2.00    1.25    -1.25 ];

Tdel = [0.612; 1.419; 1.041; 1.086];
Tdel = Tdel;

yaw = -1.68;

% generate smooth spline trajectory
mpinfo.smoother = DblIntQuadSplineSmoother(keys, Tdel, yaw);

% pass smooth spline trjaectory in text file
DblIntQuadCommunicator(mpinfo);
