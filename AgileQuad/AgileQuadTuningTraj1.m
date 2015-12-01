% AgileQuadTuningTrajectory1 generates a 'benchmark' trajectory to use for
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

% input data data
% mpinfo.obstacles.cuboids.ulVerts = [];
% mpinfo.environment.bounds = ...
%     [   -0.2    2.8;...
%         0.14    4.25;...
%         -2.75   -0.5    ]; %(m)
% mpinfo.smoother.yaw = -1.68;
% mpinfo.sampling.nTrajNodes = 10;
% mpinfo.smoother.timeScaling = 1;
%% mpinfo.optimalPath = ...
%%     [   1   2;...
%%         2   3;...
%%         3   4;...
%%         4   5   ];
% mpinfo.optPath = [ 1; 2; 3; 4; 5];
% mpinfo.evalMat = zeros(size(mpinfo.optPath,1),...
%     size(mpinfo.optPath,1));
% mpinfo.evalMat(1,2) = 1;
% mpinfo.evalMat(2,3) = 2;
% mpinfo.evalMat(3,4) = 3;
% mpinfo.evalMat(4,5) = 4;

mpinfo.comms.commfile = '../../AgileQuad/ViconWifiComm/Debug/trajectory.txt';

keys =[ 1.63    3.06    -1.2;...
        2.25    2.06    -1.2;...
        1.63    1.06    -1.2;...
        0.62    2.06    -1.2;...
        1.63    3.06    -1.2 ];

Tdel = [1.0; 1.0; 1.0; 1.0];
Tdel = 1*Tdel;

yaw = -1.68;

% generate smooth spline trajectory
mpinfo.smoother = DblIntQuadSplineSmoother(keys, Tdel, yaw);

% pass smooth spline trjaectory in text file
DblIntQuadCommunicator(mpinfo);
