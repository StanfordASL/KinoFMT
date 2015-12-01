% SingleCentralCuboid generates a single cuboid in the center of the
% obstacle space
%
%   Ross Allen, ASL, Stanford University
%   May 15, 2014
%
%   Notes:
%       - Obstacles only in xyz-space
%       - Automatically adds a center obstacle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [LWH_F, YPR_F, CM_F] = EmptyObstacleSet(obsinfo)

% add center obstacle
LWH_F = [];
YPR_F = [];
CM_F = [];
end