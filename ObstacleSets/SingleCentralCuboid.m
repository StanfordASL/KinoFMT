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

function [LWH_F, YPR_F, CM_F] = SingleCentralCuboid(obsinfo)

minSize = obsinfo.minCuboidDimension;
maxSize = obsinfo.maxCuboidDimension;

% add center obstacle
LWH_F = [rand(3,1)*(maxSize-minSize) + minSize*ones(3,1)];
YPR_F = zeros(3,1);


% try to use obstacle space to find center
try
    xCen = (obsinfo.obstacleSpace(1,1)+obsinfo.obstacleSpace(1,2))/2;
    yCen = (obsinfo.obstacleSpace(2,1)+obsinfo.obstacleSpace(2,2))/2;
    zCen = (obsinfo.obstacleSpace(3,1)+obsinfo.obstacleSpace(3,2))/2;
    CM_F = [xCen yCen zCen]';
catch
    CM_F = [0 0 0]';
end
end