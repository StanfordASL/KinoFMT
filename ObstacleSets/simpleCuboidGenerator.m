% simpleCuboidGenerator generates random cuboids in a form that can be 
% processed by a GetCuboidObstacles function
%
%   Ross Allen, ASL, Stanford University
%   May 15, 2014
%
%   Notes:
%       - Obstacles only in xyz-space
%       - Automatically adds a center obstacle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [LWH_F, YPR_F, CM_F] = simpleCuboidGenerator(nObstacles, bounds, minSize, maxSize)
LWH_F = zeros(3,nObstacles-1);
YPR_F = zeros(3,nObstacles);
CM_F = zeros(3,nObstacles-1);
lengths = bounds(:,2)-bounds(:,1);
lengths = lengths - maxSize*ones(3,1);
for i = 1:nObstacles-1
    LWH_F(:,i) = rand(3,1)*(maxSize-minSize) + minSize*ones(3,1);
    CM_F(:,i) = rand(3,1).*lengths+(bounds(:,1)+0.5*maxSize*ones(3,1));
end

% add center obstacle
LWH_F = [LWH_F, rand(3,1)*(maxSize-minSize) + minSize*ones(3,1)'];
CM_F = [CM_F, [0 0 0]'];
end