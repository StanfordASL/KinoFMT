% IndoorObstacleSet5 gives "maze" set of obstacles for the ASL vicon arena
%   in upper/lower vertice form
%
%   AUTHOR: Ross Allen, ASL, Stanford University
%   DATE:   Oct 7, 2015
%
%   Notes:
%       - Parallel walls with doors on either side
%       - Dimension to match ASL lab
%       - Thin netting to be used
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ulVerts = IndoorObstacleSet5(obstacles)

ulVerts = [
    -1.000,     0.7087,     -2.5500;...
    2.4670,     1.3947,     0.0000];

ulVerts = [ulVerts;...
    -0.0314,    2.4791,     -2.5500;...
    3.0000,     3.1651,     0.0000];     
end