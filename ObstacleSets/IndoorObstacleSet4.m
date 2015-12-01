% IndoorObstacleSet4 gives a set of obstacles for the ASL vicon arena
%
%   AUTHOR: Ross Allen, ASL, Stanford University
%   DATE:   Oct 7, 2015
%
%   Notes:
%       - Parallel walls with doors on either side
%       - Dimension to match ASL lab
%       - Thin netting to be used
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [LWH_F, YPR_F, CM_F] = IndoorObstacleSet4(obstacles)

% envL = abs(obstacles.obstacleSpace(1,1) ...
% 	- obstacles.obstacleSpace(1,2));% (m) length of environment (x)
% envW = abs(obstacles.obstacleSpace(2,1) ...
% 	- obstacles.obstacleSpace(2,2));% (m) length of environment (y)
% envH = abs(obstacles.obstacleSpace(3,1) ...
% 	- obstacles.obstacleSpace(3,2));% (m) length of environment (z)

% Length/Width/Height of fixed cuboid obstacles (-) (m-th COLUMN vector corresponds to m-th obstacle)
LWH_F = [...
    [2.9323, 0.686, 2.55]', ...
    [2.8314, 0.686, 2.55]'];
% nObs = size(LWH_F,2);
% LWH_F = LWH_F.*repmat([envL, envW, envH]', 1, nObs);

% Yaw/Pitch/Roll of fixed cuboid obstacles (deg) (m-th COLUMN vector corresponds to m-th obstacle)
YPR_F = zeros(3, 2);

% Position vector of fixed cuboid obstacle centroid [m] (m-th COLUMN vector corresponds to m-th obstacle)
CM_F = [...
    [0.9962, 1.0517, -1.275]',...	
    [1.3843, 2.8221, -1.275]'];
% CM_F = CM_F.*repmat([envL, envW, envH]', 1, nObs) + ...
% 	repmat([obstacles.obstacleSpace(1,1), ...
% 	 obstacles.obstacleSpace(2,1), ...
% 	obstacles.obstacleSpace(3,1)]', 1, nObs);

end