% ISSObstacleGenerator gives the rough dimensions of the
% International Space Station in a form that can be handled by
% DeepSpace_FMTPlanningScript
%
%   AUTHOR: Ross Allen, ASL, Stanford University
%   DATE:   Mar 12, 2014
%   REFERENCE: 3dwarehouse.sketchup.com -> ISS {Part 13 Compleet}
%
%   Notes:
%       - This is identical to DeepSpace_ISSObstacleScript but in function
%       form
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [LWH_F, YPR_F, CM_F] = ISSObstacleGenerator(obstacles);

obstacleScaleFactor = 1487;

% Length/Width/Height of fixed cuboid obstacles (m) (m-th COLUMN vector corresponds to m-th obstacle)
LWH_F = [...
    [0.019, 0.049, 0.002]', ...
    [0.019, 0.049, 0.002]', ...
    [0.063, 0.002, 0.002]', ...
    [0.003, 0.033, 0.003]', ...
    [0.011, 0.003, 0.008]', ...
    [0.003, 0.003, 0.017]', ...
    [0.003, 0.003, 0.024]', ...
    [0.027, 0.011, 0.002]'];
LWH_F = obstacleScaleFactor*LWH_F;

% Yaw/Pitch/Roll of fixed cuboid obstacles (deg) (m-th COLUMN vector corresponds to m-th obstacle)
YPR_F = [...
    [0 0 0]', ...
    [0 0 0]', ...
    [0 0 0]', ...
    [0 0 0]', ...
    [0 0 0]', ...
    [0 0 0]', ...
    [0 0 0]', ...
    [0 0 0]'];

% Position vector of fixed cuboid obstacle centroid [m] (m-th COLUMN vector corresponds to m-th obstacle)
CM_F = [...
    [-0.026, 0, 0]',...
    [0.026, 0, 0]', ...
    [0, 0, 0]', ...
    [0, 0.007, -0.003]', ...
    [0, -0.007, 0]', ...
    [0, 0.004, -0.007]', ...
    [0, 0.016, 0]', ...
    [0, 0.016, 0.012]'];
CM_F = obstacleScaleFactor*CM_F;

end