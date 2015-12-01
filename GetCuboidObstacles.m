% GetCuboidObstacles generates and and stores cuboid obstacles
% (obstacles only in xyz-space)
%
%   Ross Allen, ASL, Stanford University
%   Dec 10, 2014
%
%   Notes:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function mpinfo = GetCuboidObstacles(mpinfo)

% Unpack variables to accessed and modified
obstacles = mpinfo.obstacles;

% Unpack variables to be accessed (not modified)

% Remove existing cuboid field
if isfield(obstacles, 'cuboids')
    obstacles = rmfield(obstacles, 'cuboids');
end

% Call Cuboid Generator Function
try 
    % Try old format of obstacles first and convert to new form
    [LWH_F, YPR_F, CM_F] = obstacles.generationFunction(obstacles);
    obstacles.cuboids.ulVerts = ...
        ObstacleFormatConversion(LWH_F, YPR_F, CM_F);
catch
    % If old format failed, generationFunction outputs axis aligned obs
    obstacles.cuboids.ulVerts = obstacles.generationFunction(obstacles);
end


% Store Variables for return
mpinfo.obstacles = obstacles;


end
