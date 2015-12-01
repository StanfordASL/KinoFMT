% DSSCGetCuboidObstacles generates and and stores cuboid obstacles
% (obstacles only in xyz-space)
%
%   Ross Allen, ASL, Stanford University
%   Sep 24, 2014
%
%   Notes:
%       - COULD BE GENERALIZED TO A SYSTEM-AGNOSTIC FUNCTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function mpinfo = DSSCGetCuboidObstacles(mpinfo)

% Unpack variables to accessed and modified
obstacles = mpinfo.obstacles;

% Unpack variables to be accessed (not modified)

% Check inputs
if size(obstacles.obstacleSpace,1) > size(mpinfo.sampling.stateSampleRange,1)
    disp('Obstacle space must have less than or equal number of dimensions')
    disp('as the state space')
    disp('Exiting DSSCGetCuboidObstacles prematurely...')
    return
end


% Call Cuboid Generator Function
[LWH_F, YPR_F, CM_F] = obstacles.generationFunction(obstacles);

% Remove existing cuboid field
if isfield(obstacles, 'cuboids')
    obstacles = rmfield(obstacles, 'cuboids');
end

% Form obstacles variable so it can be used by areStatesCollided
obstacleFaces   = cell(1,size(LWH_F,2)); 
obstacleVertices = cell(1,size(LWH_F,2)); 
obstacleFaceVertexIndices = cell(1,size(LWH_F,2));
for i = 1:size(LWH_F,2)
    [~, ~, ~, obstacleFaceVertexIndices{i}, obstacleVertices{i}, obstacleFaces{i}] = ...
        polymodel(cuboid(LWH_F(1,i),LWH_F(2,i),LWH_F(3,i),'hideplot', ...
        CM_F(:,i),euler2rotmat( YPR_F(:,i),'321','Display','off')), ...
        'hideplot', 'hidedisp' );
    obstacles.cuboids.vertices{i} = obstacleVertices{i};
    obstacles.cuboids.faces{i} = obstacleFaces{i};
    obstacles.cuboids.faceVertexIndices{i} = obstacleFaceVertexIndices{i};
end


% Store Variables for return
mpinfo.obstacles = obstacles;


end

