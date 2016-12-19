%PlotCuboidObstacles generates a plot of obstacles 
%
%   Ross Allen, ASL, Stanford University
%
%   Started: 
%   Modified:   Dec 10, 2014
%
%   Inputs:
%
%   Outputs:
%
%   Notes:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function err = PlotCuboidObstacles(varargin)

err = 1;
facealpha = 1.0;

% Check if axis aligned obstacle matrix exists
if length(varargin)== 1 || length(varargin) == 2
    
    if length(varargin) == 2
        facealpha = varargin{2};
    end
    
    ulVerts = varargin{1};
    err = 2;
    % Check the dimension of the upper-lower
    if size(ulVerts,2) == 3
        
        % Setup figure
%         figure
        xmin = min(ulVerts(:,1));
        xmax = max(ulVerts(:,1));
        ymin = min(ulVerts(:,2));
        ymax = max(ulVerts(:,2));
        zmin = min(ulVerts(:,3));
        zmax = max(ulVerts(:,3));
        % loop through each obstacle
        for i=1:2:size(ulVerts,1)-1
            verts = zeros(8,3);
            verts(1:4,1) = ulVerts(i,1);
            verts(5:8,1) = ulVerts(i+1,1);
            verts([1 2 5 6], 2) = ulVerts(i,2);
            verts([3 4 7 8], 2) = ulVerts(i+1,2);
            verts([1 3 5 7], 3) = ulVerts(i,3);
            verts([2 4 6 8], 3) = ulVerts(i+1,3);
            [Acol, bcol, ~, ~] = vert2lcon(verts);
            plotregion(-Acol, -bcol, [], [], [0.7,0.2,0.3],facealpha);
        end
        xlim([xmin, xmax]);
        ylim([ymin, ymax]);
        zlim([zmin, zmax]);
        axis equal
        
        err = 0;
    end
    
    return;
end

% If not axis aligned, attempt LWH/YPR/CM obstacles
if length(varargin)== 3
    LWH_F = varargin{1};
    YPR_F = varargin{2};
    CM_F = varargin{3};
    % Form obstacles variable so it can be used by areStatesCollided
    obstacleFaces   = cell(1,size(LWH_F,2));
    obstacleVertices = cell(1,size(LWH_F,2));
    obstacleFaceVertexIndices = cell(1,size(LWH_F,2));
    for i = 1:size(LWH_F,2)
        [~, ~, ~, obstacleFaceVertexIndices{i}, obstacleVertices{i},...
        obstacleFaces{i}] = polymodel(cuboid(LWH_F(1,i),...
        LWH_F(2,i), LWH_F(3,i), 'hideplot', CM_F(:,i),...
        euler2rotmat( YPR_F(:,i),'321','Display','off')),...
        'hideplot', 'hidedisp' );
    end

    % Plot
    figure
    hold on
    verts = cell2mat(obstacleVertices);
    xmin = min(verts(1,:));
    xmax = max(verts(1,:));
    ymin = min(verts(2,:));
    ymax = max(verts(2,:));
    zmin = min(verts(3,:));
    zmax = max(verts(3,:));
    for i = 1:length(obstacleVertices)
        [Acol, bcol, ~, ~] = vert2lcon(obstacleVertices{i}');
        plotregion(-Acol, -bcol, [], [], [0.7,0.2,0.3],facealpha);
    end
    xlim([xmin, xmax]);
    ylim([ymin, ymax]);
    zlim([zmin, zmax]);
    axis equal
end
end
