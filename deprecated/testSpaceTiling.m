function testSpaceTiling()

LWH_F = [[.5 .5 3]', ...
    [2 1 3]', ...
    [1 1 3]'];
YPR_F = [[0 0 0]', ...
    [0 0 0]', ...
    [0 0 0]'];
CM_F = [[.5 .5 1.5]',...
    [1.5 2 1.5]', ...
    [2.5 .5 1.5]'];
bounds = [10 -10; 10 -10; 3 0];
% bounds = [3 0; 3 0; 3 0];

% LWH_F       = [[30 14 20]', ...                 % Length/Width/Height of fixed cuboid obstacles (m) (m-th COLUMN vector corresponds to m-th obstacle)
%     [26 16 36]', ...
%     [45 35 55]', ...
%     [35 42 28]', ...
%     [40 53 46]'];
% 
% %50 + 10.*randn([3,3])];
% YPR_F       = [[ 30  10  20]', ...              % Yaw/Pitch/Roll of fixed cuboid obstacles (deg) (m-th COLUMN vector corresponds to m-th obstacle)
%     [-10  15 -40]', ...
%     [ 42 -18  27]', ...
%     [-45  36  65]', ...
%     [-95 -07  05]'];
% %360.*(rand([3,3]) - 0.5)];
% CM_F        = [[100 -40 9]', ...                % Position vector of fixed cuboid obstacle centroid [m] (m-th COLUMN vector corresponds to m-th obstacle)
%     [-70  53 21]', ...
%     [-100 -100 15]', ...
%     [-110 45 -20]', ...
%     [-80 -25 8 ]'];
% bounds      = [ [ 2000  2000  2000]', ...       % Upper (1st column) and lower (2nd column) bounds of the sample space [m,m,m,m/s,m/s,m/s]
%                 [-2000 -2000 -2000]'];          

% Create obstacle regions
F   = cell(1,size(LWH_F,2));    obstacles   = cell(1,size(LWH_F,2));
for i = 1:size(LWH_F,2)
    [V{i}, ~, ~,~,~, F{i}] = polymodel( cuboid( LWH_F(1,i),LWH_F(2,i),LWH_F(3,i),'hideplot',CM_F(:,i),euler2rotmat( YPR_F(:,i),'321','Display','off' ) ), 'hideplot', 'hidedisp' );
%     [V{i}, ~, ~, Cn1{i}, C1{i}, F{i}] = polymodel(cuboid(LWH_F(1,i),LWH_F(2,i),'hideplot',CM_F(:,i)));
    obstacles.cuboids.vertices{i} = V{i};
    obstacles.cuboids.faces{i} = F{i};
end

disp('This state should not be collided:')
isStateCollided([0 0 0]', obstacles, 3, bounds)
disp('These states should be collided:')
isStateCollided([.5 .5 1.5]', obstacles, 3, bounds)
isStateCollided([2 2 2]', obstacles, 3, bounds)
isStateCollided([2 2 0]', obstacles, 3, bounds)
isStateCollided([2 2 3]', obstacles, 3, bounds)
isStateCollided([.5 .5 0]', obstacles, 3, bounds)
isStateCollided([2.5 1 0]', obstacles, 3, bounds)
isStateCollided([2 .5 0]', obstacles, 3, bounds)
isStateCollided([4 4 4]', obstacles, 3, bounds)

end
