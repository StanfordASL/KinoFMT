
function SimpleUAVPlotMotionPlan(mpinfo)

% Set up variables
P = mpinfo.optimalPath;
E = mpinfo.explorationTree;
xyz_bounds = mpinfo.sampling.stateSampleRange;
explorationTree = mpinfo.explorationTree;
pathIndices = mpinfo.optimalPath;
stateMat = mpinfo.stateMat;
trajMat = mpinfo.trajMat;
evalMat = mpinfo.evalMat;
obstacles = mpinfo.obstacles;
nTrajNodes = mpinfo.sampling.nTrajNodes;

x_0 = stateMat(1,:)';
x_f = stateMat(pathIndices(end,2),:)';
x   = stateMat( pathIndices, : )';

figure; hold on;
plot3( x_0(1), x_0(2), x_0(3), '-og', 'Color', [0 0.5 0], 'Linewidth', 2, 'MarkerSize', 7 );
plot3( x_f(1), x_f(2), x_f(3), '-or', 'Linewidth', 2, 'MarkerSize', 7 );
% plot3( x(1,:), x(2,:), x(3,:), '-b');
for i = 1:size(pathIndices,1)
    BVPnum = mpinfo.evalMat(pathIndices(i,1),pathIndices(i,2));
    plot3(reshape(mpinfo.trajMat(BVPnum,1,:),[],1),...
        reshape(mpinfo.trajMat(BVPnum,2,:),[],1),...
        reshape(mpinfo.trajMat(BVPnum,3,:),[],1),'.-b')
    plot3(reshape(mpinfo.trajMat(BVPnum,1,1),[],1),...
        reshape(mpinfo.trajMat(BVPnum,2,1),[],1),...
        reshape(mpinfo.trajMat(BVPnum,3,1),[],1),'ob')
end

try
    % try new obstacle format (axis aligned obstacles only)
    if isfield(mpinfo.obstacles, 'cuboids')
        PlotCuboidObstacles(mpinfo.obstacles.cuboids.ulVerts);
    end
catch
    % go back to old obstacle format
    if isfield(mpinfo.obstacles, 'cuboids')
        faceVertexIndices       = obstacles.cuboids.faceVertexIndices;  % Cn1
        faceVertexCoordinates   = obstacles.cuboids.vertices;           % C1
        Ncuboids = length(faceVertexIndices);
        for j = 1:Ncuboids
            cuboidFaceVertexCoordinates = faceVertexCoordinates{j};
            cuboidFaceVertexIndices     = faceVertexIndices{j};
            NfaceVertices               = length( cuboidFaceVertexIndices );
            for k = 1:NfaceVertices
                pts = sum( cuboidFaceVertexIndices(1:k) ) - cuboidFaceVertexIndices(1) + (1:1:cuboidFaceVertexIndices(k));
                patch( cuboidFaceVertexCoordinates(1,pts), cuboidFaceVertexCoordinates(2,pts), cuboidFaceVertexCoordinates(3,pts), ...
                    [0.7,0.2,0.3], 'FaceAlpha', 0.9);
            end
        end
    end
end

% u_T = explorationTree.*bsxfun(@rdivide, u_star, norms(u_star,2,1));
% quiver3( x(1,1:end-1), x(2,1:end-1), x(3,1:end-1), u_T(1,:), u_T(2,:), u_T(3,:), 0, 'Color', [0 0.5 0]);
% xlabel('x(t)');
% ylabel('y(t)');
% zlabel('z(t)');
% legend('x_{init}', 'x_{goal}', 'Motion Plan', 'Location', 'EastOutside');
% grid on; axis equal; %view([-20 20]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Trajectory (Multi View)
figure('Color',[1 1 1])
subplot(2,2,1)
hold on
try
    % try new obstacle format (axis aligned obstacles only)
    if isfield(mpinfo.obstacles, 'cuboids')
        PlotCuboidObstacles(mpinfo.obstacles.cuboids.ulVerts);
    end
catch
    % go back to old obstacle format
    if isfield(mpinfo.obstacles, 'cuboids')
        faceVertexIndices       = obstacles.cuboids.faceVertexIndices;  % Cn1
        faceVertexCoordinates   = obstacles.cuboids.vertices;           % C1
        Ncuboids = length(faceVertexIndices);
        for j = 1:Ncuboids
            cuboidFaceVertexCoordinates = faceVertexCoordinates{j};
            cuboidFaceVertexIndices     = faceVertexIndices{j};
            NfaceVertices               = length( cuboidFaceVertexIndices );
            for k = 1:NfaceVertices
                pts = sum( cuboidFaceVertexIndices(1:k) ) - cuboidFaceVertexIndices(1) + (1:1:cuboidFaceVertexIndices(k));
                patch( cuboidFaceVertexCoordinates(1,pts), cuboidFaceVertexCoordinates(2,pts), cuboidFaceVertexCoordinates(3,pts), ...
                    [0.7,0.2,0.3], 'FaceAlpha', 0.9);
            end
        end
    end
end
plot3(stateMat(1,1),stateMat(1,2),stateMat(1,3),'g^',...
    'MarkerFaceColor','g', 'MarkerSize', 10)
for i = 1:size(P,1)
    plot3(stateMat(P(i,2),1),stateMat(P(i,2),2),stateMat(P(i,2),3),'b*')
    tempVar = reshape(trajMat(evalMat(P(i,1),P(i,2)),1:3,:),3,nTrajNodes)';
    plot3(tempVar(:,1),tempVar(:,2),tempVar(:,3),'k.-')
end
plot3(stateMat(2,1),stateMat(2,2),stateMat(2,3),'ro',...
    'MarkerFaceColor','r', 'MarkerSize', 10)
xlim(xyz_bounds(1,:))
ylim(xyz_bounds(2,:))
zlim(xyz_bounds(3,:))
xlabel('x-pos')
ylabel('y-pos')
zlabel('z-pos')
axis off
axis equal

subplot(2,2,3)
hold on
try
    % try new obstacle format (axis aligned obstacles only)
    if isfield(mpinfo.obstacles, 'cuboids')
        PlotCuboidObstacles(mpinfo.obstacles.cuboids.ulVerts);
    end
catch
    % go back to old obstacle format
    if isfield(mpinfo.obstacles, 'cuboids')
        faceVertexIndices       = obstacles.cuboids.faceVertexIndices;  % Cn1
        faceVertexCoordinates   = obstacles.cuboids.vertices;           % C1
        Ncuboids = length(faceVertexIndices);
        for j = 1:Ncuboids
            cuboidFaceVertexCoordinates = faceVertexCoordinates{j};
            cuboidFaceVertexIndices     = faceVertexIndices{j};
            NfaceVertices               = length( cuboidFaceVertexIndices );
            for k = 1:NfaceVertices
                pts = sum( cuboidFaceVertexIndices(1:k) ) - cuboidFaceVertexIndices(1) + (1:1:cuboidFaceVertexIndices(k));
                patch( cuboidFaceVertexCoordinates(1,pts), cuboidFaceVertexCoordinates(2,pts), cuboidFaceVertexCoordinates(3,pts), ...
                    [0.7,0.2,0.3], 'FaceAlpha', 0.9);
            end
        end
    end
end
plot3(stateMat(1,1),stateMat(1,2),stateMat(1,3),'g^',...
    'MarkerFaceColor','g', 'MarkerSize', 10)
for i = 1:size(P,1)
    plot3(stateMat(P(i,2),1),stateMat(P(i,2),2),stateMat(P(i,2),3),'b*')
    tempVar = reshape(trajMat(evalMat(P(i,1),P(i,2)),1:3,:),3,nTrajNodes)';
    plot3(tempVar(:,1),tempVar(:,2),tempVar(:,3),'k.-')
end
plot3(stateMat(2,1),stateMat(2,2),stateMat(2,3),'ro',...
    'MarkerFaceColor','r', 'MarkerSize', 10)
xlim(xyz_bounds(1,:))
ylim(xyz_bounds(2,:))
zlim(xyz_bounds(3,:))
xlabel('x-pos')
ylabel('y-pos')
zlabel('z-pos')
view(0,0)
axis off
axis equal

subplot(2,2,2)
hold on
try
    % try new obstacle format (axis aligned obstacles only)
    if isfield(mpinfo.obstacles, 'cuboids')
        PlotCuboidObstacles(mpinfo.obstacles.cuboids.ulVerts);
    end
catch
    % go back to old obstacle format
    if isfield(mpinfo.obstacles, 'cuboids')
        faceVertexIndices       = obstacles.cuboids.faceVertexIndices;  % Cn1
        faceVertexCoordinates   = obstacles.cuboids.vertices;           % C1
        Ncuboids = length(faceVertexIndices);
        for j = 1:Ncuboids
            cuboidFaceVertexCoordinates = faceVertexCoordinates{j};
            cuboidFaceVertexIndices     = faceVertexIndices{j};
            NfaceVertices               = length( cuboidFaceVertexIndices );
            for k = 1:NfaceVertices
                pts = sum( cuboidFaceVertexIndices(1:k) ) - cuboidFaceVertexIndices(1) + (1:1:cuboidFaceVertexIndices(k));
                patch( cuboidFaceVertexCoordinates(1,pts), cuboidFaceVertexCoordinates(2,pts), cuboidFaceVertexCoordinates(3,pts), ...
                    [0.7,0.2,0.3], 'FaceAlpha', 0.9);
            end
        end
    end
end
plot3(stateMat(1,1),stateMat(1,2),stateMat(1,3),'g^',...
    'MarkerFaceColor','g', 'MarkerSize', 10)
for i = 1:size(P,1)
    plot3(stateMat(P(i,2),1),stateMat(P(i,2),2),stateMat(P(i,2),3),'b*')
    tempVar = reshape(trajMat(evalMat(P(i,1),P(i,2)),1:3,:),3,nTrajNodes)';
    plot3(tempVar(:,1),tempVar(:,2),tempVar(:,3),'k.-')
end
plot3(stateMat(2,1),stateMat(2,2),stateMat(2,3),'ro',...
    'MarkerFaceColor','r', 'MarkerSize', 10)
xlim(xyz_bounds(1,:))
ylim(xyz_bounds(2,:))
zlim(xyz_bounds(3,:))
xlabel('x-pos')
ylabel('y-pos')
zlabel('z-pos')
view(111,32)
axis off
axis equal

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tree
figure('Color',[1 1 1])
hold on
try
    % try new obstacle format (axis aligned obstacles only)
    if isfield(mpinfo.obstacles, 'cuboids')
        PlotCuboidObstacles(mpinfo.obstacles.cuboids.ulVerts);
    end
catch
    % go back to old obstacle format
    if isfield(mpinfo.obstacles, 'cuboids')
        faceVertexIndices       = obstacles.cuboids.faceVertexIndices;  % Cn1
        faceVertexCoordinates   = obstacles.cuboids.vertices;           % C1
        Ncuboids = length(faceVertexIndices);
        for j = 1:Ncuboids
            cuboidFaceVertexCoordinates = faceVertexCoordinates{j};
            cuboidFaceVertexIndices     = faceVertexIndices{j};
            NfaceVertices               = length( cuboidFaceVertexIndices );
            for k = 1:NfaceVertices
                pts = sum( cuboidFaceVertexIndices(1:k) ) - cuboidFaceVertexIndices(1) + (1:1:cuboidFaceVertexIndices(k));
                patch( cuboidFaceVertexCoordinates(1,pts), cuboidFaceVertexCoordinates(2,pts), cuboidFaceVertexCoordinates(3,pts), ...
                    [0.7,0.2,0.3], 'FaceAlpha', 0.9);
            end
        end
    end
end
plot3(stateMat(1,1),stateMat(1,2),stateMat(1,3),'g^',...
    'MarkerFaceColor','g', 'MarkerSize', 10)
for i = 1:size(E,1)
    plot3(stateMat(E(i,2),1),stateMat(E(i,2),2),stateMat(E(i,2),3),'b*','MarkerSize', 0.1)
    tempVar = reshape(trajMat(evalMat(E(i,1),E(i,2)),1:3,:),3,nTrajNodes)';
    plot3(tempVar(:,1),tempVar(:,2),tempVar(:,3),'y.-', 'MarkerSize', 0.1)
end
for i = 1:size(P,1)
    plot3(stateMat(P(i,2),1),stateMat(P(i,2),2),stateMat(P(i,2),3),'b*')
    tempVar = reshape(trajMat(evalMat(P(i,1),P(i,2)),1:3,:),3,nTrajNodes)';
    plot3(tempVar(:,1),tempVar(:,2),tempVar(:,3),'b.-', 'LineWidth',10)
end
plot3(stateMat(2,1),stateMat(2,2),stateMat(2,3),'ro',...
    'MarkerFaceColor','r', 'MarkerSize', 10)
xlim(xyz_bounds(1,:))
ylim(xyz_bounds(2,:))
zlim(xyz_bounds(3,:))
axis off
axis equal

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Samples
figure('Color',[1 1 1])
hold on
try
    % try new obstacle format (axis aligned obstacles only)
    if isfield(mpinfo.obstacles, 'cuboids')
        PlotCuboidObstacles(mpinfo.obstacles.cuboids.ulVerts);
    end
catch
    % go back to old obstacle format
    if isfield(mpinfo.obstacles, 'cuboids')
        faceVertexIndices       = obstacles.cuboids.faceVertexIndices;  % Cn1
        faceVertexCoordinates   = obstacles.cuboids.vertices;           % C1
        Ncuboids = length(faceVertexIndices);
        for j = 1:Ncuboids
            cuboidFaceVertexCoordinates = faceVertexCoordinates{j};
            cuboidFaceVertexIndices     = faceVertexIndices{j};
            NfaceVertices               = length( cuboidFaceVertexIndices );
            for k = 1:NfaceVertices
                pts = sum( cuboidFaceVertexIndices(1:k) ) - cuboidFaceVertexIndices(1) + (1:1:cuboidFaceVertexIndices(k));
                patch( cuboidFaceVertexCoordinates(1,pts), cuboidFaceVertexCoordinates(2,pts), cuboidFaceVertexCoordinates(3,pts), ...
                    [0.7,0.2,0.3], 'FaceAlpha', 0.9);
            end
        end
    end
end
plot3(stateMat(3:end,1),stateMat(3:end,2),stateMat(3:end,3),'.')
xlim(xyz_bounds(1,:))
ylim(xyz_bounds(2,:))
zlim(xyz_bounds(3,:))
xlabel('x-pos')
ylabel('y-pos')
zlabel('z-pos')
axis off
axis equal

end