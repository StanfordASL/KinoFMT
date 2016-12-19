function DblIntQuadPlotCondensedMPC(mpinfo, mpc_info)

addpath([pwd, '/../'])

width = 1000;
height = 1000;

for mpciter = 1:length(mpc_info.Xstart)
    if (mpc_info.exitCond(mpciter) ~= 1)
        continue;
    end
    % Set up variables
    optPath = mpc_info.optPath{mpciter};
    parents = mpc_info.treeParents{mpciter};
    xyz_bounds = mpinfo.obstacles.obstacleSpace;
    stateMat = mpinfo.stateMat;
    trajMat = mpinfo.trajMat;
    evalMat = mpinfo.evalMat;
    obstacles = mpinfo.obstacles;
    obstacles.spheres = mpc_info.spheres{mpciter};
    nTrajNodes = mpinfo.sampling.nTrajNodes;
    fa = 1.0;


    x_0 = mpc_info.Xstart{mpciter}(1:3)';
    x_f = mpc_info.Xgoal{mpciter}(1:3)';
    x   = stateMat( optPath(2:end-1), : )';
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Trajectory and Obstacles
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    curFig = figure; hold on;
    plot3( x_0(1), x_0(2), x_0(3), '-og', 'Color', [0 0.5 0], 'Linewidth', 2, 'MarkerSize', 7 );
    plot3( x_f(1), x_f(2), x_f(3), '-or', 'Linewidth', 2, 'MarkerSize', 7 );

    % plot start state
    plot3(  x_0(1), x_0(2), x_0(3), 'g^', 'MarkerFaceColor','g', 'MarkerSize', 10);
    plot3(  [x_0(1) mpinfo.stateMat(optPath(2),1)], ...
            [x_0(2) mpinfo.stateMat(optPath(2),2)], ...
            [x_0(3) mpinfo.stateMat(optPath(2),3)], '.-b', 'Linewidth', 2);
    
    % plot others
    for i = 1:size(optPath,1)-1
        BVPnum = mpinfo.evalMat(optPath(i,1),optPath(i+1,1));
        if BVPnum > 0
            plot3(reshape(mpinfo.trajMat(BVPnum,1,:),[],1),...
                reshape(mpinfo.trajMat(BVPnum,2,:),[],1),...
                reshape(mpinfo.trajMat(BVPnum,3,:),[],1),'.-b', 'Linewidth', 2);
            plot3(reshape(mpinfo.trajMat(BVPnum,1,1),[],1),...
                reshape(mpinfo.trajMat(BVPnum,2,1),[],1),...
                reshape(mpinfo.trajMat(BVPnum,3,1),[],1),'ob', 'Linewidth', 2);
        end
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
                        [0.7,0.2,0.3], 'FaceAlpha', fa);
                end
            end
        end
    end
    if isfield(obstacles, 'spheres')
        PlotSphereObstacles(obstacles.spheres);
    end

    % Try plotting smoothed trajectory
    try
       % hardcoded params
        nPlotPoints = 100;

        % extract information
        smoother = mpinfo.smoother;
        Tdel = mpc_info.Tdel{mpciter};
        nSeg = length(Tdel);
        splineCoefs = mpc_info.splineCoefs{mpciter};
        nCoef = length(splineCoefs)/nSeg;

        for l = 1:nSeg
            baseInd = (l-1)*nCoef;
            tVec = linspace(0, Tdel(l,1), nPlotPoints);
            xPos = polyval(flipud(splineCoefs(1+baseInd:nCoef+baseInd,1)), tVec);
            yPos = polyval(flipud(splineCoefs(1+baseInd:nCoef+baseInd,2)), tVec);
            zPos = polyval(flipud(splineCoefs(1+baseInd:nCoef+baseInd,3)), tVec);
            plot3(xPos, yPos, zPos);
            hold on
        end 
    catch
        disp('failed to plot smoothed trajectory')
    end


    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    axis equal
    set(curFig, 'Position', [0 0 width height]);
    xlim(xyz_bounds(1,:))
    ylim(xyz_bounds(2,:))
    zlim(xyz_bounds(3,:))
    view(180,-90)
    camroll(180)
    hold off


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Tree
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    curFig = figure('Color',[1 1 1]);
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
                        [0.7,0.2,0.3], 'FaceAlpha', fa);
                end
            end
        end
    end
    if isfield(obstacles, 'spheres')
        PlotSphereObstacles(obstacles.spheres);
    end
%     plot3(stateMat(1,1),stateMat(1,2),stateMat(1,3),'g^',...
%         'MarkerFaceColor','g', 'MarkerSize', 10)
    % for i = 1:size(Edges,1)
    %     plot3(stateMat(Edges(i,2),1),stateMat(Edges(i,2),2),stateMat(Edges(i,2),3),'b*','MarkerSize', 0.1)
    %     tempVar = reshape(trajMat(evalMat(Edges(i,1),Edges(i,2)),1:3,:),3,nTrajNodes)';
    %     plot3(tempVar(:,1),tempVar(:,2),tempVar(:,3),'k.-', 'MarkerSize', 0.1)
    % end
    
    % Plot tree
    for i = 1:size(parents,1)
        if (parents(i,1) > 0)
            BVPnum = evalMat(parents(i,1),i);
            if BVPnum > 0
                plot3(stateMat(i,1),stateMat(i,2),stateMat(i,3),'b*','MarkerSize', 0.01)
                tempVar = reshape(trajMat(BVPnum,1:3,:),3,nTrajNodes)';
                plot3(tempVar(:,1),tempVar(:,2),tempVar(:,3),'k.-', 'MarkerSize', 0.01)
            end
        end
    end
    
    % Plot Solution
    plot3(  x_0(1), x_0(2), x_0(3), 'g^', 'MarkerFaceColor','g', 'MarkerSize', 10);
    plot3(  [x_0(1) mpinfo.stateMat(optPath(2),1)], ...
            [x_0(2) mpinfo.stateMat(optPath(2),2)], ...
            [x_0(3) mpinfo.stateMat(optPath(2),3)], 'b.-', 'LineWidth',10);
    for i = 1:size(optPath,1)-1
        BVPnum = evalMat(optPath(i,1),optPath(i+1,1));
        if BVPnum > 0
            plot3(stateMat(optPath(i+1,1),1),stateMat(optPath(i+1,1),2),stateMat(optPath(i+1,1),3),'b*')
            tempVar = reshape(trajMat(BVPnum,1:3,:),3,nTrajNodes)';
            plot3(tempVar(:,1),tempVar(:,2),tempVar(:,3),'b.-', 'LineWidth',10)
        end
    end
    plot3(stateMat(2,1),stateMat(2,2),stateMat(2,3),'ro',...
        'MarkerFaceColor','r', 'MarkerSize', 10)

    % Try plotting smoothed trajectory
    try
       % hardcoded params
        nPlotPoints = 100;

        % extract information
        Tdel = mpc_info.Tdel{mpciter};
        nSeg = length(Tdel);
        splineCoefs = mpc_info.splineCoefs{mpciter};
        nCoef = length(splineCoefs)/nSeg;

        for l = 1:nSeg
            baseInd = (l-1)*nCoef;
            tVec = linspace(0, Tdel(l,1), nPlotPoints);
            xPos = polyval(flipud(splineCoefs(1+baseInd:nCoef+baseInd,1)), tVec);
            yPos = polyval(flipud(splineCoefs(1+baseInd:nCoef+baseInd,2)), tVec);
            zPos = polyval(flipud(splineCoefs(1+baseInd:nCoef+baseInd,3)), tVec);
            plot3(xPos, yPos, zPos);
            hold on
        end 
    catch
        disp('failed to plot smoothed trajectory')
    end

    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    axis equal
    set(curFig, 'Position', [0 0 width height]);
    xlim(xyz_bounds(1,:))
    ylim(xyz_bounds(2,:))
    zlim(xyz_bounds(3,:))
    view(180,-90)
    camroll(180)
    hold off

    
end
end
