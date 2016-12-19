function DblIntQuadPlotMPC(mpinfo, mpc_info, viconLogFile, obstacleLogFile, px4Log)

for mpciter = 1:length(mpc_info.Xstart)
    if (mpc_info.exitCond(mpciter) ~= 1)
        continue;
    end
    % Set up variables
    optPath = mpc_info.optPath{mpciter};
    % Edges = mpinfo.explorationTree;
    parents = mpc_info.treeParents{mpciter};
    xyz_bounds = mpinfo.obstacles.obstacleSpace;
    stateMat = mpinfo.stateMat;
    trajMat = mpinfo.trajMat;
    evalMat = mpinfo.evalMat;
    obstacles = mpinfo.obstacles;
    obstacles.spheres = mpc_info.spheres{mpciter};
    nTrajNodes = mpinfo.sampling.nTrajNodes;
    viconValid = false;
    fa = 1.0;
    viconStartTimestep = 1;
    viconStopTimestep = NaN;

    if nargin >= 3
        try
            viconData = importdata(viconLogFile);
            viconValid = true;
        catch
            disp('Invalid vicon log file');
        end
    end

    if nargin >= 4
        try
            %obstacleData = importdata(obstacleLogFile);
            obstacleValid = true;
        catch
            disp('Invalid obstacle log file');
        end
    end

    px4Valid = false;
    if nargin >= 5
        %%%%%%%%%%%%%%%%%%
        % import px4 data
        %%%%%%%%%%%%%%%%%%
        addpath([pwd, '/../AgileQuad/']);
        if ischar(px4Log)
            % import data from bin file
            px4LogPath = px4Log;
            try
                disp('importing PX4 data...');
                px4Data = ImportPX4LogData(px4LogPath);
                px4Valid = true;
            catch
                disp('Could not read PX4 data. Exiting...');
                px4Valid=false;
            end

        elseif isstruct(px4Log)
            % existing structure has been passed in for processing
            px4Data = px4Log;
            px4Valid = true;
        else
            disp('PX4 data not recognized. Exiting...');
            px4Valid = false;
        end
    end

    x_0 = stateMat(1,:)';
    x_f = stateMat(optPath(end,1),:)';
    x   = stateMat( optPath, : )';

    figure; hold on;
    plot3( x_0(1), x_0(2), x_0(3), '-og', 'Color', [0 0.5 0], 'Linewidth', 2, 'MarkerSize', 7 );
    plot3( x_f(1), x_f(2), x_f(3), '-or', 'Linewidth', 2, 'MarkerSize', 7 );

    % plot3( x(1,:), x(2,:), x(3,:), '-b');
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

    % try plotting vicon data
    if (viconValid)
        try
            if isnan(viconStopTimestep)
                plot3(viconData.data(viconStartTimestep:end, 3), ...
                        viconData.data(viconStartTimestep:end,4), ...
                        viconData.data(viconStartTimestep:end,5), 'g', 'Linewidth', 2)
            else
                plot3(viconData.data(viconStartTimestep:viconStopTimestep, 3), ...
                    viconData.data(viconStartTimestep:viconStopTimestep,4), ...
                    viconData.data(viconStartTimestep:viconStopTimestep,5), 'g', 'Linewidth', 2)
            end
        catch
            disp('failed to plot vicon data');
        end
    end

    % try plotting px4 data

    % u_T = explorationTree.*bsxfun(@rdivide, u_star, norms(u_star,2,1));
    % quiver3( x(1,1:end-1), x(2,1:end-1), x(3,1:end-1), u_T(1,:), u_T(2,:), u_T(3,:), 0, 'Color', [0 0.5 0]);
    % xlabel('x(t)');
    % ylabel('y(t)');
    % zlabel('z(t)');
    % legend('x_{init}', 'x_{goal}', 'Motion Plan', 'Location', 'EastOutside');
    % grid on; axis equal; %view([-20 20]);

    % try plotting obstacles
    if (obstacleValid) 
        try
            fid = fopen(obstacleLogFile);
            obsLine = fgetl(fid);
            obsLine = fgetl(fid);
            while ischar(obsLine)
               obs = str2num(obsLine);
               obs(1:2) = [];
               while length(obs) >= 3
                  plot3(obs(1), obs(2), obs(3), 'b*');
                  obs(1:3) = [];
               end
               obsLine = fgetl(fid);
            end
            fclose(fid);
        catch
            fclose(fid);
            disp('failed to plot obstacle log');
        end
    end

    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    xlim(xyz_bounds(1,:))
    ylim(xyz_bounds(2,:))
    zlim(xyz_bounds(3,:))
    view(260,-50)
    % view(151,-10);
    camroll(180);

    hold off



    figure; hold on;
    plot3( x_0(1), x_0(2), x_0(3), '-og', 'Color', [0 0.5 0], 'Linewidth', 2, 'MarkerSize', 7 );
    plot3( x_f(1), x_f(2), x_f(3), '-or', 'Linewidth', 2, 'MarkerSize', 7 );

    for i = 1:size(optPath,1)-1
        BVPnum = mpinfo.evalMat(optPath(i,1),optPath(i+1,1));
        if BVPnum > 0
            plot3(reshape(mpinfo.trajMat(BVPnum,1,:),[],1),...
                reshape(mpinfo.trajMat(BVPnum,2,:),[],1),...
                reshape(mpinfo.trajMat(BVPnum,3,:),[],1),'.-b');
            plot3(reshape(mpinfo.trajMat(BVPnum,1,1),[],1),...
                reshape(mpinfo.trajMat(BVPnum,2,1),[],1),...
                reshape(mpinfo.trajMat(BVPnum,3,1),[],1),'ob');
        end
    end

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
    xlim(xyz_bounds(1,:))
    ylim(xyz_bounds(2,:))
    zlim(xyz_bounds(3,:))
    view(151,-10);
    camroll(180);
    hold off


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Trajectory (Multi View)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
                        [0.7,0.2,0.3], 'FaceAlpha', fa);
                end
            end
        end
    end
    if isfield(obstacles, 'spheres')
        PlotSphereObstacles(obstacles.spheres);
    end
    plot3(stateMat(1,1),stateMat(1,2),stateMat(1,3),'g^',...
        'MarkerFaceColor','g', 'MarkerSize', 10)
    for i = 1:size(optPath,1)-1
        BVPnum = evalMat(optPath(i,1),optPath(i+1,1));
        if BVPnum > 0
            plot3(stateMat(optPath(i+1,1),1),stateMat(optPath(i+1,1),2),stateMat(optPath(i+1,1),3),'b*')
            tempVar = reshape(trajMat(BVPnum,1:3,:),3,nTrajNodes)';
            plot3(tempVar(:,1),tempVar(:,2),tempVar(:,3),'k.-')
        end
    end
    plot3(stateMat(2,1),stateMat(2,2),stateMat(2,3),'ro',...
        'MarkerFaceColor','r', 'MarkerSize', 10)
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
    xlim(xyz_bounds(1,:))
    ylim(xyz_bounds(2,:))
    zlim(xyz_bounds(3,:))
    view(180,-90)
    camroll(180)
    axis equal
    hold off

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
                        [0.7,0.2,0.3], 'FaceAlpha', fa);
                end
            end
        end
    end
    if isfield(obstacles, 'spheres')
        PlotSphereObstacles(obstacles.spheres);
    end
    plot3(stateMat(1,1),stateMat(1,2),stateMat(1,3),'g^',...
        'MarkerFaceColor','g', 'MarkerSize', 10)
    for i = 1:size(optPath,1)-1
        BVPnum = evalMat(optPath(i,1),optPath(i+1,1));
        if BVPnum > 0
            plot3(stateMat(optPath(i+1,1),1),stateMat(optPath(i+1,1),2),stateMat(optPath(i+1,1),3),'b*')
            tempVar = reshape(trajMat(BVPnum,1:3,:),3,nTrajNodes)';
            plot3(tempVar(:,1),tempVar(:,2),tempVar(:,3),'k.-')
        end
    end
    plot3(stateMat(2,1),stateMat(2,2),stateMat(2,3),'ro',...
        'MarkerFaceColor','r', 'MarkerSize', 10)
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
    xlim(xyz_bounds(1,:))
    ylim(xyz_bounds(2,:))
    zlim(xyz_bounds(3,:))
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    view(180,0)
    camroll(180)
    axis equal
    hold off

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
                        [0.7,0.2,0.3], 'FaceAlpha', fa);
                end
            end
        end
    end
    if isfield(obstacles, 'spheres')
        PlotSphereObstacles(obstacles.spheres);
    end
    plot3(stateMat(1,1),stateMat(1,2),stateMat(1,3),'g^',...
        'MarkerFaceColor','g', 'MarkerSize', 10)
    for i = 1:size(optPath,1)-1
        BVPnum = evalMat(optPath(i,1),optPath(i+1,1));
        if BVPnum > 0
            plot3(stateMat(optPath(i+1,1),1),stateMat(optPath(i+1,1),2),stateMat(optPath(i+1,1),3),'b*')
            tempVar = reshape(trajMat(BVPnum,1:3,:),3,nTrajNodes)';
            plot3(tempVar(:,1),tempVar(:,2),tempVar(:,3),'k.-')
        end
    end
    plot3(stateMat(2,1),stateMat(2,2),stateMat(2,3),'ro',...
        'MarkerFaceColor','r', 'MarkerSize', 10)
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
    xlim(xyz_bounds(1,:))
    ylim(xyz_bounds(2,:))
    zlim(xyz_bounds(3,:))
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    view(151,-30)
    camroll(180);
    axis equal
    hold off

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Tree
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
                        [0.7,0.2,0.3], 'FaceAlpha', fa);
                end
            end
        end
    end
    if isfield(obstacles, 'spheres')
        PlotSphereObstacles(obstacles.spheres);
    end
    plot3(stateMat(1,1),stateMat(1,2),stateMat(1,3),'g^',...
        'MarkerFaceColor','g', 'MarkerSize', 10)
    % for i = 1:size(Edges,1)
    %     plot3(stateMat(Edges(i,2),1),stateMat(Edges(i,2),2),stateMat(Edges(i,2),3),'b*','MarkerSize', 0.1)
    %     tempVar = reshape(trajMat(evalMat(Edges(i,1),Edges(i,2)),1:3,:),3,nTrajNodes)';
    %     plot3(tempVar(:,1),tempVar(:,2),tempVar(:,3),'k.-', 'MarkerSize', 0.1)
    % end
    for i = 1:size(parents,1)
        if (parents(i,1) > 0)
            BVPnum = evalMat(parents(i,1),i);
            if BVPnum > 0
                plot3(stateMat(i,1),stateMat(i,2),stateMat(i,3),'b*','MarkerSize', 0.1)
                tempVar = reshape(trajMat(BVPnum,1:3,:),3,nTrajNodes)';
                plot3(tempVar(:,1),tempVar(:,2),tempVar(:,3),'k.-', 'MarkerSize', 0.1)
            end
        end
    end
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
    xlim(xyz_bounds(1,:))
    ylim(xyz_bounds(2,:))
    zlim(xyz_bounds(3,:))
    view(180,-90)
    camroll(180)
    axis equal
    hold off

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
                        [0.7,0.2,0.3], 'FaceAlpha', fa);
                end
            end
        end
    end
    if isfield(obstacles, 'spheres')
        PlotSphereObstacles(obstacles.spheres);
    end
    plot3(stateMat(3:end,1),stateMat(3:end,2),stateMat(3:end,3),'.')
    xlim(xyz_bounds(1,:))
    ylim(xyz_bounds(2,:))
    zlim(xyz_bounds(3,:))
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    view(180,-90)
    camroll(180)
    axis equal
    hold off

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Time History
    figure
    hold on
    % Try plotting smoothed trajectory
    try
        tbase = 0;
        for l = 1:nSeg
            baseInd = (l-1)*nCoef;
            dtVec = linspace(0, Tdel(l,1), nPlotPoints);
            cumtVec = dtVec + tbase;
            xPos = polyval(flipud(splineCoefs(1+baseInd:nCoef+baseInd,1)), dtVec);
            yPos = polyval(flipud(splineCoefs(1+baseInd:nCoef+baseInd,2)), dtVec);
            zPos = polyval(flipud(splineCoefs(1+baseInd:nCoef+baseInd,3)), dtVec);
            plot(cumtVec, xPos, '-b');
            plot(cumtVec, yPos, '-g');
            plot(cumtVec, zPos, '-r');
            tbase = cumtVec(end);
            hold on
        end 
    catch
        disp('failed to plot smoothed trajectory')
    end
    xlabel('time [sec]')
    ylabel('position [m]')

end
end
