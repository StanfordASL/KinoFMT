
function DeepSpace_PlotTrajectory( pathIndices, stateMat, trajMat, obstacles )
    x_0 = stateMat(1,:)';
    x_f = stateMat(2,:)';
    x   = stateMat( pathIndices, : )';
    faceVertexIndices       = obstacles.cuboids.faceVertexIndices;  % Cn1
    faceVertexCoordinates   = obstacles.cuboids.vertices;           % C1
    
    
    figure; hold on;
    plot3( x_0(1), x_0(2), x_0(3), '-og', 'Color', [0 0.5 0], 'Linewidth', 2, 'MarkerSize', 7 );
    plot3( x_f(1), x_f(2), x_f(3), '-or', 'Linewidth', 2, 'MarkerSize', 7 );
    plot3( x(1,:), x(2,:), x(3,:), '-b');
    
    Ncuboids = length(faceVertexIndices);
    for j = 1:Ncuboids
        cuboidFaceVertexCoordinates = faceVertexCoordinates{j};
        cuboidFaceVertexIndices     = faceVertexIndices{j};
        NfaceVertices               = length( cuboidFaceVertexIndices );
        for k = 1:NfaceVertices
            pts = sum( cuboidFaceVertexIndices(1:k) ) - cuboidFaceVertexIndices(1) + (1:1:cuboidFaceVertexIndices(k));
            patch( cuboidFaceVertexCoordinates(1,pts), cuboidFaceVertexCoordinates(2,pts), cuboidFaceVertexCoordinates(3,pts), ...
                [0.1 0.1 0.8], 'FaceAlpha', 0.5);
        end
    end

    u_T = H.*bsxfun(@rdivide, u_star, norms(u_star,2,1));
    quiver3( x(1,1:end-1), x(2,1:end-1), x(3,1:end-1), u_T(1,:), u_T(2,:), u_T(3,:), 0, 'Color', [0 0.5 0]);
    xlabel('x(t)');
    ylabel('y(t)');
    zlabel('z(t)');
    legend('x_{init}', 'x_{goal}', 'Motion Plan', 'Location', 'EastOutside');
    grid on; axis equal; %view([-20 20]);
end