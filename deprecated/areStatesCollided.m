

function isCollided = areStatesCollided(states,obstacles,spaceTilingResolution,stateSpaceBounds,checkPaths)

persistent persistentDimension % dimension of the state space
persistent persistentBounds % bounds on the state space [[lower bounds] | [upper bounds]]
persistent persistentTileSizes % vector with the size of each tile in a given dimension
persistent persistentTile2ObstacleSet % cell array of vectors of obstacles


% Check that obstacles are present
if ~isfield(obstacles, 'cuboids')
    isCollided = false;
    return;
end

% Set up space tiling. Map each obstacle into a tile.
if isempty(persistentTile2ObstacleSet)
    persistentDimension = length(states(1,:));
    persistentBounds = stateSpaceBounds;
    persistentTileSizes = (stateSpaceBounds(:,2)-stateSpaceBounds(:,1))'/spaceTilingResolution;
%     keyboard
    persistentTile2ObstacleSet = cell(spaceTilingResolution, spaceTilingResolution, spaceTilingResolution);
    for p=1:length(obstacles.cuboids.vertices)
        indexes = mapObstacle2Tile(obstacles.cuboids.vertices{p});
        if indexes
            for n=1:length(indexes(:,1))
                persistentTile2ObstacleSet{indexes(n,1),indexes(n,2),indexes(n,3)}(1,end+1) = p;
            end
        end
    end
end

isCollided = areStatesOutOfBounds(states, stateSpaceBounds(:,1), stateSpaceBounds(:,2));
if ~checkPaths % Check if individual states intersect any obstacles
    if ~isCollided
        for p=1:length(states(:,1))
            stateIndex = mapPoint2Tile(states(p,:));
            obstacleIndexes = persistentTile2ObstacleSet{stateIndex(1,1),stateIndex(1,2),stateIndex(1,3)};
            nearbyObstacleFaces = cell(size(obstacleIndexes));
            for n = 1:length(obstacleIndexes)
                nearbyObstacleFaces{n} = obstacles.cuboids.faces{obstacleIndexes(n)};
            end
            isCollided = areStatesObstructed(states(p,:)', nearbyObstacleFaces);
            if isCollided
                break;
            end
        end
    end
else % Check if the straight lines between consecutive states intersect any obstacles
    if ~isCollided
        for p=1:length(states(:,1))-1
            % get the obstacles from either end of the path segment
            stateIndex = mapPoint2Tile(states(p,:));
            obstacleIndexes1 = persistentTile2ObstacleSet{stateIndex(1,1),stateIndex(1,2),stateIndex(1,3)};
            stateIndex = mapPoint2Tile(states(p+1,:));
            obstacleIndexes2 = persistentTile2ObstacleSet{stateIndex(1,1),stateIndex(1,2),stateIndex(1,3)};
            obstacleIndexes = unique([obstacleIndexes1, obstacleIndexes2]);
            nearbyObstacleFaces = cell(size(obstacleIndexes));
            for n = 1:length(obstacleIndexes)
                nearbyObstacleFaces{n} = obstacles.cuboids.faces{obstacleIndexes(n)};
            end
            
            isCollided = isPathObstructed(states(p:p+1,:)', nearbyObstacleFaces);
            if isCollided
                break;
            end
        end
    end
end




    function indexes = mapPoint2Tile(point)
        indexes = ceil(point./persistentTileSizes) - ceil(persistentBounds(:,1)'./persistentTileSizes);
        indexes = max(indexes, ones(size(indexes)));
        
        edgeCases = find(mod(point, persistentTileSizes) <= eps);
        if edgeCases
            incrementVector = zeros(size(indexes));
            incrementVector(edgeCases) = 1;
            indexes = [indexes; min(indexes+incrementVector, ...
                spaceTilingResolution*ones(size(indexes)))];
        end
    end

    function indexes = mapObstacle2Tile(vertices)
        
        upperVertex = max(vertices,[],2);
        lowerVertex = min(vertices,[],2);
        
        upperTile = max(mapPoint2Tile(upperVertex'),[],1);
        lowerTile = min(mapPoint2Tile(lowerVertex'),[],1);
        
        indexes = [];
        if persistentDimension == 3
            for i=lowerTile(1):upperTile(1)
                for j = lowerTile(2):upperTile(2)
                    for k = lowerTile(3):upperTile(3)
                        if ~ismember([i,j,k], indexes, 'rows','legacy')
                            indexes(end+1,:) = [i,j,k];
                        end
                    end
                end
            end
        elseif persistentDimension == 2
            for i=lowerTile(1):upperTile(1)
                for j = lowerTile(2):upperTile(2)
                    if ~ismember([i,j], indexes, 'rows','legacy')
                        indexes(end+1,:) = [i,j];
                    end
                end
            end
        else
            error('ERROR: State dimension is neither 2 nor 3. Collision checker cannot handle it.');
        end
    end

    function bool = areStatesOutOfBounds( X, LB, UB )
        % X     = [ x(t0) | x(t1) | ... ]
        % LB    = [ x_1min, x_2min, ... ]^T
        % UB    = [ x_1max, x_2max, ... ]^T
        %
        % Check configuration space boundaries
        bool = false;
        if ~isempty( find( bsxfun(@minus,X,UB') > 0, 1) ) || ...
                ~isempty( find( bsxfun(@minus,LB',X) > 0, 1) )
            bool = true;
        end
    end

    function bool = areStatesObstructed( positions, obstacleFaces )
        % positions = [ r(t0) | r(t1) | ... ]
        % obstacles:
        %   cuboids.faces = { [a1, b1, c1, d1]^T | ... | [a6, b6, c6, d6]^T for cuboid 1,
        %                     [a1, b1, c1, d1]^T | ... | [a6, b6, c6, d6]^T for cuboid 2,
        %                     ... }
        %   cuboids.vertices
        
        bool        = false;
        Nnodes      = size(positions,2);
        Ncuboids    = length(obstacleFaces);
        
        for k1 = 1:Ncuboids
            % For each cuboid, evaluate the value of the algebraic primitives of its 6 faces
            Nfaces = size(obstacleFaces{k1},2);
            f = zeros( Nfaces, Nnodes );
            
            for k2 = 1:Nfaces
                % For each cuboid face, compute "f = [a b]^T*positions + d" (2D case)
                % or "f = [a b c]^T * x + d" (3D case)
                f(k2,:) = obstacleFaces{k1}(1:(end-1),k2)'*positions + obstacleFaces{k1}(end,k2);
            end
            
            %
            if max(f,[],1) <= 0
                bool = true;
                break;
            end
        end
    end

    function bool = isPathObstructed(position_traj, obstaclFaces)
        % Check for intersection of each segment of the trajectory with obstacles
        t = []; k2 = 1;
        Nnodes      = size(position_traj,2);
        Ncuboids    = length(obstaclFaces);
        
        while isempty(t) && k2 <= Nnodes-1
            k1 = 1;
            while isempty(t) && k1 <= Ncuboids
                t = convexpoly_line_intersect( obstaclFaces{k1}, position_traj(:,k2), position_traj(:,k2+1) );
                k1 = k1 + 1;
            end
            k2 = k2 + 1;
        end
        
        bool = true;
        if isempty(t)
            bool = false;
        end
    end

    function t = convexpoly_line_intersect(f,r1,r2)
        % f = [[a1 b1 c1 d1]', [a2 b2 c2 d2]', ...
        % r1 = beginning endpoint of line segment
        % r2 = final endpoint of line segment
        tFirst = 0;
        tLast  = 1;
        k = 1;
        N = size(f,2);
        
        if r1 == r2
            g = zeros(1,N);
            for m = 1:N
                g(m) = f(1:3,m)'*r1 + f(4,m);
            end
            if g < 0
                tLast = 0;
            else
                tLast = -1;
            end
        end
        
        while tFirst < tLast && k <= N
            dist = -f(4,k) - f(1:3,k)'*r1;
            denom = f(1:3,k)'*(r2-r1);
            if denom == 0
                if dist > 0
                    tLast = -1;
                    break
                else
                    k = k + 1;
                    continue
                end
            end
            
            tk = dist/denom;
            if denom < 0
                tFirst = max(tk,tFirst);
            else
                tLast = min(tk,tLast);
            end
            
            k = k + 1;
        end
        
        if tLast < tFirst
            t = [];
        else
            t = [tFirst, tLast];
        end
    end

end