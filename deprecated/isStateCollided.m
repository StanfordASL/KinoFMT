

% function isCollided = isStateCollided(state,obstacles,spaceTilingResolution,stateSpaceBounds)
% 
% persistent persistentTile2ObstacleSet % cell array of vectors of obstacles
% persistent persistentTileSizes
% persistent persistentBounds
% persistent persistentObstacleSet
% persistent persistentDimension
% 
% if isempty(persistentTile2ObstacleSet)
% %     keyboard
%     persistentObstacleSet = obstacles;
%     persistentDimension = length(state);
%     persistentBounds = stateSpaceBounds;
%     persistentTileSizes = (stateSpaceBounds(:,1)-stateSpaceBounds(:,2))/spaceTilingResolution;
%     
%     persistentTile2ObstacleSet = cell(spaceTilingResolution, spaceTilingResolution, spaceTilingResolution);
%     for m=1:length(obstacles.cuboids.vertices)
%         indexes = mapObstacle2Tile(obstacles.cuboids.vertices{m}.coords);
%         if indexes
%             for n=1:length(indexes(:,1))
%                 persistentTile2ObstacleSet{indexes(n,1),indexes(n,2),indexes(n,3)}{end+1} = obstacles.cuboids.faces{m};
%             end
%         end
%     end
% end
% 
% stateIndex = mapPoint2Tile(state);
% isCollided = areStatesOutOfBounds(state, stateSpaceBounds(:,2), stateSpaceBounds(:,1));
% if ~isCollided
%     nearbyObstacleFaces = persistentTile2ObstacleSet{stateIndex(1,1),stateIndex(2,1),stateIndex(3,1)};
%     isCollided = areStatesObstructed(state, nearbyObstacleFaces);
% end
% 
%     function indexes = mapPoint2Tile(point)
%         indexes = ceil(point./persistentTileSizes) - ceil(persistentBounds(:,2)./persistentTileSizes);
%         indexes = max(indexes, ones(size(indexes)));
%         
%         edgeCases = find(point./persistentTileSizes <= eps);
%         if edgeCases
%             incrementVector = zeros(size(indexes));
%             incrementVector(edgeCases) = 1;
%             indexes = [indexes, min(indexes+incrementVector, ...
%                         spaceTilingResolution*ones(size(indexes)))];
%         end
%     end
% 
%     function indexes = mapObstacle2Tile(vertices)
%         
%         upperVertex = max(vertices,[],2);
%         lowerVertex = min(vertices,[],2);
%         
%         upperTile = max(mapPoint2Tile(upperVertex),[],2);
%         lowerTile = min(mapPoint2Tile(lowerVertex),[],2);
%         
%         indexes = [];
%         if persistentDimension == 3
%             for i=lowerTile(1):upperTile(1)
%                 for j = lowerTile(2):upperTile(2)
%                     for k = lowerTile(3):upperTile(3)
%                         if isempty(indexes) || ~ismember([i,j,k], indexes, 'rows')
%                             indexes(end+1,:) = [i,j,k];
%                         end
%                     end
%                 end
%             end     
%         elseif persistentDimension == 2
%             for i=lowerTile(1):upperTile(1)
%                 for j = lowerTile(2):upperTile(2)
%                     if isempty(indexes) || ~ismember([i,j], indexes, 'rows')
%                         indexes(end+1,:) = [i,j];
%                     end
%                 end
%             end                   
%         else
%             error('ERROR: State dimension is neither 2 nor 3. Collision checker cannot handle it.');
%         end
%     end
% 
% function bool = areStatesOutOfBounds( X, LB, UB )
%     % X     = [ x(t0) | x(t1) | ... ]
%     % LB    = [ x_1min, x_2min, ... ]^T
%     % UB    = [ x_1max, x_2max, ... ]^T
%     %
%     % Check configuration space boundaries
%     bool = false;
%     if ~isempty( find( bsxfun(@minus,X,UB) > 0, 1) ) || ...
%             ~isempty( find( bsxfun(@minus,LB,X) > 0, 1) )
%         bool = true;
%     end
% end
% 
% function bool = areStatesObstructed( positions, obstacleFaces )
%     % positions = [ r(t0) | r(t1) | ... ]
%     % obstacles:
%     %   cuboids.faces = { [a1, b1, c1, d1]^T | ... | [a6, b6, c6, d6]^T for cuboid 1, 
%     %                     [a1, b1, c1, d1]^T | ... | [a6, b6, c6, d6]^T for cuboid 2, 
%     %                     ... }
%     %   cuboids.vertices
% 
%     bool        = false;
%     Nnodes      = size(positions,2);
%     Ncuboids    = length(obstacleFaces);
%     
%     for k1 = 1:Ncuboids
%         % For each cuboid, evaluate the value of the algebraic primitives of its 6 faces
%         Nfaces = size(obstacleFaces{k1},2);
%         f = zeros( Nfaces, Nnodes );
%         
%         for k2 = 1:Nfaces
%             % For each cuboid face, compute "f = [a b]^T*positions + d" (2D case)
%             % or "f = [a b c]^T * x + d" (3D case)
%             f(k2,:) = obstacleFaces{k1}(1:(end-1),k2)'*positions + obstacleFaces{k1}(end,k2);
%         end
%         
%         % 
%         if max(f,[],1) <= 0
%             bool = true;
%             break;
%         end
%     end
% end
%  
% end