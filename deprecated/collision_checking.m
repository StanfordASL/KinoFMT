

% function is_violation   = collision_checking( positions, bounds, obstacles )
%     % Check upper and lower state space boundaries
%     is_violation    = areStatesOutOfBounds( x_i, bounds(:,2), bounds(:,1) );
%     
%     % Check for collisions with fixed obstacles
%     if ~is_violation
%         is_violation = areStatesObstructed( positions, obstacles );
%     end
%     
%     % If a vertex resides in C_obs (bounds + cuboid debris), the
%     % constraints have been violated and exit.  Otherwise, check if any of
%     % the line segments connecting vertices intersect the debris.
%     if ~is_violation
%         is_violation = isPathObstructed( positions, obstacles );
%     end
% end
% 
% % function bool   = collision_checking(x_i, t_i, bounds, F, G0, CM_G0, V_G0, t_0, parameters )
% %     [n,m]   = size(x_i);
% %     
% %     % Check upper and lower state space boundaries
% %     G_i(1:n,:)      = x_i - repmat( bounds(:,1),1,m );
% %     G_i(n+(1:n),:)  = repmat( bounds(:,2),1,m ) - x_i;
% %     
% %     % Check for collisions with fixed obstacles
% %     for k1 = 1:length(F)
% %         f = zeros( size(F{k1},2),m );
% %         for k2 = 1:size(F{k1},2)
% %             f(k2,:) = F{k1}(1:3,k2)'*x_i(1:3,:) + F{k1}(4,k2);
% %         end
% %         G_i(2*n+k1,:) = -max(f,[],1);
% %     end
% %     
% %     % If a vertex resides in C_obs (bounds + cuboid debris), the
% %     % constraints have been violated and exit.  Otherwise, check if any of
% %     % the line segments connecting vertices intersect the debris.
% %     if ~isempty(find(G_i > 0, 1))
% %         bool = 1;
% %     else
% %         % Propagate the moving debris up to t_i(1)
% %         [x_G, G] = obstacle_propagate( [CM_G0; V_G0], G0, t_0, t_i(1), parameters );
% %         
% %         % Check for intersection of each segment of the trajectory with the
% %         % fixed debris (F) and the moving debris (G)
% %         t = []; s = []; k2 = 1;
% %         while isempty(t) && isempty(s) && k2 <= size(x_i,2)-1
% %             
% %             k1 = 1;
% %             while isempty(t) && k1 <= length(F)
% %                 t = convexpoly_line_intersect( F{k1}, x_i(1:3,k2), x_i(1:3,k2+1) );
% %                 k1 = k1 + 1;
% %             end
% %             
% %             k1 = 1;
% %             while isempty(t) && isempty(s) && k1 <= length(G)
% %                 s = convexpoly_line_intersect( G{k1}, x_i(1:3,k2), x_i(1:3,k2+1) );
% %                 k1 = k1 + 1;
% %             end
% %             
% %             % Propagate moving obstacles to the next time
% %             [x_G, G] = obstacle_propagate( x_G, G, t_i(k2), t_i(k2+1), parameters );
% %             k2 = k2 + 1;
% %         end
% %         
% %         if isempty(t) && isempty(s)
% %             bool = 0;
% %         else
% %             bool = 1;
% %         end
% %     end
% % end
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
% function bool = areStatesObstructed( positions, obstacles )
%     % positions = [ r(t0) | r(t1) | ... ]
%     % obstacles:
%     %   cuboids.faces = { [a1, b1, c1, d1]^T | ... | [a6, b6, c6, d6]^T for cuboid 1, 
%     %                     [a1, b1, c1, d1]^T | ... | [a6, b6, c6, d6]^T for cuboid 2, 
%     %                     ... }
%     %   cuboids.vertices
% 
%     bool        = false;
%     Nnodes      = size(positions,2);
%     Ncuboids    = length(obstacles.cuboids.faces);
%     
%     for k1 = 1:Ncuboids
%         % For each cuboid, evaluate the value of the algebraic primitives of its 6 faces
%         Nfaces = size(obstacles.cuboids.faces{k1},2);
%         f = zeros( Nfaces, Nnodes );
%         
%         for k2 = 1:Nfaces
%             % For each cuboid face, compute "f = [a b]^T*positions + d" (2D case)
%             % or "f = [a b c]^T * x + d" (3D case)
%             f(k2,:) = obstacles.cuboids.faces{k1}(1:(end-1),k2)'*positions + obstacles.cuboids.faces{k1}(end,k2);
%         end
%         
%         % 
%         if max(f,[],1) < 0
%             bool = true;
%             break;
%         end
%     end
% end
% 
% function bool = isPathObstructed( position_traj, obstacles )
%     % Check for intersection of each segment of the trajectory with obstacles
%     t = []; k2 = 1;
%     Nnodes      = size(position_traj,2);
%     Ncuboids    = length(obstacles.cuboids.faces);
%     
%     while isempty(t) && k2 <= Nnodes-1
%         k1 = 1;
%         while isempty(t) && k1 <= Ncuboids
%             t = convexpoly_line_intersect( obstacles.cuboids.faces{k1}, position_traj(:,k2), position_traj(:,k2+1) );
%             k1 = k1 + 1;
%         end
%         k2 = k2 + 1;
%     end
% 
%     bool = true;
%     if isempty(t)
%         bool = false;
%     end
% end
