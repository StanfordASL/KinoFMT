
% S = pre-computed sample set (n x max_iter)
% SAMPLE      = @(iter) S(:,iter);
%
% extents     = abs( max(S,[],2) - min(S,[],2) );     % Estimate of sample space extents along each dimension
% goal_radius = 0.005*norm( extents );                % Radius of ball used to assess convergence for probabilistic trees
%
% function bool = termination_condition( t_f, x_f, x_goal, parameters, extents, goal_radius )
%     bool = ( norm( (x_f - x_goal)./extents ) <= goal_radius );
%     return bool;
% end
%
% TERMINATION = @(t_f, x_f, x_goal, parameters) termination_condition( t_f, x_f, x_goal, parameters, extents, goal_radius );

function [x_star, u_star, t_star, J_star, feasibility, varargout] = RRTstar( t_0, x_0, x_f, ...
    SAMPLE, NN, STEER, PROPAGATE, CCHECK, COST, TERMINATION, ...
    Nrewire_neighbors_max, soln, max_iter, extents, treeweights_search_radius, ...
    plotparameters, parameters)

    % RRTSTAR   Runs the RRT* algorithm
    %
    % INPUTS:
	%   t_0                                 The initial time
    %   x_0                         [n x 1] Initial state
    %   x_f                         [n x 1] Goal state
    %   x_sample                              	= SAMPLE( iter )
    %   [nodes_to_xs, ustar_to_xs, Jstar_to_xs] = NN( x_sample, tree, STEER, parameters)
    %                [ustar_to_xf, Jstar_to_xf] = STEER( x_0, x_f, t_0, parameters)
    %                           [x(t), u(t), t] = PROPAGATE( x_0, ustar, t_0, parameters )
    %                                      bool = CCHECK( x(t), u(t), t, parameters )   (returns true on a violation, false if safe)
    %                                      cost = COST( u )
    %                                      bool = TERMINATION( t_f, x_f, x_goal, parameters)
    %   Nrewire_neighbors_max       Maximum number of neighbors to rewire
    %   soln                        'feasible', 'suboptimal'
    %   max_iter                    maximum allowed number of iterations
    %   extents                     Estimate of sample space extents along each dimension
    %   treeweights_search_radius 	Radius of the ball used to compute weights for probabilistic trees
    %
    %   plotparameters.Cn                           	Indices of coordinates
    %   plotparameters.C                              	Coordinates of fixed obstacles
    %   plotparameters.states = [i1 i2 i3]            	The three indices of a state x that you want to plot to debug tree construction
    %   plotparameters.labels = {xlabel,ylabel,zlabel} 	Three strings corresponding to plotted states
    %
    %   parameters                  Structure containing additional parameters that might be useful for NN, STEER, PROPAGATE, etc
    %
    % TREE STRUCTURE:
    %     tree.nodes                = NaN(1, Nnodes_max);      	% Node indices
    %     tree.x                    = NaN(n, Nnodes_max);      	% Tree node states
    %     tree.parents              = NaN(1, Nnodes_max);      	% Node parents
    %     tree.t                    = NaN(1, Nnodes_max);      	% Time of arrival at a node
    %     tree.Jstar_from_x0        = NaN(1, Nnodes_max);     	% Costs-to-come from the initial state
    %     tree.Jstar_to_xf          = NaN(1, Nnodes_max);     	% Costs-to-go to the goal state
    %     tree.ustar_from_parent    = cell(1, Nnodes_max);     	% Control trajectories from each node's parent
    %     tree.ustar_to_xf          = cell(1, Nnodes_max);     	% Control trajectories to the goal state
    %     tree.plothandles          = NaN(2, Nnodes_max);      	% Handles to plot edges
    %     tree.weights              = NaN(1, Nedges_max);      	% Node weight representing the probability of finding (Euclidean distance) nearest neighbors
    %     tree.node_star         	= NaN;                      % Node index corresponding to the optimal path endpoint
    %     tree.connected_vertices 	= NaN;                  	% List of node indices that correspond to edge-connected nodes
    %
    % NOTES:
    % - Plot colors:    magenta = attempted (failed) direct trajectory from x_0 to x_f
    %                   red     = unsafe trajectory (edge) between tree vertex and candidate milestone
    %                   blue    = safe trajectory between vertex and milestone (milestone added as new vertex, trajectory added as new edge)
    %                   cyan    = safe rewired trajectory
    % - Currently aborts calculations (and returns results) if a single
    %   iteration takes longer than 'max_time'.
    % - Assigns weights to milestones according to the number of neighbors in
    %   a ball of radius defined as 0.05*Normed distance between x_0 and x_f
    % - Assumes controls generated (u_star) correspond to the times generated
    %   by PROPAGATE by a ZOH (i.e. for N times, the first 1:N-1 values).
    % - Assumes N-1 is at least 2, and that controls from STEER are stored in
    %   a single column vector, like from FMINCON (U vectors from PROPAGATE need 
    %   not be, so long as CCHECK and COST can accommodate U in that form).
    
    % Additional inputs
    plot_tree                      	= false;
    add_random_intermediate_nodes 	= true;
    Nrandom_intermediate_nodes    	= 1;
    
    % Beginning of function
    TIMER0    	= tic;                          % Counter for total RRT running time
    max_time    = 3*60;                     	% Maximum time allowed for one iteration [s] (otherwise exits the while loop and skips ahead)
    n           = size(x_0,1);                	% Number of states
    
    % Reset certain inputs
    if n < 3
        plot_tree = false;                      % Currently RRTstar is configured to plot state vectors with 3 or more states
    end
    
    % Initialize timekeeping variables and cost as a function of iteration
    T_RUNTIMES 	= zeros(1,max_iter);   J_STAR    = zeros(1,max_iter);
    T_SAMPLE    = zeros(1,max_iter);   T_CCHECK  = zeros(1,max_iter);       T_STEER  = zeros(1,max_iter);       T_PROP  = zeros(1,max_iter);
    T_NN        = zeros(1,max_iter);   T_CCHECK2 = zeros(1,max_iter);       T_STEER2 = zeros(1,max_iter);       T_PROP2 = zeros(1,max_iter);
    T_ADDNODES  = zeros(1,max_iter);   T_REWIRE  = zeros(1,max_iter);       N_EVALS  = zeros(11,max_iter);
    
    %% Initialize the tree
    if add_random_intermediate_milestone
        Nnew_nodes_max_per_iter = (1+Nrandom_intermediate_nodes);
        Nnodes_max  = Nnew_nodes_max_per_iter*max_iter+1;
        Nedges_max  = Nnew_nodes_max_per_iter*max_iter;
    else
        Nnodes_max  = max_iter+1;
        Nedges_max  = max_iter;
    end

    tree.nodes                  = NaN(1, Nnodes_max);      	% Node indices
    tree.x                      = NaN(n, Nnodes_max);      	% Tree node states
    tree.parents                = NaN(1, Nnodes_max);      	% Node parents
    tree.t                      = NaN(1, Nnodes_max);      	% Time of arrival at a node
    tree.Jstar_from_x0       	= NaN(1, Nnodes_max);     	% Costs-to-come from the initial state
    tree.Jstar_to_xf            = NaN(1, Nnodes_max);     	% Costs-to-go to the goal state
    tree.ustar_from_parent      = cell(1, Nnodes_max);     	% Control trajectories from each node's parent
    tree.ustar_to_xf            = cell(1, Nnodes_max);     	% Control trajectories to the goal state
    tree.plothandles            = NaN(2, Nnodes_max);      	% Handles to plot edges
    tree.weights                = NaN(1, Nedges_max);      	% Node weight representing the probability of finding (Euclidean distance) nearest neighbors
    tree.node_star              = NaN;                   	% Node index corresponding to the optimal path endpoint
    tree.connected_vertices 	= NaN;                  	% List of node indices that correspond to edge-connected nodes
    
    % Add the initial state
    tree.nodes(1)               = 1;        
    tree.x(:,1)                 = x_0;      
    tree.parents(1)             = 0;
    tree.t(1)                   = t_0;
    tree.Jstar_from_x0(1)       = 0;
    tree.weights(1)             = 1;
    tree.node_star              = 1;
    tree.connected_vertices     = 1;
    
    % Create figure to visualize the progress of the RRT*
    if plot_tree
        scrsz   = get(0,'ScreenSize');
        a       = [0.65, 0.75];
        figure('OuterPosition', [(0.5-a(1)/2)*scrsz(3), (1-a(2))*scrsz(4), a(1)*scrsz(3), a(2)*scrsz(4)]); hold on;
        tree.plothandles(1,1) = ...
        plot3( tree.x(plotparameters.states(1),1), tree.x(plotparameters.states(2),1), tree.x(plotparameters.states(3),1), '-og', 'Color', [0 0.5 0] );
        plot3( x_f(plotparameters.states(1),1), x_f(plotparameters.states(2),1), x_f(plotparameters.states(3),1), '-or' );
        for k1 = 1:length(plotparameters.Cn)
            for k2 = 1:length(plotparameters.Cn{k1})
                pts = sum(plotparameters.Cn{k1}(1:k2))-plotparameters.Cn{k1}(1) + (1:1:plotparameters.Cn{k1}(k2));
                patch( plotparameters.C{k1}(1,pts), plotparameters.C{k1}(2,pts), plotparameters.C{k1}(3,pts), [0.1 0.1 0.8] );
            end
        end
        xlabel(plotparameters.labels{1});   ylabel(plotparameters.labels{2});   zlabel(plotparameters.labels{3});
        grid on;    legend('x_0', 'x_f');   view([-40, 40]);    axis equal;     pause(1);
    end
    
    
    %% First, attempt to connect x0 to xf directly and check feasibility.
    [ustar_to_goal, Jstar_to_goal]  = feval(STEER, x_0, x_f, t_0, parameters);
    [x_i, u_i, t_i]                 = feval(PROPAGATE, x_0, ustar_to_goal, t_0, parameters );
    m                               = numel(u_i)/(numel(t_i)-1);            % Number of control variables
    
    if ~TERMINATION( t_i(end), x_i(:,end), x_f, parameters ) || feval(CCHECK, x_i, u_i, t_i, parameters )
        % No feasible solution is found; begin generating the RRT.
        feasibility                             = false;
        if plot_tree
            plot3(x_i(plotparameters.states(1),:), x_i(plotparameters.states(2),:), x_i(plotparameters.states(3),:), '-m'); view([-40, 40]); axis equal;
        end
        clearvars ustar_to_goal Jstar_to_goal x_i u_i t_i
    else
        % A feasible solution has been found!  Save the results as a fake node in the tree 
        % at the last (unused) column so that a comparison can be made between the direct 
        % solution and any future RRT-generated solutions (if "soln" = suboptimal, otherwise skip ahead)
        feasibility                             = true;
        tree.node_star                          = Nnodes_max+1;
        tree.nodes(tree.node_star)              = tree.node_star;
        tree.x(:,tree.node_star)                = x_f;
        tree.parents(tree.node_star)            = 1;
        tree.t(tree.node_star)                  = t_i(end);
        tree.ustar_from_parent{tree.node_star}  = ustar_to_goal;
        tree.Jstar_from_x0(tree.node_star)      = tree.Jstar_from_x0( tree.parents(end) ) + Jstar_to_goal;
        tree.Jstar_to_xf(tree.node_star)        = 0;
        tree.ustar_to_xf{tree.node_star}        = [];
        if plot_tree
            plot3(x_i(plotparameters.states(1),:), x_i(plotparameters.states(2),:), x_i(plotparameters.states(3),:), '-b'); view([-40, 40]); axis equal;
        end
    end
    
    
    %% Generate the RRT
    iter = 1;

    while (strcmpi(soln, 'suboptimal') || feasibility == false) && iter <= max_iter && max(T_RUNTIMES) < max_time
        % Sample among the set S for a candidate milestone, x_r
        TIMER1  = tic;
        x_r = SAMPLE(iter);
        T_SAMPLE(iter) = toc;      N_EVALS(1,iter) = N_EVALS(1,iter) + 1;
        
        % Find nearest neighbors, in order of most optimal to least optimal
        tic;
        [nodes_to_xr, ustar_to_xr, Jstar_to_xr] = feval(NN, x_r, tree, STEER, parameters);
        T_NN(iter)     = toc;      N_EVALS(2,iter) = N_EVALS(2,iter) + 1;
        
        % Check the above candidate edges for feasibility, in order of most
        % optimal edge first to least optimal edge last
        i = 1;
        while i <= length(nodes_to_xr)
            % If the i-th control trajectory is not available (e.g. only
            % the nodes_to_xr were generated by NN with ustar_to_xr empty,
            % for instance), generate the required control trajectory
            if length(ustar_to_xr) < i
                tic;
                [ustar_to_xr{i}, Jstar_to_xr(i)] = feval(STEER, tree.x(:,nodes_to_xr(i)), x_r, tree.t(nodes_to_xr(i)), parameters);
                T_STEER(iter)  = T_STEER(iter) + toc;     N_EVALS(3,iter) = N_EVALS(3,iter) + 1;
            end
            tic;
            [x_ir, u_ir, t_ir]  = feval(PROPAGATE, tree.x(:,nodes_to_xr(i)), ustar_to_xr{i}, tree.t(nodes_to_xr(i)), parameters );
            T_PROP(iter)       = T_PROP(iter) + toc;      N_EVALS(4,iter) = N_EVALS(4,iter) + 1;
            
            % Check feasibility.  If safe and feasible, save the candidate edge to the tree, add the candidate
            % milestone as a new vertex and then begin with the next sample candidate in S
            tic;
            if feval(CCHECK, x_ir, u_ir, t_ir, parameters ) == false
                % One of the constraints is violated.  The trajectory from node i to the candidate vertex is not safe.
                % Continue to the next vertex in the tree.
                T_CCHECK(iter) = T_CCHECK(iter) + toc;    N_EVALS(5,iter) = N_EVALS(5,iter) + 1;
                if plot_tree
                    plot3(x_ir(plotparameters.states(1),:), x_ir(plotparameters.states(2),:),   x_ir(plotparameters.states(3),:),   '-r');
                    plot3(x_r(plotparameters.states(1),1),  x_r(plotparameters.states(2),1),    x_r(plotparameters.states(3),1),    'sr'); view([-40, 40]); axis equal;
                end
            else
                T_CCHECK(iter) = T_CCHECK(iter) + toc;    N_EVALS(5,iter) = N_EVALS(5,iter) + 1;
                % Node i can safely connect to the candidate vertex, hence add the candidate vertex, x_r, to the list of
                % vertices in the tree (nodes) and add the trajectory info as an edge
                tic;
                Ncandidate_random_nodes         = size(x_ir,2) - 2;
                index                        	= 1 + ( Nnew_nodes_max_per_iter*(iter-1)+1 );
                tree.nodes(index)               = index;
                tree.x(:,index)                 = x_ir(:,end);
                tree.parents(index)             = nodes_to_xr(i);
                tree.t(index)                   = t_ir(end);
                tree.ustar_from_parent{index}   = ustar_to_xr{i};
                tree.Jstar_from_x0(index)       = tree.Jstar_from_x0( tree.parents(index) ) + Jstar_to_xr(i);

                % Assign weights to each new milestone according to the number of nearby milestones.  Increment the weights of 
                % old milestones that are near the new primary milestone by 1 each
                neighbors                       = find( norms( bsxfun(@minus, bsxfun(@rdivide,tree.x(:,tree.connected_vertices),extents), x_ir(:,end)./extents), 2, 1 ) < treeweights_search_radius );
                tree.weights(index)          	= 1 + length(neighbors);
                tree.weights(tree.connected_vertices(neighbors)) = tree.weights(tree.connected_vertices(neighbors)) + 1;
                tree.connected_vertices(end+1) 	= index;
                new_milestones                  = index;
                
                % Plot the newly added edge.
                if plot_tree
                    tree.plothandles(1,index) 	= plot3(x_ir(plotparameters.states(1),:), x_ir(plotparameters.states(2),:),   x_ir(plotparameters.states(3),:),   '-b');       
                    tree.plothandles(2,index) 	= plot3(x_ir(plotparameters.states(1),end), x_ir(plotparameters.states(2),end),   x_ir(plotparameters.states(3),end),  'sk');
                end
                
                if add_random_intermediate_nodes
                    % Any state along the trajectory is also a vertex. Choose one of these points at random and add it as a
                    % secondary milestone to the tree.
                    if Ncandidate_random_nodes > 0
                        Nnew_nodes              = min(Nnew_nodes_max_per_iter,Ncandidate_random_nodes);
                        Ns                      = sort( 1 + randperm( Ncandidate_random_nodes, Nnew_nodes ) );
                        indices                 = index + 1:Nnew_nodes;
                        tree.nodes(indices)   	= indices;
                        tree.x(:,indices)      	= x_ir(:,Ns);
                        tree.parents(indices)  	= nodes_to_xr(i);
                        tree.t(indices)        	= t_ir(Ns);
                        
                        for k = 1:Nnew_nodes
                            j = indices(k);
%                             if ~isempty( strfind(char(STEER),'impulsive') )
%                                 tree.ustar_from_parent{j}        = [ustar_to_xr{i}(1:m*(Ns(k)-1)); [0 0 0]'; ustar_to_xr{i}(m*(length(t_ir)-1) + (1:m*(Ns(k)-1))); [0 0 0]'];
%                             else
                            tree.ustar_from_parent{j}    	= ustar_to_xr{i}(1:m*(Ns(k)-1));
%                             end
                            tree.Jstar_from_x0(j)       	= tree.Jstar_from_x0( tree.parents(j) ) + feval(COST, tree.ustar_from_parent{j});

                            neighbors                                           = find( norms( bsxfun(@minus, bsxfun(@rdivide,tree.x(:,tree.connected_vertices),extents), x_ir(:,Ns(k))./extents), 2, 1 ) < treeweights_search_radius );
                            tree.weights(index)                                 = 1 + length(neighbors);
                            tree.weights(tree.connected_vertices(neighbors))    = tree.weights(tree.connected_vertices(neighbors)) + 1;
                        end
                        
                        tree.connected_vertices(end+1:Nnew_nodes)       	= indices;
                        new_milestones                                      = [index, indices];

                        % Replot up to the secondary node in case the primary node is rewired and its plot deleted.
                        if plot_tree
                            tree.plothandles(1, index)     = plot3(x_ir(plotparameters.states(1),1:Ns(1)), x_ir(plotparameters.states(2),1:Ns(1)),   x_ir(plotparameters.states(3),1:Ns(1)),   '-b');        
                            tree.plothandles(2, index)     = plot3(x_ir(plotparameters.states(1),Ns(1)),  x_ir(plotparameters.states(2),Ns(1)),    x_ir(plotparameters.states(3),Ns(1)),   'ok'); view([-40, 40]); axis equal;
                        end
                    end
                end
                T_ADDNODES(iter)   = toc;                N_EVALS(6,iter) = N_EVALS(6,iter) + 1;    view([-40, 40]); axis equal;

                for r = new_milestones
                    % Now attempt to connect the new milestones (vertex) x_r to the target state x_f by finding an obstacle-free
                    % steering law and then checking for feasibility/safety
                    tic;
                    [ustar_to_goal, Jstar_to_goal] = feval(STEER, tree.x(:,r), x_f, tree.t(r), parameters );
                    T_STEER2(iter) = T_STEER2(iter) + toc;    N_EVALS(7,iter) = N_EVALS(7,iter) + 1;
                    tic;
                    [x_rf, u_rf, t_rf]  = feval(PROPAGATE, tree.x(:,r), ustar_to_goal, tree.t(r), parameters );
                    T_PROP2(iter) = T_PROP2(iter) + toc;      N_EVALS(8,iter) = N_EVALS(8,iter) + 1;

                    tic;
                    if feval(CCHECK, x_rf, u_rf, t_rf, parameters ) == false
                    	if TERMINATION( t_rf(end), x_rf(:,end), x_f, parameters )
                            % A feasible solution has been found! Save the trajectory from x_r, the new vertex, to the goal x_f
                            feasibility             = true;
                            tree.Jstar_to_xf(r)     = Jstar_to_goal; 
                            tree.ustar_to_xf{r}     = ustar_to_goal;
                        end
                    else
                        % No feasible solution found.  Leave "Jstar_to_xf" and "ustar_to_xf" with NaN and empty values.
                        % Continue to next sample (if neighbors = 0), otherwise check nearest neighbors in tree for re-wiring.
                    end
                    T_CCHECK2(iter) = T_CCHECK2(iter) + toc;  N_EVALS(9,iter) = N_EVALS(9,iter) + 1;
                end
                
                % Continue to re-wire phase and/or proceed to next sample candidate
                break
            end
            i = i + 1;
        end
        
        % Rewire any suboptimal trajectories
        if Nrewire_neighbors_max > 0
            % If a new vertex was added, attempt to re-wire neighboring nodes (unless neighbors = 0), starting from the next node i
            % (previous nodes have more optimal paths from i to r which violated constraints, and thus it is
            % assumed that it is not worth checking if r to i will be a worthy rewiring).
            j = 1; i = i + 1;
            while i <= length(nodes_to_xr) && j <= Nrewire_neighbors_max
                % If re-wiring neighbor vertices (neighbors in terms of Jstar optimality), then continue trying to connect to x_r from
                % neighborhood vertices x_j.  If the most optimal path to x_r and back to x_j is more optimal than the branch from x_j's parent to x_j, re-wire the tree
                tic;
                x_j = tree.x(:,nodes_to_xr(i));
                for r = new_milestones
                    % Now attempt to connect the new milestones x_r back to the neighboring tree vertices x_j by finding an obstacle-free
                    % steering law and then checking for feasibility/safety
                    [ustar_xr_to_xj, Jstar_xr_to_xj] = feval(STEER, tree.x(:,r), x_j, tree.t(r), parameters);
                    [x_rj, u_rj, t_rj]               = feval(PROPAGATE, tree.x(:,r), ustar_xr_to_xj, tree.t(r), parameters );
                    if feval(CCHECK, x_rj, u_rj, t_rj, parameters )
                        % One of the constraints is violated.  The trajectory from x_r to x_j is not safe.
                        % Continue to the next x_r.
                    else
                        rewire_node = nodes_to_xr(i);
                        if tree.Jstar_from_x0( rewire_node ) > tree.Jstar_from_x0(r) + Jstar_xr_to_xj
                            tree.parents( rewire_node )              = r;
                            tree.t( rewire_node )                    = t_rj(end);
                            tree.ustar_from_parent{ rewire_node }    = ustar_xr_to_xj;
                            tree.Jstar_from_x0( rewire_node )        = tree.Jstar_from_x0(r) + Jstar_xr_to_xj;

                            if plot_tree
                                delete( tree.plothandles( 1, rewire_node ) );
                                delete( tree.plothandles( 2, rewire_node ) );
                                tree.plothandles(1, rewire_node )  	= plot3(x_rj(plotparameters.states(1),:), x_rj(plotparameters.states(2),:),   x_rj(plotparameters.states(3),:),   '-c');
                                tree.plothandles(2, rewire_node ) 	= plot3(x_rj(plotparameters.states(1),end), x_rj(plotparameters.states(2),end),   x_rj(plotparameters.states(3),end),   'sk');
                            end
                        end
                    end
                end
                T_REWIRE(iter) = T_REWIRE(iter) + toc;        N_EVALS(10,iter) = N_EVALS(10,iter) + 1;
                i = i + 1;
                j = j + 1;
            end
        end
        
        % Track the current most optimal solution
        [J_STAR(iter), ~] = min(tree.Jstar_from_x0 + tree.Jstar_to_xf);
        T_RUNTIMES(iter) = toc(TIMER1);
        
        % Update the RRT figure
        if plot_tree
            title(['Feasible RRT-generated Trajectories (Iteration ', num2str(iter), ', ', num2str(iter/max_iter*100, '%3.1f'), '% complete)']);
            pause(1);
        end
        
        iter = iter + 1;
    end
    
    % Trim the information saved during RRT generation
    T_RUNTIMES  	= T_RUNTIMES( 1:(iter-1) );
    J_STAR       	= J_STAR( 1:(iter-1) );
    T_SAMPLE        = T_SAMPLE( 1:(iter-1) );
    T_NN            = T_NN( 1:(iter-1) );
    T_STEER         = T_STEER( 1:(iter-1) );
    T_PROP          = T_PROP( 1:(iter-1) );
    T_CCHECK        = T_CCHECK( 1:(iter-1) );
    T_ADDNODES      = T_ADDNODES( 1:(iter-1) );
    T_STEER2        = T_STEER2( 1:(iter-1) );
    T_PROP2         = T_PROP2( 1:(iter-1) );
    T_CCHECK2       = T_CCHECK2( 1:(iter-1) );
    T_REWIRE        = T_REWIRE( 1:(iter-1) );
    N_EVALS         = N_EVALS( :, 1:(iter-1) );
    T_EVALS_MAT     = [T_SAMPLE; T_NN; T_STEER; T_PROP; T_CCHECK; T_ADDNODES; T_STEER2; T_PROP2; T_CCHECK2; T_REWIRE]';
    T_EVALS_LABELS  = {'Sample State-Space'; 'Nearest-Neighbor (NN)'; 'Steering Law (Tree to Candidate)'; 'Propagate (Tree to Candidate)'; ...
                        'Constraint Check (Tree to Candidate)'; 'Add Milestone(s)'; 'Steering Law (Milestone to Goal)'; 'Propagate (Milestone to Goal)'; ...
                        'Constraint Check (Milestone to Goal)'; 'Re-wire Tree'};

    % Find the most optimal solution among all feasible trajectories in the tree
    [J_star, tree.node_star] = min(tree.Jstar_from_x0 + tree.Jstar_to_xf);
    if isempty(tree.node_star)
        tree.node_star = 1;
    end

    % Reconstruct the optimal trajectory using the branch going through node_star
    if ~isempty( tree.ustar_to_xf{tree.node_star} )
        [x_star,u_star,t_star] = feval(PROPAGATE, tree.x(:,tree.node_star), tree.ustar_to_xf{tree.node_star}, tree.t(tree.node_star), parameters );
    else
        x_star = tree.x(:,tree.node_star);
        u_star = [];
        t_star = tree.t(tree.node_star);
    end
    milestones(1) = length(u_star);
    
    i = tree.node_star;
    while tree.nodes(i) ~= 1
        [x_i, u_i, t_i] = feval(PROPAGATE, tree.x(:,tree.parents(i)), tree.ustar_from_parent{i}, tree.t(tree.parents(i)), parameters );
        x_star          = [x_i, x_star(:,2:end)];
        u_star          = [u_i, u_star(:,1:end)];
        t_star          = [t_i, t_star(2:end)];
        milestones      = [length(u_i), milestones(1:end)];
        i               = tree.parents(i);
    end
    milestones  = unique(cumsum([1,milestones]));
    runtime     = toc(TIMER0);
    
    varargout{1} = milestones;
    varargout{2} = runtime;
    varargout{3} = J_STAR;
    varargout{4} = T_RUNTIMES;
    varargout{5} = T_EVALS_MAT;
    varargout{6} = T_EVALS_LABELS;
    varargout{7} = N_EVALS;
end

