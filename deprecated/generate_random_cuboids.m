
function obstacles = generate_random_cuboids( max_LWH, min_LWH, NcuboidsMax, stateSpaceObsCoverage, ...
    stateSpaceEdgeClearance, spaceTilingResolution, stateSpaceBounds, stateInit, stateGoal )
    
    % INPUTS
    % max_LWH                   = [ Lmax; Wmax; Hmax ] or a scalar representing the uniform maximum-allowed dimension size
    % min_LWH                   = [ Lmin; Wmin; Hmin ] or a scalar representing the uniform minimum-allowed dimension size
    % NcuboidsMax               = the maximum number of cuboids allowed
    % stateSpaceObsCoverage     = scalar in [0,1] representing fraction of state space volume that should be covered by obstacles
    % stateSpaceEdgeClearance   = scalar in [0, 0.5) representing fraction of state space edge that should be free of obstacles
    % spaceTilingResolution     = scalar needed for collision checking (see areStatesCollided)
    % stateSpaceBounds          = [ lowerBounds, upperBounds ] = [ LB1, UB1; LB2, UB2; ... ]
    % stateInit                 = [ x1_init, x2_init, ..., xn_init ]
    % stateGoal                 = [ x1_goal, x2_goal, ..., xn_goal ]
    %
    % OUTPUTS
    % obstacles
    %
    % Note: stateSpaceEdgeClearance near 0.5 with non-zero differences
    % between max_LWH and min_LWH will always fail, since it must be small 
    % enough to allow completely clear state space edges plus a volume in 
    % the middle that can fully contain randomly-sized and positioned cuboids
    
    % Define state space bounds and span
    xLB         = stateSpaceBounds(1,1);
    yLB         = stateSpaceBounds(2,1);
    zLB         = stateSpaceBounds(3,1);
    xUB         = stateSpaceBounds(1,2);
    yUB         = stateSpaceBounds(2,2);
    zUB         = stateSpaceBounds(3,2);
    LB          = [ xLB; yLB; zLB ];
    span        = [ xUB - xLB; yUB - yLB; zUB - zLB ];
    
    % Prepare for while loop
    maxIter     = NcuboidsMax*20;
    Vspace      = prod(abs(span));
    Vobs        = 0;
    Ncuboids    = 0;
    iteration   = 1;
    checkPaths  = false;
    
    maxCircumscribingSphereRadius = sqrt( sum((0.5*abs(max_LWH - min_LWH)).^2, 1) );
    circumscribingSphereCenters = zeros(3,NcuboidsMax);
    circumscribingSphereRadii   = zeros(1,NcuboidsMax);
    faceVertexCoordinates       = cell(1,NcuboidsMax);
    faceVertexIndices           = cell(1,NcuboidsMax);
    faces                       = cell(1,NcuboidsMax);
    
    % Compute the uniform distributions and lower bounds of obstacle
    % dimensions and centroid positions
    obstacleLWH_span  	= max_LWH - min_LWH;
    obstacleLWH_LB      = min_LWH;
    obstacleCM_LB       = LB + maxCircumscribingSphereRadius + stateSpaceEdgeClearance.*span;
    obstacleCM_span     = (1-2*stateSpaceEdgeClearance).*span - 2*maxCircumscribingSphereRadius;
    
    % Generate the cuboidal obstacles randomly one at a time
    while Vobs/Vspace < stateSpaceObsCoverage && Ncuboids < NcuboidsMax
        
        % Attempt to add a new cuboidal obstacle to the state space
        success = false;
        while ~success && iteration <= maxIter
            % Increment the iteration counter
            iteration = iteration + 1;
            
            % Generate N = 1 new cuboids
            N    	= 1;
            LWH   	= bsxfun( @plus, ...
                    obstacleLWH_LB, ...
                    bsxfun( @times, obstacleLWH_span, rand([3,N]) ) );    	% Length/Width/Height of fixed cuboid obstacles (m) (m-th COLUMN vector corresponds to m-th obstacle)
            YPR   	= 360.*( rand([3,N]) - 0.5 );                           % Yaw/Pitch/Roll of fixed cuboid obstacles (deg) (m-th COLUMN vector corresponds to m-th obstacle)
            CM   	= bsxfun( @plus, ...
                    obstacleCM_LB, ...
                    bsxfun( @times, obstacleCM_span, randn([3,N]) ) );      % Initial position vector of moving cuboid debris (m-th COLUMN vector corresponds to m-th obstacle)
            
            % Temporarily store the new cuboid
            L       = LWH(1,1);
            W       = LWH(2,1);
            H       = LWH(3,1);
            v_CG    = CM(:,1);
            R       = euler2rotmat( YPR(:,1),'321','Display','off' );
            [~, ~, ~, faceVertexIndices{Ncuboids+1}, faceVertexCoordinates{Ncuboids+1}, faces{Ncuboids+1}] ...
                    = polymodel( cuboid( L,W,H,'hideplot',v_CG,R ), 'hideplot', 'hidedisp' );
                
            % Be sure that the sphere circumscribing the cuboid does not
            % contain any other saved cuboids' spheres (to prevent overlap)
            Rsphere                                     = sqrt( (L/2)^2 + (W/2)^2 + (H/2)^2 );
            circumscribingSphereCenters(:,Ncuboids+1)   = v_CG;
            circumscribingSphereRadii(Ncuboids+1)       = Rsphere;
            
            VectorsToSphereCenters              = bsxfun( @minus, circumscribingSphereCenters(:,1:Ncuboids), v_CG );
            DistancesToSphereCenters            = sqrt(sum( VectorsToSphereCenters.^2, 1 ));
            minDistancesToSphereCenters         = circumscribingSphereRadii(1:Ncuboids) + Rsphere;
            if sum( DistancesToSphereCenters < minDistancesToSphereCenters ) > 0
                % The circumscribing sphere of the obstacle collides with
                % another.  Skip ahead and try to generate a new one.
                continue;
            end
             
            % The new obstacle does not overlap any others. Test that 
            % stateInit and stateGoal do not collide with it.
            newObstacle.cuboids.vertices          = faceVertexCoordinates(Ncuboids+1);
            newObstacle.cuboids.faceVertexIndices = faceVertexIndices(Ncuboids+1);
            newObstacle.cuboids.faces             = faces(Ncuboids+1);
            if ~areStatesCollided( [stateInit; stateGoal], newObstacle, spaceTilingResolution, stateSpaceBounds, checkPaths )
                % We've successfully generated a new obstacle
                success = true;
            end
        end
        
        % If unsuccessful and exceeding the maximum iteration count, exit.
        if ~success && iteration > maxIter
            error('Could not generate the desired obstacle coverage within the iteration limit.');
        end
        
        % Otherwise, update the coverage and Ncuboids to account for the 
        % new obstacle and save the already-stored results
        Vobs        = Vobs + prod(abs(LWH));
        Ncuboids    = Ncuboids + 1;
    end
    
    % Shrink the last obstacle to yield the exact desired obstacle coverage
    if Vobs/Vspace >= stateSpaceObsCoverage
        % NOTE: If stateInit and stateGoal do not collide with the current
        % obstacle, they surely will not collide with the shrunken obstacle
        Vshrink = ((Vobs/Vspace) - stateSpaceObsCoverage)*Vspace;
        
        % Compute the new height (L,W,H,v_CG, and R should still correspond
        % to the previously added obstacle).  Overwrite the last obstacle.
        Hnew    = Vshrink/(L*W);
        [~, ~, ~, faceVertexIndices{Ncuboids}, faceVertexCoordinates{Ncuboids}, faces{Ncuboids}] ...
                    = polymodel( cuboid( L,W,Hnew,'hideplot',v_CG,R ), 'hideplot', 'hidedisp' );
        
    else
        warning(['Reached upper limit for the number of cuboidal obstacles (', num2str(NcuboidsMax), ...
            ') without reaching desired obstacle coverage (', num2str(stateSpaceObsCoverage*100), ...
            '%).  Current coverage is ', num2str((Vobs/Vspace)*100), '%.']);
    end
    
    % Truncate results and save to obstacles structure
    obstacles.cuboids.vertices          = faceVertexCoordinates(1:Ncuboids);
    obstacles.cuboids.faceVertexIndices = faceVertexIndices(1:Ncuboids);
    obstacles.cuboids.faces             = faces(1:Ncuboids);
end