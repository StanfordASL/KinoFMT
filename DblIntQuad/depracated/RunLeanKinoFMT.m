% RunLeanKinoFMT executes the kinodynamic Fast Marching Trees algorithm 
%   with collision checking explicitly written to avoid overhead of data
%   passing
%
%   Ross Allen, ASL, Stanford University
%   Oct 20, 2014
%
%   Inputs:
%
%   Functionality:
%
%   Notes:
%       - This code assumes 3 spacial dimensions. Needs re-write for 
%           1D or 2D spacial problems
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function mpinfo = RunLeanKinoFMT(mpinfo)

% Unpack variables to be accessed and modified
%NOTE: modified in loop to avoid having to making additional copies
%costMat

% Unpack variables to be accessed (not modified)
nTotSamples = mpinfo.nTotSamples;
nGoalSamples = mpinfo.sampling.nGoalSamples;
nTrajNodes = mpinfo.sampling.nTrajNodes;

% Prepare variables
Xstart = 1;
Xgoal = [2:nGoalSamples+1];
P = [];                         % Path from Xstart to Xgoal
V = 1:nTotSamples;              % set of all states
Vgoal = Xgoal;                  % goal state(s)
E = [];                         % edges
W = V;                          % states not in tree
W(Xstart) = [];                 % remove Xstart
H = Xstart;                     % states on frontier of tree (just Xstart initially)
z = Xstart;                     % current state
cost2Come = zeros(1,nTotSamples);      	% cost to arrive at each state

% Kinodynamic FMT
while isempty(uniqueIntersect(Vgoal, z))
    NzOut = mpinfo.outNeighborCell{z}(:,1);	% set of nearest outgoing neighbors
    Hnew = [];
    Xnear = uniqueIntersect(NzOut, W);
    for j = 1:length(Xnear)
        x = Xnear(j);
        NxIn = mpinfo.inNeighborCell{x}(:,1);
        Ynear = uniqueIntersect(NxIn, H);
        yMin = -1; 
        xMinCost2Come = inf;
        for i=1:length(Ynear)
            y = Ynear(i);
            yxCost2Come = cost2Come(y) +...
                mpinfo.costMat(mpinfo.evalMat(y,x),1);
            if yxCost2Come <= xMinCost2Come
                yMin = y;
                xMinCost2Come = yxCost2Come;
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%
        % Check collisions %
        %%%%%%%%%%%%%%%%%%%%
        % don't separate into other functions to avoid overhead
        % currently assumes 3 spacial dimensions
        nodes2check = reshape(mpinfo.trajMat(...
            mpinfo.evalMat(yMin,x),1:3,:),3,nTrajNodes)';
        for nodeIter = 1:nTrajNodes-1
%             collisionFree = checkCollision(nodes2check(nodeIter,:),...
%                 nodes2check(nodeIter+1,:),...
%                 mpinfo.obstacles.cuboids.ulVerts, ...
%                 mpinfo.environment.bounds);

            % note: v = nodes2check(nodeIter,:)
            % note: w = nodes2check(nodeIter+1,:)
            node_v = nodes2check(nodeIter,:);
            node_w = nodes2check(nodeIter+1,:);
            collisionFree = 1;
            bb_min = min(node_v,node_w);
            bb_max = max(node_v,node_w);
            
            % check bounds (Assumes bounds exist, will fail otherwise)
            if any(bb_min' < mpinfo.environment.bounds(:,1)) || any(bb_max' > mpinfo.environment.bounds(:,2))
                collisionFree = 0;
            end
            
            % check obstacles
            if (collisionFree)
                for k = 1:2:size(mpinfo.obstacles.cuboids.ulVerts,1)
                    % obs = obstacles(k:k+1,:);
                    % Broadphase Validity check
                    if ~(any(bb_max < mpinfo.obstacles.cuboids.ulVerts(k,:)) || any(bb_min > mpinfo.obstacles.cuboids.ulVerts(k+1,:)))
                        %     if ~BroadphaseValidQ(bb_min, bb_max,obs)
                        % Motion Validity check
                        v_to_w = node_w - node_v;
                        corner = node_v < mpinfo.obstacles.cuboids.ulVerts(k,:);
                        lambdas = (corner.*mpinfo.obstacles.cuboids.ulVerts(k,:) + ...
                            ~corner.*mpinfo.obstacles.cuboids.ulVerts(k+1,:) - node_v)./v_to_w+5*eps;
                        for i_mv = 1:3
                            for i_fcp = 1:3
                                % Face Contains Projection Validity check
                                if i_mv ~= i_fcp && ~(mpinfo.obstacles.cuboids.ulVerts(k,i_fcp) < node_v(i_fcp) + v_to_w(i_fcp)*lambdas(i_mv) && ...
                                        node_v(i_fcp) + v_to_w(i_fcp)*lambdas(i_mv) < mpinfo.obstacles.cuboids.ulVerts(2,i_fcp))
                                    
                                    collisionFree = 0;
                                    break;
                                end
                            end
                            if ~collisionFree
                                break;
                            end
                        end
                        if ~collisionFree
                            break;
                        end
                    end
                    if ~collisionFree
                        break;
                    end
                end
            end
            
            collision = ~collisionFree;
            if collision % change costMat
                mpinfo.costMat(mpinfo.evalMat(yMin,x),1) = Inf;
                break;
            end
        end

        
        if ~collision
            E = [E; yMin x];
            Hnew = [Hnew x];
            W(find(W==x,1)) = [];
            cost2Come(x) = xMinCost2Come; %hmmmmm
        end
    end
    H = [H Hnew];
    H(find(H==z,1)) = [];
    if isempty(H)
        disp('FMT FAILURE: Set H has emptied before Xgoal reached')
        mpinfo.optimalPath = NaN;
        mpinfo.optimalCost = NaN;
        mpinfo.explorationTree = E;
        return
    end
    [~, zind] = min(cost2Come(H));
    z = H(zind);
end

mpinfo.explorationTree = E;
mpinfo.optimalPath = FMTPath(Xstart, Vgoal, E, cost2Come);
mpinfo.optimalCost = cost2Come(z);
% Consolidate results

end

function C = uniqueIntersect(A,B)
% determine intersection of integer vectors with less overhead
%
% Assumes A and B have non repeating values which should be true by
% construction of the sets used in this algorithm
C = [];
As = sort(A);
Bs = sort(B);
if length(As) >= length(Bs)
    C = As(ismembc(As,Bs)); 
else
    C = Bs(ismembc(Bs,As));
end
end