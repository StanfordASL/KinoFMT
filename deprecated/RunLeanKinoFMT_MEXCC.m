% RunLeanKinoFMT_MEXCC executes the kinodynamic Fast Marching Trees algorithm 
%   with collision checking handled in a mex function for speed
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

function mpinfo = RunLeanKinoFMT_MEXCC(mpinfo)

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
        % NOTE: nodes2check in transpose of that used in original
        % RunKinoFMT
        % NOTE: ulVerts is transposed when passed to Check CollisionMEX
        nodes2check = reshape(mpinfo.trajMat(...
            mpinfo.evalMat(yMin,x),1:3,:),3,nTrajNodes);
        
        collisionFree = CheckCollisionMEX(nodes2check, ...
            mpinfo.obstacles.cuboids.ulVerts', ...
            mpinfo.environment.bounds);
        
        collision = ~collisionFree;
        if collision % change costMat
            mpinfo.costMat(mpinfo.evalMat(yMin,x),1) = Inf;
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