s=dbstatus;
save('myBreakpoints.mat', 's');
clear all
load('myBreakpoints.mat');
dbstop(s);
clc
close all

% Add path for machine learning (ML) functions
addpath([pwd, '/../']);
addpath([pwd, '/../MachineLearning/']);
addpath([pwd, '/../MachineLearning/Reachability Classifier/']);
addpath([pwd, '/../../../../planningdata/GenKinoFMT/']);

load('WAFR_Data_1.mat')

%% Going Online
disp('Going Online!')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%  Start Online Computation %%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


tic
%% Approximate neighborhood of start and goal states with ML
% disp('Approximating Neighboorhoods of Start and Goal States')
stateMat(1,:) = Xstart;
stateMat(2,:) = Xgoal;
start_neighbors_feature_matrix  = deep_space_extract_2PBVP_features( ...
    repmat(Xstart,nSamples+2,1), stateMat(1:end,:) );
reachable_from_Xstart    = is_reachable( svm_output,...
    start_neighbors_feature_matrix);
goal_neighbors_feature_matrix = deep_space_extract_2PBVP_features( ...
    stateMat(1:end,:), repmat(Xgoal,nSamples+2,1) );
Xgoal_reachable_from = is_reachable(svm_output,...
    goal_neighbors_feature_matrix);

%% Approximate cost to neighborhood of start and goal states
start_estimated_neighbors = find(reachable_from_Xstart);
n_start_neighbors = length(start_estimated_neighbors);
start_neighbors_cost_features  = deep_space_extract_2PBVP_features_cost_classifier( ...
    repmat(Xstart,n_start_neighbors,1), stateMat(start_estimated_neighbors,:) );
if useUnweightedBLR
    start_estimated_neighbor_costs =...
        [ones(length(start_neighbors_cost_features(:,1)),1),...
        start_neighbors_cost_features]*blr_output;
else
    [~, start_estimated_neighbor_costs] = batchWeightedLinearRegression(...
        cost_training_features, cost_training_output, blr_best_tau,...
        start_neighbors_cost_features, xExtent);
end
[sortedVal, sortedIX]= sort(start_estimated_neighbor_costs);
start_estimated_neighbors_sorted = [sortedIX, sortedVal];

goal_estimated_neighbors = find(Xgoal_reachable_from);
n_goal_neighbors = length(goal_estimated_neighbors);
goal_neighbors_cost_features  = deep_space_extract_2PBVP_features_cost_classifier( ...
    stateMat(goal_estimated_neighbors,:), repmat(Xgoal,n_goal_neighbors,1) );
if useUnweightedBLR
    goal_estimated_neighbor_costs =...
        [ones(length(goal_neighbors_cost_features(:,1)),1),...
        goal_neighbors_cost_features]*blr_output;
else
    [~, goal_estimated_neighbor_costs] = batchWeightedLinearRegression(...
        cost_training_features, cost_training_output, blr_best_tau,...
        goal_neighbors_cost_features, xExtent);
end
[sortedVal, sortedIX]= sort(goal_estimated_neighbor_costs);
goal_estimated_neighbors_sorted = [sortedIX, sortedVal];

%% Optimally connect start and state to its approximate out-neighboorhood
% disp('Optimally Connecting Xstart to out-Neighborhood') % NOTE remove later
clear boundary_values
boundary_values.x0 = Xstart(1);
boundary_values.y0 = Xstart(2);
boundary_values.z0 = Xstart(3);
boundary_values.xdot0 = Xstart(4);
boundary_values.ydot0 = Xstart(5);
boundary_values.zdot0 = Xstart(6);
boundary_values.t0 = 0;
outNeighbors = [];

neighbor_counter = 0;
j = 1;
while neighbor_counter  < max_neighbors && j <= n_start_neighbors
    
    curNeigh = start_estimated_neighbors_sorted(j,1);
    if curNeigh == 1
        j = j+1;
        continue;
    end
    
    % Set boundary values
    boundary_values.xf = stateMat(curNeigh ,1);
    boundary_values.yf = stateMat(curNeigh ,2);
    boundary_values.zf = stateMat(curNeigh ,3);
    boundary_values.xdotf = stateMat(curNeigh ,4);
    boundary_values.ydotf = stateMat(curNeigh ,5);
    boundary_values.zdotf = stateMat(curNeigh ,6);
    
    % Clear previous problem
    clear bvpinfo
    bvpinfo.numerics = numerics;
    bvpinfo.robot = robot;
    bvpinfo.boundary_values = boundary_values;
    bvpinfo.environment = environment;
    bvpinfo.options = options;
    
    % Call Solver
    bvpinfo = DeepSpaceOptimizer(bvpinfo);
    
    % Save Cost, Trajectory and Control and neighborhood
    if bvpinfo.solution.exitflag > 0
        neighbor_counter = neighbor_counter + 1;
%         % Check Collision
%         nodes2check = [bvpinfo.solution.x, bvpinfo.solution.y, bvpinfo.solution.z]; 
%         collision = areStatesCollided(nodes2check,obstacles,spaceTilingRes,xyz_bounds,true);
%         
%         if ~collision
        n2PBVPs = n2PBVPs+1;
        curCost = bvpinfo.solution.cost;
        evalMat(1,curNeigh) = n2PBVPs;
        costMat(n2PBVPs,1) = curCost;
        trajMat(n2PBVPs,1,:) = bvpinfo.solution.x;
        trajMat(n2PBVPs,2,:) = bvpinfo.solution.y;
        trajMat(n2PBVPs,3,:) = bvpinfo.solution.z;
        trajMat(n2PBVPs,4,:) = bvpinfo.solution.xdot;
        trajMat(n2PBVPs,5,:) = bvpinfo.solution.ydot;
        trajMat(n2PBVPs,6,:) = bvpinfo.solution.zdot;
        controlMat(n2PBVPs,1,:) = bvpinfo.solution.ux;
        controlMat(n2PBVPs,2,:) = bvpinfo.solution.uy;
        controlMat(n2PBVPs,3,:) = bvpinfo.solution.uz;
        controlMat(n2PBVPs,4,:) = bvpinfo.solution.eta;
        outNeighbors = [outNeighbors; [curNeigh curCost]];
        if isempty(inNeighborCell{1,curNeigh})
            inNeighborCell{1,curNeigh}(1,:) = [1 curCost];
        else
            inPos = find(inNeighborCell{1,curNeigh}(:,2) > curCost,1,'first');
            if isempty(inPos)
                inNeighborCell{1,curNeigh} = [inNeighborCell{1,curNeigh}; 1 curCost];
            else
                inNeighborCell{1,curNeigh} = [inNeighborCell{1,curNeigh}(1:inPos,:);...
                    inNeighborCell{1,curNeigh}(inPos:end,:)];
                inNeighborCell{1,curNeigh}(1,:) = [1 curCost];
            end
        end
%         end
    end
    
    j = j+1;
end
[~,sortIX] = sort(outNeighbors(:,2));
outNeighbors = outNeighbors(sortIX,:);
outNeighborCell{1,1} = outNeighbors';


%% Optimally connect goal state to its approximate in-neighboorhood
% disp('Optimally Connecting Xgoal to in-Neighborhood')
clear boundary_values
boundary_values.xf = Xgoal(1);
boundary_values.yf = Xgoal(2);
boundary_values.zf = Xgoal(3);
boundary_values.xdotf = Xgoal(4);
boundary_values.ydotf = Xgoal(5);
boundary_values.zdotf = Xgoal(6);
boundary_values.t0 = 0;
inNeighbors = [];

neighbor_counter = 0;
i = 1;
while neighbor_counter  < max_neighbors && i <= n_goal_neighbors 
    
    curNeigh = goal_estimated_neighbors_sorted(i,1);
    if curNeigh == 2
        i = i+1;
        continue;
    end
    
    % Set boundary values
    boundary_values.x0 = stateMat(curNeigh,1);
    boundary_values.y0 = stateMat(curNeigh,2);
    boundary_values.z0 = stateMat(curNeigh,3);
    boundary_values.xdot0 = stateMat(curNeigh,4);
    boundary_values.ydot0 = stateMat(curNeigh,5);
    boundary_values.zdot0 = stateMat(curNeigh,6);
    
    % Clear previous problem
    clear bvpinfo
    bvpinfo.numerics = numerics;
    bvpinfo.robot = robot;
    bvpinfo.boundary_values = boundary_values;
    bvpinfo.environment = environment;
    bvpinfo.options = options;
    
    % Call Solver
    bvpinfo = DeepSpaceOptimizer(bvpinfo);
    
    % Save Cost, Trajectory and Control
    if bvpinfo.solution.exitflag > 0
        neighbor_counter = neighbor_counter + 1;
        
%         % Check Collision
%         nodes2check = [bvpinfo.solution.x, bvpinfo.solution.y, bvpinfo.solution.z]; 
%         collision = areStatesCollided(nodes2check,obstacles,spaceTilingRes,xyz_bounds,true);
%         
%         if ~collision
        n2PBVPs = n2PBVPs+1;
        curCost = bvpinfo.solution.cost;
        evalMat(curNeigh,2) = n2PBVPs;
        costMat(n2PBVPs,1) = curCost;
        trajMat(n2PBVPs,1,:) = bvpinfo.solution.x;
        trajMat(n2PBVPs,2,:) = bvpinfo.solution.y;
        trajMat(n2PBVPs,3,:) = bvpinfo.solution.z;
        trajMat(n2PBVPs,4,:) = bvpinfo.solution.xdot;
        trajMat(n2PBVPs,5,:) = bvpinfo.solution.ydot;
        trajMat(n2PBVPs,6,:) = bvpinfo.solution.zdot;
        controlMat(n2PBVPs,1,:) = bvpinfo.solution.ux;
        controlMat(n2PBVPs,2,:) = bvpinfo.solution.uy;
        controlMat(n2PBVPs,3,:) = bvpinfo.solution.uz;
        controlMat(n2PBVPs,4,:) = bvpinfo.solution.eta;
        inNeighbors = [inNeighbors; [curNeigh curCost]];
        if isempty(outNeighborCell{curNeigh,1})
            outNeighborCell{curNeigh,1}(:,1) = [2; curCost];
        else
            outPos = find(outNeighborCell{curNeigh,1}(2,:) > curCost,1,'first');
            if isempty(outPos)
                outNeighborCell{curNeigh,1} = [outNeighborCell{curNeigh,1}, [2; curCost]];
            else
                outNeighborCell{curNeigh,1} = [outNeighborCell{curNeigh,1}(:,1:outPos),...
                    outNeighborCell{curNeigh,1}(:,outPos:end)];
                outNeighborCell{curNeigh,1}(:,outPos) = [2; curCost];
            end
        end
%         end
    end
    
    i = i+1;
end
[~,sortIX] = sort(inNeighbors(:,2));
inNeighbors = inNeighbors(sortIX,:);
inNeighborCell{1,2} = inNeighbors;


%% Save/Load workspace for NOTE debugging
% save('tempFMTworkspace2')
% load('tempFMTworkspace2')

%% Begin Fast Marching Trees Algorithm
% disp('Running FMT Algorithm')
Xstart_proxy = 1;
Xgoal_proxy = 2;
P = [];                     % Path from Xstart to Xgoal
V = 1:nSamples+2;           % set of all states
Vgoal = Xgoal_proxy;                  % goal state(s)
E = [];                     % edges
W = V;                      % states not in tree
W(Xstart_proxy) = [];                  % remove Xstart
H = Xstart_proxy;                      % states in tree (just Xstart)
z = Xstart_proxy;                      % current state
NzOut = outNeighborCell{z,1};     % set of nearest outgoing neighbors
cost2Come = zeros(1,nSamples+2);    % cost to arrive at each state

while isempty(intersect(Vgoal, z))
    Hnew = [];
    Xnear = intersect(NzOut, W);        %hmmm
    for j = 1:length(Xnear)
        x = Xnear(j);
        NxIn = inNeighborCell{1,x};
        Ynear = intersect(NxIn, H);     %hmmmm
        yMin = -1; 
        xMinCost2Come = inf;
        for i=1:length(Ynear)
            y = Ynear(i);
            yxCost2Come = cost2Come(y) + costMat(evalMat(y,x),1);
            if yxCost2Come <= xMinCost2Come
                yMin = y;
                xMinCost2Come = yxCost2Come;
            end
        end
        
        % Check Collision
        collision = false;
        
%         if exist('obstacles', 'var')
            nodes2check = reshape(trajMat(evalMat(y,x),1:3,:),3,nTrajNodes)';
            collision = areStatesCollided(nodes2check,obstacles,spaceTilingRes,xyz_bounds,true);
            if collision % change costMat
                costMat(evalMat(y,x),1) = Inf;
            end
%         end
        
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
        disp('FMT Fail: Set H has emptied before Xgoal reached')
        return
    end
    [~, zind] = min(cost2Come(H));
    z = H(zind);
    NzOut = outNeighborCell{z,1};
end

P = FMTPath(Xstart_proxy, Vgoal, E, cost2Come);
% P = [1 Xstart_proxy;...
%     P;...
%     Xgoal_proxy 2];
toc
save('tempFMTworkspace3')