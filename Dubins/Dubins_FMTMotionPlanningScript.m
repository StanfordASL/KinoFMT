% Dubins_FMTMotionPlanning.m attempts to solve the kinodynamic motion
% plannning problem
%
%   Ross Allen, ASL, Stanford University
%   Jan 24, 2013
%
%   NOTES:
%   - After incorporation of initial and goal states, it is assumed they
%   are identified and indexed by 1 and 2, respectively. Note that c++ code
%   will have these states indexed and identified by 0 and 1,
%   respectively. This will need to adapted when goal regions are included
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

s=dbstatus;
save('myBreakpoints.mat', 's');
clear all
load('myBreakpoints.mat');
dbstop(s);
clear s
clc
close all

% Add path for machine learning (ML) functions
addpath([pwd, '/MachineLearning/Reachability Classifier/']);

% General Parameters
% precompDataFiles = ...
%     '~/Segovia/planningdata/GenKinoFMT/DubinsPrecomputeV2_500x10_Jan20-2014_';
load('~/Segovia/planningdata/GenKinoFMT/DubinsPrecomputeV2_500x10_Jan20-2014.mat');
Xinit = [-7 -5 0];       % (m, m, deg) initial state
Xgoal = [8 5 180];      % (m, m, deg) goal state

% Machine Learning Parameters
neighborCostThreshold = 12;        % (cost) radius of neighborhood  NOTE: this should be changed to be based off average cost
nMLTrainingSamples = 150;            % (#) number of training examples for ML
train_options = svmset('MaxIter', 5e5, 'polyorder', 4, ...
     'kktviolationlevel', 0.02, 'boxconstraint', 5);
 
% Dubins Optimal Control Parameters
environment.xbounds = [-10 10];
environment.ybounds = [-10 10];
options.print_summary = false; % (boo)
options.plot_results = false;  % (b00)
options.storeExitFlags = true;
options.storeSolverOutput = true;
options.solver = optimset('Algorithm','sqp','GradObj','on',...
    'GradConstr','on','DerivativeCheck','off', 'Display', 'off',...
    'TolFun', 1e-6, 'TolCon', 1e-6, 'TolX', 1e-6);      % NOTE: These options can be tuned for fast solution
options.solver.MaxFunEvals = Inf; 
options.solver.MaxIter = 1000;
robot.V = 1;                % (m/s) dubin's constant velocity
robot.turnrate = 45;        % (deg/s) dubin's max turnrate

% Generate Obstacles    NOTE: should check for initial or goal state in obstacles
spaceTilingRes = 5;
LWH_F = [...
    [4 1 1]', ...
    [2 1 1]', ...
    [1 1 1]']; % Length/Width/Height of fixed cuboid obstacles (m) (m-th COLUMN vector corresponds to m-th obstacle)
YPR_F = [...
    [0 0 0]', ...
    [0 0 0]', ...
    [0 0 0]']; % Yaw/Pitch/Roll of fixed cuboid obstacles (deg) (m-th COLUMN vector corresponds to m-th obstacle)
CM_F = [...
    [0 0 0.5]',...
    [1.5 2 0.5]', ...
    [2.5 .5 0.5]']; % Position vector of fixed cuboid obstacle centroid [m] (m-th COLUMN vector corresponds to m-th obstacle)
bounds = [fliplr(environment.xbounds); fliplr(environment.ybounds); 1 0];
obsF   = cell(1,size(LWH_F,2));
obstacles   = cell(1,size(LWH_F,2));
for i = 1:size(LWH_F,2)
    [obsV{i}, ~, ~,~,~, obsF{i}] = polymodel(...
        cuboid( LWH_F(1,i),LWH_F(2,i),LWH_F(3,i),'hideplot',CM_F(:,i),...
        euler2rotmat( YPR_F(:,i),'321','Display','off' ) ),...
        'hideplot', 'hidedisp' );
    obstacles.cuboids.vertices{i} = obsV{i};
    obstacles.cuboids.faces{i} = obsF{i};
end

% Load Precomputed Data
if exist('precompDataFiles')
    trajMat = Text2Matrix([precompDataFiles, 'Trajectory.txt']);
    costMat = Text2Matrix([precompDataFiles, 'Cost.txt']);
    stateMat = Text2Matrix([precompDataFiles, 'StateID.txt']);
end
costMat = costMat(:,:,1); % remove neighbor data and recompute later
stateMat(:,1) = [];       % remove state ID tag (doesn't alighn with ML code)
%   NOTE: should add code that sets trajectory nodes to NaN for infeasible
%   connections
if exist('controlMat')
    clear controlMat
end
[nSamples, nStateDims, nTrajNodes, ~] = size(trajMat);
    % nSamples = number of sampled states (not including init and goal)
    % nStateDims = dimension of state space (length of state vector)
    % nTrajNodes = number of nodes used to approximate a trajectory
    % (includes inititial and final states for each trajectory)
disp('The information from the text files is done being imported')
    

% Train Machine Learning Algorithms
%       Note: need to change sampling reduction to sample BVP pairs, not
%       just individual samples
if exist('nMLTrainingSamples')
    trainingSamples = randsample(nSamples, nMLTrainingSamples);
    redStateMat = stateMat(trainingSamples, :);
    redCostMat = costMat(trainingSamples, trainingSamples);
    svm_output = reachability_classifier(redStateMat, redCostMat, ...
        neighborCostThreshold, @dubins_extract_2PBVP_features, train_options);
else
    svm_output = reachability_classifier(stateMat, costMat, ...
    neighborCostThreshold, @dubins_extract_2PBVP_features, train_options);
end 

% Reform Data Matrices to incorporate initial and goal data
trajMat = cat(1, NaN*ones(2,nStateDims, nTrajNodes, nSamples), trajMat);
trajMat = cat(4, NaN*ones(nSamples+2,nStateDims,nTrajNodes,1), trajMat);
trajMat = cat(4, NaN*ones(nSamples+2,nStateDims,nTrajNodes,1), trajMat);
stateMat = cat(1, NaN*ones(2, nStateDims), stateMat);
costMat = cat(1, Inf*ones(2,nSamples), costMat);
costMat = cat(2, Inf*ones(nSamples+2, 2), costMat);


% Generate Neighborhoods (only for currently available samples)
costMat = cat(3, costMat, NaN*ones(nSamples+2,nSamples+2,2));
for i = 3:nSamples+2
    % outgoing neighbor set
    [sortedCost, sortedInd] = sort(costMat(i,:,1));
    ind = find(sortedCost > neighborCostThreshold, 1, 'first');
    costMat(i,1:ind-1,2) = sortedInd(1:ind-1); 
    
    % incoming neighbor set
    [sortedCost, sortedInd] = sort(costMat(:,i,1));
    ind = find(sortedCost > neighborCostThreshold, 1, 'first');
    costMat(i,1:ind-1,3) = sortedInd(1:ind-1);
end

disp('Going Online!')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%  Start Online Computation %%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Save/Load for NOTE debugging
save('tempFMTworkspace1')
% load('tempFMTworkspace1')

tic 

% Approximate neighborhood of initial and goal states with ML
disp('Approximating Neighboorhoods') % NOTE remove later
stateMat(1,:) = Xinit;
stateMat(2,:) = Xgoal;
init_neighbors_feature_matrix  = dubins_extract_2PBVP_features( ...
repmat(Xinit,nSamples+2,1), stateMat(1:end,:) );
reachable_from_Xinit    = is_reachable( svm_output,...
    init_neighbors_feature_matrix);
goal_neighbors_feature_matrix = dubins_extract_2PBVP_features( ...
stateMat(1:end,:), repmat(Xgoal,nSamples+2,1) );
Xgoal_reachable_from = is_reachable(svm_output,...
    goal_neighbors_feature_matrix);

% Optimally connect initial and state to its approximate out-neighboorhood
disp('Optimally Connecting Xinit to out-Neighborhood') % NOTE remove later
numerics.n_nodes = nTrajNodes;
boundary_values.x0 = Xinit(1);
boundary_values.y0 = Xinit(2);
boundary_values.theta0 = Xinit(3);
boundary_values.t0 = 0;
for j = 2:nSamples+2 
    if reachable_from_Xinit(j) == 1
        % Set boundary values
        boundary_values.xf = stateMat(j,1);
        boundary_values.yf = stateMat(j,2);
        boundary_values.thetaf = stateMat(j,3);
        
        % Clear previous problem
        clear dubprob
        dubprob.numerics = numerics;
        dubprob.robot = robot;
        dubprob.boundary_values = boundary_values;
        dubprob.environment = environment;
        dubprob.options = options;
        
        % Call Solver
        dubprob = DubinsOptimizer(dubprob);
        
        % Save Cost, Trajectory and Control
        costMat(1,j,1) = dubprob.solution.cost;
        if dubprob.solution.exitflag <= 0
            costMat(1,j,1) = Inf;
        else
            trajMat(j,1,:,1) = dubprob.solution.x;
            trajMat(j,2,:,1) = dubprob.solution.y;
            trajMat(j,3,:,1) = dubprob.solution.theta;
        end
    end
end

% Optimally connect goal state to its approximate in-neighboorhood
disp('Optimally Connecting Xgoal to in-Neighborhood') % NOTE remove later
boundary_values.xf = Xgoal(1);
boundary_values.yf = Xgoal(2);
boundary_values.thetaf = Xgoal(3);
boundary_values.t0 = 0;
for i = [1, 3:nSamples+2]
    if Xgoal_reachable_from(i) == 1
        % Set boundary values
        boundary_values.x0 = stateMat(i,1);
        boundary_values.y0 = stateMat(i,2);
        boundary_values.theta0 = stateMat(i,3);
        
        % Clear previous problem
        clear dubprob
        dubprob.numerics = numerics;
        dubprob.robot = robot;
        dubprob.boundary_values = boundary_values;
        dubprob.environment = environment;
        dubprob.options = options;
        
        % Call Solver
        dubprob = DubinsOptimizer(dubprob);
        
        % Save Cost, Trajectory and Control
        costMat(i,2,1) = dubprob.solution.cost;
        if dubprob.solution.exitflag <= 0
            costMat(i,2,1) = Inf;
        else
            trajMat(2,1,:,i) = dubprob.solution.x;
            trajMat(2,2,:,i) = dubprob.solution.y;
            trajMat(2,3,:,i) = dubprob.solution.theta;
        end
    end 
end

% Sort Neighbors
disp('Sorting Neighbors')
for i = 1:nSamples+2
    % outgoing neighbor set
    [sortedCost, sortedInd] = sort(costMat(i,:,1));
    ind = find(sortedCost > neighborCostThreshold, 1, 'first');
    costMat(i,1:ind-1,2) = sortedInd(1:ind-1); 
    
    % incoming neighbor set
    [sortedCost, sortedInd] = sort(costMat(:,i,1));
    ind = find(sortedCost > neighborCostThreshold, 1, 'first');
    costMat(i,1:ind-1,3) = sortedInd(1:ind-1);
end

% Save/Load workspace for NOTE debugging
save('tempFMTworkspace2')
% load('tempFMTworkspace2')

% Begin Fast Marching Trees Algorithm
disp('Running FMT Algorithm')
V = 1:nSamples+2;          % set of all states
Vgoal = 2;                  % goal state(s)
E = [];                     % edges
W = V;                      % states not in tree
W(1) = [];                  % remove Xinit
H = 1;                      % states in tree (just Xinit)
z = 1;                      % current state
NzOut = costMat(z,:,2);     % set of nearest outgoing neighbors
% NzIn = costMat(z,:,3);      % set of nearest incoming neighbors
cost2Come = zeros(1,nSamples+2);    % cost to arrive at each state

while isempty(intersect(Vgoal, z))
    Hnew = [];
    Xnear = intersect(NzOut, W);        %hmmm
    for j = 1:length(Xnear)
        x = Xnear(j);
%         NxOut = costMat(x,:,2);
        NxIn = costMat(x,:,3);
        Ynear = intersect(NxIn, H);     %hmmmm
        yMin = -1; 
        xMinCost2Come = inf;
        for i=1:length(Ynear)
            y = Ynear(i);
            yxCost2Come = cost2Come(y) + costMat(y,x,1);
            if yxCost2Come <= xMinCost2Come
                yMin = y;
                xMinCost2Come = yxCost2Come;
            end
        end
        
        % Check Collision
        collision = 0;
        for node_iter = nTrajNodes:-1:2
            collision = isStateCollided(...
                [trajMat(x,1,node_iter,y), trajMat(x,2,node_iter,y), 0]',...
                obstacles, spaceTilingRes, bounds);
            if collision % change costMat
                costMat(y,x,1) = Inf;
                if node_iter == nTrajNodes
                    costMat(:,x,1) = Inf; % all traj to x are infeasible
                end
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
        disp('FMT Fail')
        return
    end
    [~, zind] = min(cost2Come(H));
    z = H(zind);
    NzOut = costMat(z,:,2);
end

P = FMTPath(1, Vgoal, E, cost2Come);

toc

save('tempFMTworkspace3')

