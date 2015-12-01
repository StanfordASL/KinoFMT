% DeepSpace_FMTMotionPlanning.m attempts to solve the kinodynamic motion
% plannning problem
%
%   Ross Allen, ASL, Stanford University
%   Jan 28, 2013
%
%   NOTES:
%   - "DeepSpace" refers to a spacecraft in deepspace (non-orbiting)
%   - After incorporation of initial and goal states, it is assumed they
%   are identified and indexed by 1 and 2, respectively. Note that c++ code
%   will have these states indexed and identified by 0 and 1,
%   respectively. This will need to adapted when goal regions are included
%   - nSamples = number of sampled states (not including init and goal)
%   - ntateDims = dimension of state space (length of state vector)
%   - nTrajNodes = number of nodes used to approximate a trajectory
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

s=dbstatus;
save('myBreakpoints.mat', 's');
clear all
load('myBreakpoints.mat');
dbstop(s);
clc
close all

% Add path for machine learning (ML) functions
addpath([pwd, '/../']);
addpath([pwd, '/../MachineLearning/Reachability Classifier/']);
addpath([pwd, '/../../../../planningdata/GenKinoFMT/']);

% General Parameters
% precompTextFiles = ...
%     '~/Segovia/planningdata/GenKinoFMT/DeepSpacePrecompute_200x10_Jan28-2014_';
% precompMatFile = ...
%     '~/Segovia/planningdata/GenKinoFMT/DeepSpacePrecompute_200x10_Jan28-2014.mat';
precompMatFile = 'DeepSpacePrecompute_200x10_Jan28-2014.mat';
Xinit = [-7 6 -5 0 0 0];      % (m,m,m,m/s,m/s,m/s) initial state
Xgoal = [8 -7 5 0 0 0];      % (m,m,m,m/s,m/s,m/s) goal state


% Machine Learning Parameters
neighborCostThreshold = NaN;        % (cost) radius of neighborhood  NOTE: this should be changed to be based off average cost
nMLTrainingSamples = 90;            % (#) number of training examples for ML. NaN->use all samples
nMLTestingSamples = 20;            % (#) number of training examples for ML. NaN->use all samples
nAddSamples = 0;                  % (#) number of additional samples to run after ML training
% train_options = svmset('polyorder', 4);
train_options = svmset('MaxIter', 5e5, 'polyorder', 2,...);
      'kktviolationlevel', 0, 'boxconstraint', 0.01);

 
% DeepSpace Optimal Control Parameters (NOTE should import some of these so they are guarenteed to be the same)
environment = [];   % no environmental bounds
robot.mass = 1;                % (kg) spacecraft mass
robot.ThrustMax = 0.1;        % (N) spacecraft
options.print_summary = false; % (boo)
options.plot_results = false;  % (b00)
options.storeExitFlags = true;
options.storeSolverOutput = true;
options.solver = optimset('Algorithm','sqp','GradObj','on',...
    'GradConstr','on','DerivativeCheck','off', 'Display', 'off',...
    'TolFun', 1e-2, 'TolCon', 5e-2, 'TolX', 1e-2);      % NOTE: These options can be tuned for fast solution 
options.solver.MaxIter = 10;

%% Generate Obstacles (don't access until online)    NOTE: should check for initial or goal state in obstacles
spaceTilingRes = 10;
bounds = [-10 10; -10 10; -10 10];  % NOTE: beware of [lb ub] format, may not be consistent with Joe's code
LWH_F = [...
    [4 1 1]', ...
    [2 2 2]', ...
    [1 3 3]']; % Length/Width/Height of fixed cuboid obstacles (m) (m-th COLUMN vector corresponds to m-th obstacle)
YPR_F = [...
    [0 0 0]', ...
    [0 0 0]', ...
    [0 0 0]']; % Yaw/Pitch/Roll of fixed cuboid obstacles (deg) (m-th COLUMN vector corresponds to m-th obstacle)
CM_F = [...
    [0 0 0.5]',...
    [1.5 2 0.5]', ...
    [2.5 0.5 0.5]']; % Position vector of fixed cuboid obstacle centroid [m] (m-th COLUMN vector corresponds to m-th obstacle)
% LWH_F = [...
%     [0.1 0.1 0.1]']; % Length/Width/Height of fixed cuboid obstacles (m) (m-th COLUMN vector corresponds to m-th obstacle)
% YPR_F = [...
%     [0 0 0]']; % Yaw/Pitch/Roll of fixed cuboid obstacles (deg) (m-th COLUMN vector corresponds to m-th obstacle)
% CM_F = [...
%     [0 0 5]']; % Position vector of fixed cuboid obstacle centroid [m] (m-th COLUMN vector corresponds to m-th obstacle)

% obsF   = cell(1,size(LWH_F,2));
% obstacles   = cell(1,size(LWH_F,2));
% for i = 1:size(LWH_F,2)
%     [obsV{i}, ~, ~,~,~, obsF{i}] = polymodel(...
%         cuboid( LWH_F(1,i),LWH_F(2,i),LWH_F(3,i),'hideplot',CM_F(:,i),...
%         euler2rotmat( YPR_F(:,i),'321','Display','off' ) ),...
%         'hideplot', 'hidedisp' );
%     obstacles.cuboids.vertices{i} = obsV{i};
%     obstacles.cuboids.faces{i} = obsF{i};
% end
% XinitCol = isStateCollided(Xinit(1:3)',obstacles, spaceTilingRes, bounds);
% if XinitCol
%     disp('Xinit is in collision with obstacle. Redefine problem')
%     return
% end
% XgoalCol = isStateCollided(Xgoal(1:3)',obstacles, spaceTilingRes, bounds);
% if XgoalCol
%     disp('Xinit is in collision with obstacle. Redefine problem')
%     return
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% HI ROSS! USE THIS CODE TO SET UP OBSTACLES, WHEN YOU"RE READY %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
obstacles   = cell(1,size(LWH_F,2));
obstacleFaces   = cell(1,size(LWH_F,2)); 
obstacleVertices = cell(1,size(LWH_F,2)); 
obstacleFaceVertexIndices = cell(1,size(LWH_F,2));
for i = 1:size(LWH_F,2)
    [~, ~, ~, obstacleFaceVertexIndices{i}, obstacleVertices{i}, obstacleFaces{i}] = ...
        polymodel(cuboid(LWH_F(1,i),LWH_F(2,i),LWH_F(3,i),'hideplot', ...
        CM_F(:,i),euler2rotmat( YPR_F(:,i),'321','Display','off')), ...
        'hideplot', 'hidedisp' );
    obstacles.cuboids.vertices{i} = obstacleVertices{i};
    obstacles.cuboids.faces{i} = obstacleFaces{i};
    obstacles.cuboids.faceVertexIndices{i} = obstacleFaceVertexIndices{i};
end
XinitCol = areStatesCollided(Xinit(1:3),obstacles, spaceTilingRes, bounds, false);
if XinitCol
    disp('Xinit is in collision with obstacle. Redefine problem')
    return
end
XgoalCol = areStatesCollided(Xgoal(1:3),obstacles, spaceTilingRes, bounds, false);
if XgoalCol
    disp('Xinit is in collision with obstacle. Redefine problem')
    return
end

%% Load Precomputed Data
if exist('precompTextFiles')
    trajMat = Text2Matrix([precompTextFiles, 'Trajectory.txt']);
    costMat = Text2Matrix([precompTextFiles, 'Cost.txt']);
    stateMat = Text2Matrix([precompTextFiles, 'StateID.txt']);
elseif exist('precompMatFile')
    load(precompMatFile);
else
    disp('Error: no precomputed data supplied. Exiting...')
    return
end
costMat = costMat(:,:,1); % remove neighbor data and recompute later
stateMat(:,1) = [];       % remove state ID tag (doesn't alighn with ML code)
if exist('controlMat')
    clear controlMat
end
[nSamples, nStateDims, nTrajNodes, ~] = size(trajMat);
disp('The information from the text files is done being imported')

%% Calculate neighborhood cost threshold
if ~exist('neighborCostThreshold') || isnan(neighborCostThreshold)
    apprxNumNeighbors = nSamples/4;
    sortedCostMat = sort(costMat,2);
    neighborCostThreshold = sum(sortedCostMat(:,floor(apprxNumNeighbors)+1))/nSamples;
end
disp(['Neighborhood Cost Threshold = ', num2str(neighborCostThreshold')])
disp('------------------------')

%% Train Machine Learning Algorithms
%       NOTE: need to change sampling reduction to sample BVP pairs, not
%       just individual samples
if exist('nMLTrainingSamples') && ~isnan(nMLTrainingSamples) && nMLTrainingSamples <= nSamples
    trainingSamples = randsample(nSamples, nMLTrainingSamples);
    trainingStateMat = stateMat(trainingSamples, :);
    trainingCostMat = costMat(trainingSamples, trainingSamples);
    [svm_output, n_training_errors, percent_training_errors] = ...
        reachability_classifier(trainingStateMat, trainingCostMat, ...
        neighborCostThreshold, @deep_space_extract_2PBVP_features, train_options);
else
    [svm_output, n_training_errors, percent_training_errors] = ...
        reachability_classifier(stateMat, costMat, ...
        neighborCostThreshold, @deep_space_extract_2PBVP_features, train_options);
end
disp(['Number of training errors = ', num2str(n_training_errors)]);
disp(['Training error percentage = ', num2str(percent_training_errors)]);

%% Test SVM Algorithm
if exist('nMLTestingSamples') && ~isnan(nMLTestingSamples) && nMLTestingSamples <= nSamples-nMLTrainingSamples
    testingSamples = [1:nSamples]';
    testingSamples(trainingSamples) = [];
    testingSamples = randsample(testingSamples, nMLTestingSamples);
    testingStateMat = stateMat(testingSamples,:);
    testingCostMat = costMat(testingSamples, testingSamples);
    testingExamplesMat1 = zeros( nMLTestingSamples^2, nStateDims ); % Stores the set of all initial points for 2PBVP's
    testingExamplesMat2 = zeros( nMLTestingSamples^2, nStateDims ); % Stores the set of all initial points for 2PBVP's
    testing_reachability = zeros( nMLTestingSamples^2, 1 ); % Stores the reachability indicators (1 = reachable, -1 = unreachable)
    k = 0;
    for k2 = 1:nMLTestingSamples
        for k1 = 1:nMLTestingSamples
            if k1 ~= k2
                k = k + 1;
                testingExamplesMat1(k,:)  = testingStateMat( k1, : );
                testingExamplesMat2(k,:)  = testingStateMat( k2, : );
                if testingCostMat(k1,k2) <= neighborCostThreshold
                    testing_reachability(k) = 1;    % reachable
                else
                    testing_reachability(k) = -1;   % unreachable
                end
            end
        end
    end
    if k == 0
        error('No feasible cases found for testing reachabiltiy classifier.');
    else
        testingExamplesMat1( (k+1):end, : )  = [];
        testingExamplesMat2( (k+1):end, : )  = [];
        testing_reachability( (k+1):end )    = [];
    end
    testing_data   = deep_space_extract_2PBVP_features( testingExamplesMat1, testingExamplesMat2);
    testing_predicted_reachability = svmclassify(svm_output, testing_data);
    n_testing_errors = nnz(testing_predicted_reachability - testing_reachability);
    percent_testing_errors = 100*n_testing_errors/k;
else
    disp('Invalid number of test samples');
end
disp(['Number of testing errors = ', num2str(n_testing_errors)]);
disp(['Testing error percentage = ', num2str(percent_testing_errors)]);
disp('------------------------')

%% Save/Load for NOTE debugging
save('tempFMTworkspace0')
% load('tempFMTworkspace0')

%% Run more offline samples to more densily populate state space
if exist('nAddSamples') && nAddSamples > 0
    disp('Generating Additional Samples')
    nOrigSamples = nSamples;    
    % Draw new samples from state space
    halton_skip = randi(1e10,1);
    config_space = [-10 10; -10 10; -10 10; -1 1; -1 1; -1 1]; %NOTE needs to match Precomp inputs
    addStates = HaltonSampling(nStateDims, nAddSamples, config_space, ...
        halton_skip, 0, true);    
    % Reform Matrices to incorporate additional samples
    stateMat = [stateMat; addStates];
    trajMat = cat(1, trajMat, NaN*ones(nAddSamples,nStateDims,...
        nTrajNodes,nOrigSamples));
    trajMat = cat(4, trajMat, NaN*ones(nAddSamples+nOrigSamples,...
        nStateDims,nTrajNodes,nAddSamples));
    costMat = cat(1, costMat, Inf*ones(nAddSamples,nOrigSamples));
    costMat = cat(2, costMat, Inf*ones(nAddSamples+nOrigSamples,...
        nAddSamples));
    nSamples = nOrigSamples + nAddSamples;
    % Classify neighborhoods
    for i = nOrigSamples+1:nSamples
        curState = stateMat(i,:);
        curFeatVec = deep_space_extract_2PBVP_features( ...
            repmat(curState,nSamples,1), stateMat(1:end,:));
        curReachSet = is_reachable( svm_output, curFeatVec);
        curReachSet = find(curReachSet);
        numerics.n_nodes = nTrajNodes;
        boundary_values.x0 = curState(1);
        boundary_values.y0 = curState(2);
        boundary_values.z0 = curState(3);
        boundary_values.xdot0 = curState(4);
        boundary_values.ydot0 = curState(5);
        boundary_values.zdot0 = curState(6);
        boundary_values.t0 = 0;
        for iter = 1:length(curReachSet)
            j = curReachSet(iter);
            disp([num2str(i), ', ', num2str(j)])
            boundary_values.xf = stateMat(j,1);
            boundary_values.yf = stateMat(j,2);
            boundary_values.zf = stateMat(j,3);
            boundary_values.xdotf = stateMat(j,4);
            boundary_values.ydotf = stateMat(j,5);
            boundary_values.zdotf = stateMat(j,6);
            % Clear previous problem
            clear bvpinfo
            bvpinfo.numerics = numerics;
            bvpinfo.robot = robot;
            bvpinfo.boundary_values = boundary_values;
            bvpinfo.environment = environment;
            bvpinfo.options = options;
            % Call Solver
            if i == j
                costMat(i,j,1) = 0;
                trajMat(j,1,:,i) = boundary_values.x0*ones(1,1,nTrajNodes,1);
                trajMat(j,2,:,i) = boundary_values.y0*ones(1,1,nTrajNodes,1);
                trajMat(j,3,:,i) = boundary_values.z0*ones(1,1,nTrajNodes,1);
                trajMat(j,4,:,i) = boundary_values.xdot0*ones(1,1,nTrajNodes,1);
                trajMat(j,5,:,i) = boundary_values.ydot0*ones(1,1,nTrajNodes,1);
                trajMat(j,6,:,i) = boundary_values.zdot0*ones(1,1,nTrajNodes,1);
            else
                bvpinfo = DeepSpaceOptimizer(bvpinfo);
                % Save Cost, Trajectory and Control
                costMat(i,j,1) = bvpinfo.solution.cost;
                if bvpinfo.solution.exitflag <= 0
                    costMat(i,j,1) = Inf;
                else
                    trajMat(j,1,:,i) = bvpinfo.solution.x;
                    trajMat(j,2,:,i) = bvpinfo.solution.y;
                    trajMat(j,3,:,i) = bvpinfo.solution.z;
                    trajMat(j,4,:,i) = bvpinfo.solution.xdot;
                    trajMat(j,5,:,i) = bvpinfo.solution.ydot;
                    trajMat(j,6,:,i) = bvpinfo.solution.zdot;
                end
            end
        end
    end
end

%% Reform Data Matrices to incorporate initial and goal data
trajMat = cat(1, NaN*ones(2,nStateDims, nTrajNodes, nSamples), trajMat);
trajMat = cat(4, NaN*ones(nSamples+2,nStateDims,nTrajNodes,1), trajMat);
trajMat = cat(4, NaN*ones(nSamples+2,nStateDims,nTrajNodes,1), trajMat);
stateMat = cat(1, NaN*ones(2, nStateDims), stateMat);
costMat = cat(1, Inf*ones(2,nSamples), costMat);
costMat = cat(2, Inf*ones(nSamples+2, 2), costMat);

%% Generate Neighborhoods (only for currently available samples)
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

%% Going Online
disp('Going Online!')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%  Start Online Computation %%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Save/Load for NOTE debugging
save('tempFMTworkspace1')
% load('tempFMTworkspace1')

%% Approximate neighborhood of initial and goal states with ML
disp('Approximating Neighboorhoods') % NOTE remove later
stateMat(1,:) = Xinit;
stateMat(2,:) = Xgoal;
init_neighbors_feature_matrix  = deep_space_extract_2PBVP_features( ...
    repmat(Xinit,nSamples+2,1), stateMat(1:end,:) );
reachable_from_Xinit    = is_reachable( svm_output,...
    init_neighbors_feature_matrix);
goal_neighbors_feature_matrix = deep_space_extract_2PBVP_features( ...
    stateMat(1:end,:), repmat(Xgoal,nSamples+2,1) );
Xgoal_reachable_from = is_reachable(svm_output,...
    goal_neighbors_feature_matrix);
% NOTE: add check to see that some neighborhood is estimated

%% Optimally connect initial and state to its approximate out-neighboorhood
disp('Optimally Connecting Xinit to out-Neighborhood') % NOTE remove later
numerics.n_nodes = nTrajNodes;
boundary_values.x0 = Xinit(1);
boundary_values.y0 = Xinit(2);
boundary_values.z0 = Xinit(3);
boundary_values.xdot0 = Xinit(4);
boundary_values.ydot0 = Xinit(5);
boundary_values.zdot0 = Xinit(6);
boundary_values.t0 = 0;
for j = 2:nSamples+2 
    if reachable_from_Xinit(j) == 1
        % Set boundary values
        boundary_values.xf = stateMat(j,1);
        boundary_values.yf = stateMat(j,2);
        boundary_values.zf = stateMat(j,3);
        boundary_values.xdotf = stateMat(j,4);
        boundary_values.ydotf = stateMat(j,5);
        boundary_values.zdotf = stateMat(j,6);
        
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
        costMat(1,j,1) = bvpinfo.solution.cost;
        if bvpinfo.solution.exitflag <= 0
            costMat(1,j,1) = Inf;
        else
            trajMat(j,1,:,1) = bvpinfo.solution.x;
            trajMat(j,2,:,1) = bvpinfo.solution.y;
            trajMat(j,3,:,1) = bvpinfo.solution.z;
            trajMat(j,4,:,1) = bvpinfo.solution.xdot;
            trajMat(j,5,:,1) = bvpinfo.solution.ydot;
            trajMat(j,6,:,1) = bvpinfo.solution.zdot;
        end
    end
end

%% Optimally connect goal state to its approximate in-neighboorhood
disp('Optimally Connecting Xgoal to in-Neighborhood') % NOTE remove later
boundary_values.xf = Xgoal(1);
boundary_values.yf = Xgoal(2);
boundary_values.zf = Xgoal(3);
boundary_values.xdotf = Xgoal(4);
boundary_values.ydotf = Xgoal(5);
boundary_values.zdotf = Xgoal(6);
boundary_values.t0 = 0;
for i = [1, 3:nSamples+2]
    if Xgoal_reachable_from(i) == 1
        % Set boundary values
        boundary_values.x0 = stateMat(i,1);
        boundary_values.y0 = stateMat(i,2);
        boundary_values.z0 = stateMat(i,3);
        boundary_values.xdot0 = stateMat(i,4);
        boundary_values.ydot0 = stateMat(i,5);
        boundary_values.zdot0 = stateMat(i,6);
        
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
        costMat(i,2,1) = bvpinfo.solution.cost;
        if bvpinfo.solution.exitflag <= 0
            costMat(i,2,1) = Inf;
        else
            trajMat(2,1,:,i) = bvpinfo.solution.x;
            trajMat(2,2,:,i) = bvpinfo.solution.y;
            trajMat(2,3,:,i) = bvpinfo.solution.z;
            trajMat(2,4,:,i) = bvpinfo.solution.xdot;
            trajMat(2,5,:,i) = bvpinfo.solution.ydot;
            trajMat(2,6,:,i) = bvpinfo.solution.zdot;
        end
    end 
end

%% Sort Neighbors
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

%% Save/Load workspace for NOTE debugging
save('tempFMTworkspace2')
% load('tempFMTworkspace2')

%% Begin Fast Marching Trees Algorithm
disp('Running FMT Algorithm')
P = [];                     % Path from Xinit to Xgoal
V = 1:nSamples+2;           % set of all states
Vgoal = 2;                  % goal state(s)
E = [];                     % edges
W = V;                      % states not in tree
W(1) = [];                  % remove Xinit
H = 1;                      % states in tree (just Xinit)
z = 1;                      % current state
NzOut = costMat(z,:,2);     % set of nearest outgoing neighbors
cost2Come = zeros(1,nSamples+2);    % cost to arrive at each state

while isempty(intersect(Vgoal, z))
    Hnew = [];
    Xnear = intersect(NzOut, W);        %hmmm
    for j = 1:length(Xnear)
        x = Xnear(j);
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
        nodes2check = reshape(trajMat(x,1:3,:,y),3,nTrajNodes)'; 
        collision = areStatesCollided(nodes2check,obstacles,spaceTilingRes,bounds,true);
        if collision % change costMat
            costMat(y,x,1) = Inf;
        end
%         collision = 0;
%         for node_iter = nTrajNodes:-1:2
%             collision = isStateCollided(...
%                 [trajMat(x,1,node_iter,y), trajMat(x,2,node_iter,y), trajMat(x,3,node_iter,y)]',...
%                 obstacles, spaceTilingRes, bounds);
%             if collision % change costMat
%                 costMat(y,x,1) = Inf;
%                 if node_iter == nTrajNodes
%                     costMat(:,x,1) = Inf; % all traj to x are infeasible
%                 end
%                 break;
%             end
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
    NzOut = costMat(z,:,2);
end

P = FMTPath(1, Vgoal, E, cost2Come);

save('tempFMTworkspace3')

%% Plot Results

% Trajectory
quiverScale = 3;
figure
hold on
for i = 1:length(obstacles.cuboids.vertices)  
    [Acol, bcol, ~, ~] = vert2lcon(obstacles.cuboids.vertices{i}');
    plotregion(Acol, -bcol, [], [], [0.7,0.2,0.3],1);
end
plot3(stateMat(1,1),stateMat(1,2),stateMat(1,3),'g^',...
    'MarkerFaceColor','g', 'MarkerSize', 10)
quiver3(stateMat(1,1),stateMat(1,2),stateMat(1,3),...
    stateMat(1,4),stateMat(1,5),stateMat(1,6),quiverScale)
for i = 1:size(P,1)
    plot3(stateMat(P(i,2),1),stateMat(P(i,2),2),stateMat(P(i,2),3),'b*')
    quiver3(stateMat(P(i,2),1),stateMat(P(i,2),2),stateMat(P(i,2),3),...
        stateMat(P(i,2),4),stateMat(P(i,2),5),stateMat(P(i,2),6),quiverScale)
    tempVar = reshape(trajMat(P(i,2),1:3,:,P(i,1)),3,nTrajNodes)';
    plot3(tempVar(:,1),tempVar(:,2),tempVar(:,3),'k.-')
end
xlim([-10 10])
ylim([-10 10])
zlim([-10 10])

