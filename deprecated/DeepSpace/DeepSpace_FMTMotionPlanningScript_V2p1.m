% DeepSpace_FMTMotionPlanningScipt_V2-1.m attempts to solve the kinodynamic motion
% plannning problem. This only executes the online phase and minimizes
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
%
%   NOTES on V2:
%   - uses new training data format (see PrecompScript_V2)
%   - 'IDs' refer to the row index of a state in stateMat
%   - 'CaseNums' refers to the case number of a successful 2PBVP problem 
%   (i.e. the row index of a specific initStateIDs and finalStateIDs pair)
%   - n2PBVPs specifically refers to the number of successful 2PBVPs
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
addpath([pwd, '/../MachineLearning/']);
addpath([pwd, '/../MachineLearning/Reachability Classifier/']);
addpath([pwd, '/../../../../planningdata/GenKinoFMT/']);

load('DeepSpaceAllOfflineSave_2000x10000x10_Mar11-2014.mat')
max_neighbors = 1; % sets the total number of 2pbvps to solve online
n_cost_trainings = 5;
nCostTestingSamples = 50;

%% Remove additional matrix elements that were added for start and goal data
% !!!NOTE!!! only do this if the loaded data has nan rows
if isnan(stateMat(1,1))
    stateMat(1:2,:) = [];
    evalMat(1:2,:) = [];
    evalMat(:,1:2) = [];
end

%% Define new start and goal states
Xstart = [70 70 70 -1 -1 -1];      % (m,m,m,m/s,m/s,m/s) initial state
Xgoal = [-70 -70 -70  0 0 0];      % (m,m,m,m/s,m/s,m/s) goal state

%% re-Generate Obstacles (overwrite loaded obstacles
clear obstacles
% max_LWH = [10; 10; 10];
% min_LWH = [5; 5; 5];
% NcuboidsMax = 10;
% stateSpaceObsCoverage = 0.1;
% stateSpaceEdgeClearance = 0.05;
% spaceTilingRes = 100;
% xyz_bounds = config_space(1:3,:); % get from precomp
% disp('----------------------')
% disp('Generating random cuboids')
% disp('----------------------')
% obstacles = generate_random_cuboids( max_LWH, min_LWH, NcuboidsMax, stateSpaceObsCoverage, ...
%     stateSpaceEdgeClearance, spaceTilingRes, 0.75*xyz_bounds, Xstart(1:3), Xgoal(1:3) );
%
nObstacles = 10;
minObstacleDimension = 10;
maxObstacleDimension = 50;
[LWH_F, YPR_F, CM_F] = generateObstacles(nObstacles, xyz_bounds, minObstacleDimension, maxObstacleDimension);
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
% 
% % Check if initial or final states are immediately obstructed (in this is
% % done to avoid wasting. In reality, it is impossible to start in an
% % obstacle)
% XstartCol = areStatesCollided(Xstart(1:3),obstacles, spaceTilingRes, xyz_bounds, false);
% if XstartCol
%     disp('Xstart is in collision with obstacle. Redefine problem')
%     return
% end
% XgoalCol = areStatesCollided(Xgoal(1:3),obstacles, spaceTilingRes, xyz_bounds, false);
% if XgoalCol
%     disp('Xgoal is in collision with obstacle. Redefine problem')
%     return
% end

%% Train unweighted cost estimator
% disp('Training Unweighted Batched Linear Regression Cost Estimator')
% [blr_output, blr_error_rms, blr_error_max]  =...
%     DeepSpaceTrainCostEstimator( stateMat, evalMat, costMat,...
%     trainingInitStateIDs, trainingFinalStateIDs,...
%     @deep_space_extract_2PBVP_features_cost_classifier); 
% 
% disp(['RMS training error of cost estimator = ', num2str(blr_error_rms)]);
% disp(['Max training error of cost estimator = ', num2str(blr_error_max)]);
% disp('---------------------------')
% 
% %% Test unweighted cost estimator
% disp('Testing Unweighted Batched Linear Regression Cost Estimator')
% testing_cost = zeros( nMLTestingSamples, 1 ); % Stores the reachability indicators (1 = reachable, -1 = unreachable)
% for k = 1:nMLTestingSamples
%     curTotCaseNum = testingSampleTotCaseNums(k,1);
%     testing_cost(k) = costMat(evalMat(initStateIDs(curTotCaseNum,1),...
%             finalStateIDs(curTotCaseNum,1)));
% end
% testing_cost_data   = deep_space_extract_2PBVP_features_cost_classifier( testingExamplesMat1, testingExamplesMat2);
% testing_estimated_cost = [ones(length(testing_cost_data(:,1)),1),...
%     testing_cost_data]*blr_output;
% testing_cost_error = testing_estimated_cost - testing_cost;
% testing_blr_error_rms = sqrt(sum(testing_cost_error.^2)/length(testing_cost_error));
% testing_blr_error_max = max(testing_cost_error);
% 
% disp(['RMS testing error of cost estimator = ', num2str(testing_blr_error_rms)]);
% disp(['Max testing error of cost estimator = ', num2str(testing_blr_error_max)]);
% disp('---------------------------')

%% Train & test weighted cost estimator
disp('Training/Testing Weighted Batched Linear Regression Cost Estimator')
% blr_taus = [0.1 0.2 .5 .7 1];
% blr_taus = [0.2 0.3];
blr_taus = [];
for k = 1:n_cost_trainings
    costTrainingSampleTotCaseNums = randsample(validTotCaseNums, nMLTrainingSamples); % sample without replacement
    costTrainingInitStateIDs = initStateIDs(costTrainingSampleTotCaseNums,1);
    costTrainingFinalStateIDs = finalStateIDs(costTrainingSampleTotCaseNums,1);
    costTrainingExamplesMat1 = stateMat(costTrainingInitStateIDs,:); % Stores the set of all initial points for 2PBVP's
    costTrainingExamplesMat2 = stateMat(costTrainingFinalStateIDs,:); % Stores the set of all final points for 2PBVP's
    cost_training_features   = deep_space_extract_2PBVP_features_cost_classifier( costTrainingExamplesMat1, costTrainingExamplesMat2);
    cost_training_output   = Inf*ones(size(cost_training_features,1),1);
    for i = 1:size(cost_training_features,1)
        cost_training_output(i,1) = costMat(evalMat(costTrainingInitStateIDs(i,1),costTrainingFinalStateIDs(i,1)));
    end
    xExtent = max(cost_training_features) - min(cost_training_features);

    
    costTestingSampleTotCaseNums = [1:nTot2PBVPs]';
    costTestingSampleTotCaseNums(costTrainingSampleTotCaseNums) = []; % remove 2PBVPs used in training
    costTestingSampleTotCaseNums(find(validTotCaseNumsBool(...
        costTestingSampleTotCaseNums,1)==0)) = []; % remove invalid 2PBVPs
    costTestingSampleTotCaseNums = randsample(costTestingSampleTotCaseNums,...
        nCostTestingSamples ); % sample without replacement
    costTestingInitStateIDs = initStateIDs(costTestingSampleTotCaseNums,1);
    costTestingFinalStateIDs = finalStateIDs(costTestingSampleTotCaseNums,1);
    costTestingExamplesMat1 = stateMat(costTestingInitStateIDs,:); % Stores the set of all initial points for 2PBVP's
    costTestingExamplesMat2 = stateMat(costTestingFinalStateIDs,:); % Stores the set of all final points for 2PBVP's
    cost_testing_features   = deep_space_extract_2PBVP_features_cost_classifier( costTestingExamplesMat1, costTestingExamplesMat2);
    cost_testing_output   = Inf*ones(size(cost_testing_features,1),1);
    for i = 1:size(cost_testing_features,1)
        cost_testing_output(i,1) = costMat(evalMat(costTestingInitStateIDs(i,1),costTestingFinalStateIDs(i,1)));
    end
    
    % Unweighted
    blr_output = batchUnweightedLinearRegression(...
        cost_training_features, cost_training_output);
    curTestingEstOutput = [ones(length(cost_testing_features(:,1)),1),...
        cost_testing_features]*blr_output;
    curCostError = curTestingEstOutput - cost_testing_output;
    testingErrorMax(1,k) = max(abs(curCostError));
    testingErrorRms(1,k) = sqrt(sum(curCostError.^2)/length(curCostError));
    
    % Weighted
    for i = 1:length(blr_taus)
        [~, curTestingEstOutput] = batchWeightedLinearRegression(...
            cost_training_features, cost_training_output, blr_taus(i),...
            cost_testing_features, xExtent);
        curCostError = curTestingEstOutput - cost_testing_output;
        testingErrorMax(i+1,k) = max(abs(curCostError));
        testingErrorRms(i+1,k) = sqrt(sum(curCostError.^2)/length(curCostError));
    end
    
end

avgTestingErrorRms = sum(testingErrorRms,2)/size(testingErrorRms,2);
maxTestingErrorMax = max(testingErrorMax,[],2);
[~, blr_best_tau] = min(avgTestingErrorRms);
useUnweightedBLR = true;
if blr_best_tau == 1
    useUnweightedBLR = true;
else
    blr_best_tau = blr_taus(blr_best_tau - 1);
end

%% Recapture training data from the best run

% if useUnwieghtedBLR
%     
% else
%     costTrainingSampleTotCaseNums = randsample(validTotCaseNums, nMLTrainingSamples); % sample without replacement
%     costTrainingInitStateIDs = initStateIDs(costTrainingSampleTotCaseNums,1);
%     costTrainingFinalStateIDs = finalStateIDs(costTrainingSampleTotCaseNums,1);
%     costTrainingExamplesMat1 = stateMat(costTrainingInitStateIDs,:); % Stores the set of all initial points for 2PBVP's
%     costTrainingExamplesMat2 = stateMat(costTrainingFinalStateIDs,:); % Stores the set of all final points for 2PBVP's
%     cost_training_features   = deep_space_extract_2PBVP_features_cost_classifier( costTrainingExamplesMat1, costTrainingExamplesMat2);
%     cost_training_output   = Inf*ones(size(cost_training_features,1),1);
%     for i = 1:size(cost_training_features,1)
%         cost_training_output(i,1) = costMat(evalMat(costTrainingInitStateIDs(i,1),costTrainingFinalStateIDs(i,1)));
%     end
%     xExtent = max(cost_training_features) - min(cost_training_features);
% end


%% Reform Data Matrices to incorporate initial and goal data
stateMat = cat(1, NaN*ones(2, nStateDims), stateMat);
evalMat = cat(1, zeros(2,nSamples), evalMat);
evalMat = cat(2, zeros(nSamples+2, 2), evalMat);


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

%% Plot Results

% Trajectory
arrowScale = 20;
arrowHead= 5;
arrowThick = 2;
figure
hold on
if exist('obstacles', 'var')
    for i = 1:length(obstacles.cuboids.vertices)
        [Acol, bcol, ~, ~] = vert2lcon(obstacles.cuboids.vertices{i}');
        plotregion(-Acol, -bcol, [], [], [0.7,0.2,0.3],1);
    end
end

plot3(stateMat(1,1),stateMat(1,2),stateMat(1,3),'g^',...
    'MarkerFaceColor','g', 'MarkerSize', 10)
quiver3(stateMat(1,1),stateMat(1,2),stateMat(1,3),...
    stateMat(1,4),stateMat(1,5),stateMat(1,6),arrowScale, 'MaxHeadSize', arrowHead, 'LineWidth', arrowThick)
for i = 1:size(P,1)
    plot3(stateMat(P(i,2),1),stateMat(P(i,2),2),stateMat(P(i,2),3),'b*')
    quiver3(stateMat(P(i,2),1),stateMat(P(i,2),2),stateMat(P(i,2),3),...
        stateMat(P(i,2),4),stateMat(P(i,2),5),stateMat(P(i,2),6),arrowScale, 'MaxHeadSize', arrowHead, 'LineWidth', arrowThick)
    tempVar = reshape(trajMat(evalMat(P(i,1),P(i,2)),1:3,:),3,nTrajNodes)';
    plot3(tempVar(:,1),tempVar(:,2),tempVar(:,3),'k.-')
end
xlim(xyz_bounds(1,:))
ylim(xyz_bounds(2,:))
zlim(xyz_bounds(3,:))