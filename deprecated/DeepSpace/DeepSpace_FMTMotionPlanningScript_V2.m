% DeepSpace_FMTMotionPlanning_V2.m attempts to solve the kinodynamic motion
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
addpath([pwd, '/../MachineLearning/Reachability Classifier/']);
addpath([pwd, '/../../../../planningdata/GenKinoFMT/']);

%% Load Precomputed Data and set options
precompMatFile = 'DeepSpacePrecomputeV2_50x100x10_Mar11-2014.mat';
try
    load(precompMatFile);
catch
    disp('Invalid precomputed data file. Exiting...')
    return
end
disp('The information from the text files is done being imported')

% Initial and Final states (not "known" until online)
Xstart = [0 -40 0 0 0 0];      % (m,m,m,m/s,m/s,m/s) initial state
Xgoal = [0 40 -20 0 0 0];      % (m,m,m,m/s,m/s,m/s) goal state

% Machine Learning Parameters
neighborCostThreshold = NaN;        % (cost) radius of neighborhood  NOTE: this should be changed to be based off average cost
neighborCostQuantile = 0.1;         % Quntile of all cost data for cost threshold determination
% nMLTrainingSamples = 9500;            % (#) number of training examples for ML. NaN->use all samples
% nMLTestingSamples = 400;            % (#) number of training examples for ML. NaN->use all samples
nMLTrainingSamples = 90;            % (#) number of training examples for ML. NaN->use all samples
nMLTestingSamples = 5;            % (#) number of training examples for ML. NaN->use all samples
nAddSamples = 0;                  % (#) number of additional samples to run after ML training
train_options = svmset('MaxIter', 5e5, 'polyorder', 3,...);
      'kktviolationlevel', 0, 'boxconstraint', 0.01);

% Check Options
if nMLTrainingSamples+nMLTestingSamples > n2PBVPs || ...
        nMLTestingSamples <= 0
    disp('Invalid number of training and testing examples. Exiting...')
    return
end
    
 
% DeepSpace Optimal Control Parameters (NOTE some imported from precomp)
options.print_summary = false; % (boo)
options.plot_results = false;  % (b00)
options.storeExitFlags = true;
options.storeSolverOutput = true;
options.solver = optimset('Algorithm','sqp','GradObj','on',...
    'GradConstr','on','DerivativeCheck','off', 'Display', 'off',...
    'TolFun', 1e-2, 'TolCon', 5e-2, 'TolX', 1e-2);      % NOTE: These options can be tuned for fast solution 
options.solver.MaxIter = 10;

%% Generate Obstacles (don't access until online)
spaceTilingRes = 10;
xyz_bounds = config_space(1:3,:); % get from precomp
disp('----------------------')
disp('Generating ISS approximation of obstacles')
disp('----------------------')
DeepSpace_ISSObstacleScript;
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

% Check if initial or final states are immediately obstructed (in this is
% done to avoid wasting. In reality, it is impossible to start in an
% obstacle)
XstartCol = areStatesCollided(Xstart(1:3),obstacles, spaceTilingRes, xyz_bounds, false);
if XstartCol
    disp('Xstart is in collision with obstacle. Redefine problem')
    return
end
XgoalCol = areStatesCollided(Xgoal(1:3),obstacles, spaceTilingRes, xyz_bounds, false);
if XgoalCol
    disp('Xgoal is in collision with obstacle. Redefine problem')
    return
end


%% Calculate neighborhood cost threshold (NOTE: based on quantile of cost, different from V1)
if ~exist('neighborCostThreshold') || isnan(neighborCostThreshold)
    neighborCostThreshold = quantile(costMat, neighborCostQuantile);
end
disp(['Neighborhood Cost Threshold = ', num2str(neighborCostThreshold)])
disp('------------------------')

%% Determine valid total case numbers (should move to precompute)
validTotCaseNumsBool = zeros(nTot2PBVPs,1);
validTotCaseNums = [];
for i = 1:nTot2PBVPs
    if evalMat(initStateIDs(i,1), finalStateIDs(i,1)) > 0
        validTotCaseNumsBool(i,1) = 1;
        validTotCaseNums(end+1,1) = i;
    end
end
if length(validTotCaseNums) ~= n2PBVPs
    disp('There is an inconsistency between nTot2PBVPs and (valid) n2PBVPS.')
    disp('Diagnose before continuing. Exiting...')
    return
end

%% Train Machine Learning Algorithms
trainingSampleTotCaseNums = randsample(validTotCaseNums, nMLTrainingSamples); % sample without replacement(
trainingInitStateIDs = initStateIDs(trainingSampleTotCaseNums,1);
trainingFinalStateIDs = finalStateIDs(trainingSampleTotCaseNums,1);
[svm_output, n_training_errors, percent_training_errors] = ...
    DeepSpaceTrainSVM(stateMat, evalMat, costMat,...
    trainingInitStateIDs, trainingFinalStateIDs, ...
    neighborCostThreshold, @deep_space_extract_2PBVP_features, train_options);
disp(['Number of training errors = ', num2str(n_training_errors)]);
disp(['Training error percentage = ', num2str(percent_training_errors)]);

%% Test SVM Algorithm
testingSampleTotCaseNums = [1:nTot2PBVPs]';
testingSampleTotCaseNums(trainingSampleTotCaseNums) = []; % remove 2PBVPs used in training
testingSampleTotCaseNums(find(validTotCaseNumsBool(...
    testingSampleTotCaseNums,1)==0)) = []; % remove invalid 2PBVPs
testingSampleTotCaseNums = randsample(testingSampleTotCaseNums,...
    nMLTestingSamples); % sample without replacement
testingExamplesMat1 = stateMat(initStateIDs(testingSampleTotCaseNums,1),:); % Stores the set of all initial points for 2PBVP's
testingExamplesMat2 = stateMat(finalStateIDs(testingSampleTotCaseNums,1),:); % Stores the set of all final points for 2PBVP's
testing_reachability = zeros( nMLTestingSamples, 1 ); % Stores the reachability indicators (1 = reachable, -1 = unreachable)
for k = 1:nMLTestingSamples
    curTotCaseNum = testingSampleTotCaseNums(k,1);
    if costMat(evalMat(initStateIDs(curTotCaseNum,1),...
            finalStateIDs(curTotCaseNum,1))) <= neighborCostThreshold
        testing_reachability(k) = 1;    % reachable
    else 
        testing_reachability(k) = -1;   % unreachable
    end
end
testing_data   = deep_space_extract_2PBVP_features( testingExamplesMat1, testingExamplesMat2);
testing_predicted_reachability = svmclassify(svm_output, testing_data); % use svmclassify instead of is_reachable to output -1 or 1 to be consisten with training
n_testing_errors = nnz(testing_predicted_reachability - testing_reachability);
percent_testing_errors = 100*n_testing_errors/k;

disp(['Number of testing errors = ', num2str(n_testing_errors)]);
disp(['Testing error percentage = ', num2str(percent_testing_errors)]);
disp('------------------------')

%% Save/Load for NOTE debugging
save('tempFMTworkspace0')
% load('tempFMTworkspace0')

%% Solve 2PBVPs that were not solved in  precomp based on reachability prediction
disp('Solving remaining 2PBVPs based on predicted reachability')
disp('-----------------------------')
for i = 1:nSamples
    disp(i)
    for j = 1:nSamples
%         disp([num2str(i), ', ', num2str(j)])
        if i ~= j && evalMat(i,j) == 0 % non trivial and has not been evaluated
            curInitState = stateMat(i,:);
            curFinalState = stateMat(j,:);
            curReachPred = is_reachable(svm_output,...
                deep_space_extract_2PBVP_features(curInitState, curFinalState));
            if curReachPred == 1
                clear boundary_values
                boundary_values.x0 = curInitState(1);
                boundary_values.y0 = curInitState(2);
                boundary_values.z0 = curInitState(3);
                boundary_values.xdot0 = curInitState(4);
                boundary_values.ydot0 = curInitState(5);
                boundary_values.zdot0 = curInitState(6);
                boundary_values.t0 = 0;
                boundary_values.xf = curFinalState(1);
                boundary_values.yf = curFinalState(2);
                boundary_values.zf = curFinalState(3);
                boundary_values.xdotf = curFinalState(4);
                boundary_values.ydotf = curFinalState(5);
                boundary_values.zdotf = curFinalState(6);
                
                % Clear previous problem
                clear bvpinfo
                bvpinfo.numerics = numerics;
                bvpinfo.robot = robot;
                bvpinfo.boundary_values = boundary_values;
                bvpinfo.environment = environment;
                bvpinfo.options = options;
                % Call solver
                bvpinfo = DeepSpaceOptimizer(bvpinfo);
                % Record solution if feasible
                if bvpinfo.solution.exitflag > 0
                    n2PBVPs = n2PBVPs+1;
                    evalMat(i,j) = n2PBVPs;
                    costMat(n2PBVPs,1) = bvpinfo.solution.cost;
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
                end
            end
        end
    end
end

% Save/Load for NOTE debugging
save('tempFMTworkspace0')
% load('tempFMTworkspace0')

%% Reform Data Matrices to incorporate initial and goal data
stateMat = cat(1, NaN*ones(2, nStateDims), stateMat);
evalMat = cat(1, zeros(2,nSamples), evalMat);
evalMat = cat(2, zeros(nSamples+2, 2), evalMat);

%% Generate Neighborhoods (only for currently available samples)
outNeighborCell = cell(nSamples+2,1); % outgoing neighbors in ascending cost
inNeighborCell = cell(1,nSamples+2); % incoming neighbors in ascending cost
for i = 3:nSamples+2
    outNeighborIDs = find(evalMat(i,:) > 0)';
    outNeighborCosts = costMat(evalMat(i,outNeighborIDs));
    outNeighbors = [outNeighborIDs, outNeighborCosts];
    [~,sortIX] = sort(outNeighbors(:,2));
    outNeighbors = outNeighbors(sortIX,:);
    outNeighborCell{i,1} = outNeighbors';
    
    inNeighborIDs = find(evalMat(:,i) > 0);
    inNeighborCosts = costMat(evalMat(inNeighborIDs,i));
    inNeighbors = [inNeighborIDs, inNeighborCosts];
    [~,sortIX] = sort(inNeighbors(:,2));
    inNeighbors = inNeighbors(sortIX,:);
    inNeighborCell{1,i} = inNeighbors;
end

%% Going Online
disp('Going Online!')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%  Start Online Computation %%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Save/Load for NOTE debugging
% save('tempFMTworkspace1')
% % load('tempFMTworkspace1')
% tic
% %% Approximate neighborhood of initial and goal states with ML
% disp('Approximating Neighboorhoods of Initial and Goal States')
% stateMat(1,:) = Xstart;
% stateMat(2,:) = Xgoal;
% init_neighbors_feature_matrix  = deep_space_extract_2PBVP_features( ...
%     repmat(Xstart,nSamples+2,1), stateMat(1:end,:) );
% reachable_from_Xstart    = is_reachable( svm_output,...
%     init_neighbors_feature_matrix);
% goal_neighbors_feature_matrix = deep_space_extract_2PBVP_features( ...
%     stateMat(1:end,:), repmat(Xgoal,nSamples+2,1) );
% Xgoal_reachable_from = is_reachable(svm_output,...
%     goal_neighbors_feature_matrix);
% 
% %% Optimally connect initial and state to its approximate out-neighboorhood
% disp('Optimally Connecting Xstart to out-Neighborhood') % NOTE remove later
% clear boundary_values
% boundary_values.x0 = Xstart(1);
% boundary_values.y0 = Xstart(2);
% boundary_values.z0 = Xstart(3);
% boundary_values.xdot0 = Xstart(4);
% boundary_values.ydot0 = Xstart(5);
% boundary_values.zdot0 = Xstart(6);
% boundary_values.t0 = 0;
% outNeighbors = [];
% for j = 2:nSamples+2 
%     if reachable_from_Xstart(j) == 1
%         % Set boundary values
%         boundary_values.xf = stateMat(j,1);
%         boundary_values.yf = stateMat(j,2);
%         boundary_values.zf = stateMat(j,3);
%         boundary_values.xdotf = stateMat(j,4);
%         boundary_values.ydotf = stateMat(j,5);
%         boundary_values.zdotf = stateMat(j,6);
%         
%         % Clear previous problem
%         clear bvpinfo
%         bvpinfo.numerics = numerics;
%         bvpinfo.robot = robot;
%         bvpinfo.boundary_values = boundary_values;
%         bvpinfo.environment = environment;
%         bvpinfo.options = options;
%         
%         % Call Solver
%         bvpinfo = DeepSpaceOptimizer(bvpinfo);
%         
%         % Save Cost, Trajectory and Control and neighborhood
%         if bvpinfo.solution.exitflag > 0
%             n2PBVPs = n2PBVPs+1;
%             curCost = bvpinfo.solution.cost;
%             evalMat(1,j) = n2PBVPs;
%             costMat(n2PBVPs,1) = curCost;
%             trajMat(n2PBVPs,1,:) = bvpinfo.solution.x;
%             trajMat(n2PBVPs,2,:) = bvpinfo.solution.y;
%             trajMat(n2PBVPs,3,:) = bvpinfo.solution.z;
%             trajMat(n2PBVPs,4,:) = bvpinfo.solution.xdot;
%             trajMat(n2PBVPs,5,:) = bvpinfo.solution.ydot;
%             trajMat(n2PBVPs,6,:) = bvpinfo.solution.zdot;
%             controlMat(n2PBVPs,1,:) = bvpinfo.solution.ux;
%             controlMat(n2PBVPs,2,:) = bvpinfo.solution.uy;
%             controlMat(n2PBVPs,3,:) = bvpinfo.solution.uz;
%             controlMat(n2PBVPs,4,:) = bvpinfo.solution.eta;
%             outNeighbors = [outNeighbors; [j curCost]];
%             if isempty(inNeighborCell{1,j})
%                 inNeighborCell{1,j}(1,:) = [1 curCost];
%             else
%                 inPos = find(inNeighborCell{1,j}(:,2) > curCost,1,'first');
%                 inNeighborCell{1,j} = [inNeighborCell{1,j}(1:inPos,:);...
%                     inNeighborCell{1,j}(inPos:end,:)];
%                 inNeighborCell{1,j}(1,:) = [1 curCost];
%             end
%         end
%     end
% end
% [~,sortIX] = sort(outNeighbors(:,2));
% outNeighbors = outNeighbors(sortIX,:);
% outNeighborCell{1,1} = outNeighbors';
% 
% 
% %% Optimally connect goal state to its approximate in-neighboorhood
% disp('Optimally Connecting Xgoal to in-Neighborhood')
% clear boundary_values
% boundary_values.xf = Xgoal(1);
% boundary_values.yf = Xgoal(2);
% boundary_values.zf = Xgoal(3);
% boundary_values.xdotf = Xgoal(4);
% boundary_values.ydotf = Xgoal(5);
% boundary_values.zdotf = Xgoal(6);
% boundary_values.t0 = 0;
% inNeighbors = [];
% for i = [1, 3:nSamples+2]
%     if Xgoal_reachable_from(i) == 1
%         % Set boundary values
%         boundary_values.x0 = stateMat(i,1);
%         boundary_values.y0 = stateMat(i,2);
%         boundary_values.z0 = stateMat(i,3);
%         boundary_values.xdot0 = stateMat(i,4);
%         boundary_values.ydot0 = stateMat(i,5);
%         boundary_values.zdot0 = stateMat(i,6);
%         
%         % Clear previous problem
%         clear bvpinfo
%         bvpinfo.numerics = numerics;
%         bvpinfo.robot = robot;
%         bvpinfo.boundary_values = boundary_values;
%         bvpinfo.environment = environment;
%         bvpinfo.options = options;
%         
%         % Call Solver
%         bvpinfo = DeepSpaceOptimizer(bvpinfo);
%         
%         % Save Cost, Trajectory and Control
%         if bvpinfo.solution.exitflag > 0
%             n2PBVPs = n2PBVPs+1;
%             curCost = bvpinfo.solution.cost;
%             evalMat(i,2) = n2PBVPs;
%             costMat(n2PBVPs,1) = curCost;
%             trajMat(n2PBVPs,1,:) = bvpinfo.solution.x;
%             trajMat(n2PBVPs,2,:) = bvpinfo.solution.y;
%             trajMat(n2PBVPs,3,:) = bvpinfo.solution.z;
%             trajMat(n2PBVPs,4,:) = bvpinfo.solution.xdot;
%             trajMat(n2PBVPs,5,:) = bvpinfo.solution.ydot;
%             trajMat(n2PBVPs,6,:) = bvpinfo.solution.zdot;
%             controlMat(n2PBVPs,1,:) = bvpinfo.solution.ux;
%             controlMat(n2PBVPs,2,:) = bvpinfo.solution.uy;
%             controlMat(n2PBVPs,3,:) = bvpinfo.solution.uz;
%             controlMat(n2PBVPs,4,:) = bvpinfo.solution.eta;
%             inNeighbors = [inNeighbors; [i curCost]];
%             if isempty(outNeighborCell{i,1})
%                 outNeighborCell{i,1}(:,1) = [2; curCost];
%             else
%                 outPos = find(outNeighborCell{i,1}(2,:) > curCost,1,'first');
%                 outNeighborCell{i,1} = [outNeighborCell{i,1}(:,1:outPos), outNeighborCell{i,1}(:,outPos:end)];
%                 outNeighborCell{i,1}(:,outPos) = [2; curCost];
%             end
%         end
%     end 
% end
% [~,sortIX] = sort(inNeighbors(:,2));
% inNeighbors = inNeighbors(sortIX,:);
% inNeighborCell{1,2} = inNeighbors;
% 
% 
% %% Save/Load workspace for NOTE debugging
% save('tempFMTworkspace2')
% % load('tempFMTworkspace2')
% 
% %% Begin Fast Marching Trees Algorithm
% disp('Running FMT Algorithm')
% P = [];                     % Path from Xstart to Xgoal
% V = 1:nSamples+2;           % set of all states
% Vgoal = 2;                  % goal state(s)
% E = [];                     % edges
% W = V;                      % states not in tree
% W(1) = [];                  % remove Xstart
% H = 1;                      % states in tree (just Xstart)
% z = 1;                      % current state
% NzOut = outNeighborCell{z,1};     % set of nearest outgoing neighbors
% cost2Come = zeros(1,nSamples+2);    % cost to arrive at each state
% 
% while isempty(intersect(Vgoal, z))
%     Hnew = [];
%     Xnear = intersect(NzOut, W);        %hmmm
%     for j = 1:length(Xnear)
%         x = Xnear(j);
%         NxIn = inNeighborCell{1,x};
%         Ynear = intersect(NxIn, H);     %hmmmm
%         yMin = -1; 
%         xMinCost2Come = inf;
%         for i=1:length(Ynear)
%             y = Ynear(i);
%             yxCost2Come = cost2Come(y) + costMat(evalMat(y,x),1);
%             if yxCost2Come <= xMinCost2Come
%                 yMin = y;
%                 xMinCost2Come = yxCost2Come;
%             end
%         end
%         
%         % Check Collision
%         nodes2check = reshape(trajMat(evalMat(y,x),1:3,:),3,nTrajNodes)'; 
%         collision = areStatesCollided(nodes2check,obstacles,spaceTilingRes,xyz_bounds,true);
%         if collision % change costMat
%             costMat(evalMat(y,x),1) = Inf;
%         end
%         
%         if ~collision
%             E = [E; yMin x];
%             Hnew = [Hnew x];
%             W(find(W==x,1)) = [];
%             cost2Come(x) = xMinCost2Come; %hmmmmm
%         end
%     end
%     H = [H Hnew];
%     H(find(H==z,1)) = [];
%     if isempty(H)
%         disp('FMT Fail: Set H has emptied before Xgoal reached')
%         return
%     end
%     [~, zind] = min(cost2Come(H));
%     z = H(zind);
%     NzOut = outNeighborCell{z,1};
% end
% 
% P = FMTPath(1, Vgoal, E, cost2Come);
% toc
% save('tempFMTworkspace3')
% 
% %% Plot Results
% 
% % Trajectory
% arrowScale = 20;
% arrowHead= 5;
% arrowThick = 2;
% figure
% hold on
% for i = 1:length(obstacles.cuboids.vertices)  
%     [Acol, bcol, ~, ~] = vert2lcon(obstacles.cuboids.vertices{i}');
%     plotregion(-Acol, -bcol, [], [], [0.7,0.2,0.3],1);
% end
% plot3(stateMat(1,1),stateMat(1,2),stateMat(1,3),'g^',...
%     'MarkerFaceColor','g', 'MarkerSize', 10)
% quiver3(stateMat(1,1),stateMat(1,2),stateMat(1,3),...
%     stateMat(1,4),stateMat(1,5),stateMat(1,6),arrowScale, 'MaxHeadSize', arrowHead, 'LineWidth', arrowThick)
% for i = 1:size(P,1)
%     plot3(stateMat(P(i,2),1),stateMat(P(i,2),2),stateMat(P(i,2),3),'b*')
%     quiver3(stateMat(P(i,2),1),stateMat(P(i,2),2),stateMat(P(i,2),3),...
%         stateMat(P(i,2),4),stateMat(P(i,2),5),stateMat(P(i,2),6),arrowScale, 'MaxHeadSize', arrowHead, 'LineWidth', arrowThick)
%     tempVar = reshape(trajMat(evalMat(P(i,1),P(i,2)),1:3,:),3,nTrajNodes)';
%     plot3(tempVar(:,1),tempVar(:,2),tempVar(:,3),'k.-')
% end
% xlim(xyz_bounds(1,:))
% ylim(xyz_bounds(2,:))
% zlim(xyz_bounds(3,:))