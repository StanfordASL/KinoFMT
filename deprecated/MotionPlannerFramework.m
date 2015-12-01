%MotionPlannerFramework solves a motion planning problem for a general system
%
%   Ross Allen, ASL, Stanford University
%
%   Started:        Feb 3, 2015
%
%   Inputs:
%
%   Outputs:
%
%   Notes:
%       - Matlab 2012b or later should be used as certain Matlab functions
%       experience adverse 'legacy' effects that will cause erros if older
%       versions are used (e.g. svmtrain, intersect, ismember, etc)
%       - 'mpinfo' stands for motion planning information and holds all
%       relevant data for a motion planning problem (i.e. sampling info, robot
%       info, machine learning info, etc.)
%       - This has been generalized to a system-agnostic function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [mpinfo] = MotionPlannerFramework(mpinfo, planningPhase)

%% Offline Phase

% Ensure that proper path variables are established
addpath([pwd, '/../MachineLearning/']);
addpath([pwd, '/../MachineLearning/Reachability Classifier/']);
addpath([pwd, '/../ObstacleSets/']);
addpath([pwd, '/../../../../planningdata/GenKinoFMT/']);

% Perform initial input error checks and retrieve system definitions
if planningPhase <= 0
	% TODO: Add input error check
    mpinfo.inputError = InitialErrorCheck(mpinfo);
    if mpinfo.inputError.flag > 0
        disp('User input invalid. Check inputError code');
        return;
    end
    mpinfo.progress.InitialErrorCheck = true;
    mpinfo.systemDefs = mpinfo.definitionsFunction();
    mpinfo.progress.RetrieveSystemDefs = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
end

% Sample State Space
if planningPhase  <= 1
    disp('----------------------------------------')
    disp('Sampling State Space')
    mpinfo = StateSampler(mpinfo);
    mpinfo.progress.SampleStateSpace = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
    disp('----------------------------------------')
end

% Solve Initial Set of 2PBVPs
if planningPhase <= 2
    disp('----------------------------------------')
    disp('Solving Initial Set of Optimal 2PBVPs')
    mpinfo = Solve2PBVPsInitialSet(mpinfo);
    mpinfo.progress.SolveTraining2PBVPs = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
    save([mpinfo.savefile, '_phase', num2str(planningPhase-1)]);
    disp('----------------------------------------')
end

% Train SVM Neighborhood Classifier
if planningPhase <= 3
    disp('----------------------------------------')
    disp('Training SVM Reachability Classifier')
    mpinfo = TrainNeighborClassifier(mpinfo);
    mpinfo.progress.TrainSVM = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
    disp('----------------------------------------')
end

% Solve Remaining 2PBVPs Based on Classification
if planningPhase <= 4
    disp('----------------------------------------')
    disp('Solving Remaining 2PBVPs Based on Predicted Reachability')
    mpinfo = Solve2PBVPsRemainingSet(mpinfo);
    mpinfo.progress.SolveRemaining2PBVPs = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
    save([mpinfo.savefile, '_phase', num2str(planningPhase-1)]);
    disp('----------------------------------------')
end

% Train Cost Estimator
if planningPhase <= 5
    disp('----------------------------------------')
    disp('Training Cost Estimator')
    mpinfo = TrainCostEstimator(mpinfo);
    mpinfo.progress.TrainCostEstimator = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
    disp('----------------------------------------')
end

% Reform Matrices to Accomondate Initial and Goal States
if planningPhase <= 6
    disp('----------------------------------------')
    disp('Reforming Data Matrices to Incorporate Start and Goal')
    mpinfo = ReformDataMatrices(mpinfo);
    mpinfo.progress.ReformMatrices = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
    disp('----------------------------------------')
end

% Sort Neighborhoods for current samples
if planningPhase <= 7
    disp('----------------------------------------')
    disp('Sorting Neighborhoods')
    mpinfo = SortNeighborhoods(mpinfo,...
        [mpinfo.sampling.nGoalSamples+2:mpinfo.nTotSamples]);
    mpinfo.progress.SortNeighborhoods = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
    disp('----------------------------------------')
end

% Introduce Obstacles
if planningPhase <= 8
    disp('----------------------------------------')
    disp('Generating Obstacle Set')
    mpinfo = GetCuboidObstacles(mpinfo);
    mpinfo.progress.IntroduceObstacles = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
    disp('----------------------------------------')
end

save([mpinfo.savefile, '_precomputed']);


%% Online Phase

disp('----------------------------------------')
disp('Going Online!')
disp('----------------------------------------')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%  Start Online Computation %%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (isfield(mpinfo,'profiler') && mpinfo.profiler == true)
    profile on
end

tic

% Perform online error check
mpinfo.onlineError = OnlineErrorCheck(mpinfo);
if mpinfo.inputError.flag > 0
    disp('User input invalid at online initiation. Check onlineError code');
    return;
end

% Introduce start state
mpinfo.stateMat(1,:) = mpinfo.termStates.Xstart;

% Sample Goal Region
mpinfo = GoalRegionSampler(mpinfo);
mpinfo.stateMat(2:mpinfo.sampling.nGoalSamples+1,:) = mpinfo.termStates.Xgoal;

% Classify neighborhoods of start and goal states
mpinfo = ClassifyTerminalNeighborhoods(mpinfo);

% Approximate cost of outgoing neighborhood of start state
mpinfo.termStates.startEstOutNeighbors = ...
    Estimate2PBVPCosts(mpinfo.termStates.reachableFromXstart,...
    1, [1:mpinfo.nTotSamples]', mpinfo);

% Approximate cost of incoming neighborhoods of goal region
mpinfo.termStates.goalEstInNeighbors = cell(...
    mpinfo.sampling.nGoalSamples, 1);
for i = 1:mpinfo.sampling.nGoalSamples
 mpinfo.termStates.goalEstInNeighbors{i} = ...
    Estimate2PBVPCosts(mpinfo.termStates.XgoalReachableFrom(:,i),...
    [1:mpinfo.nTotSamples]', i, mpinfo);
end
clear i

% Optimally connect start and state to its approximate out-neighboorhood
mpinfo = Solve2PBVPsNeighborhoodSet(...
    ones(size(mpinfo.termStates.startEstOutNeighbors,1),1),...
    mpinfo.termStates.startEstOutNeighbors(:,1),...
    mpinfo.onlineOptions.maxNeighbors, mpinfo);
if isempty(mpinfo.outNeighborCell{1})
    disp('FAILURE: No neighbors found for Xstart');
    return;
end

% Optimally connect goal region samples to approximate in-neighboorhoods
for i = 2:mpinfo.sampling.nGoalSamples+1
    mpinfo = Solve2PBVPsNeighborhoodSet(...
        mpinfo.termStates.goalEstInNeighbors{i-1}(:,1),...
        i*ones(size(mpinfo.termStates.goalEstInNeighbors{i-1},1),1),...
        mpinfo.onlineOptions.maxNeighbors, mpinfo);
end
clear i

% Run Kinodynamic Fast Marching Trees
mpinfo = RunKinoFMT(mpinfo);

if ~isnan(mpinfo.optimalCost)
    % Run smoothing function if it exist
    if (isfield(mpinfo.systemDefs, 'TrajectorySmoother') && ...
        isfield(mpinfo, 'smoother') && mpinfo.smoother.applySmoothing)
        mpinfo = mpinfo.systemDefs.TrajectorySmoother(mpinfo);
    end
    
    mpinfo.onlineCompTime = toc;
    
    % Write data to communication file for transmission to robot
    if (isfield(mpinfo,'comms') && mpinfo.comms.xmit)
       mpinfo.systemDefs.CommsWriter(mpinfo); 
    end
end

if (isfield(mpinfo,'profiler') && mpinfo.profiler == true)
    profile viewer
    profsave(profile('info'), [mpinfo.savefile, '_profiler'])
end

% Save data
save([mpinfo.savefile, '_complete']);

end

