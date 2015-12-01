%DSSCParaMotionPlannerFinal solves the final steps of the deep space space 
%   spaccraft (DSSC) motion planning problem
%
%   Ross Allen, ASL, Stanford University
%
%   Started:        Sep 23, 2014
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
%       - System dynamics are based on a point-mass spacecraft in a 6D
%       state space (x,y,z,vx,vy,vz) with control space being the thrust
%       vector which can point arbitrarily, yet magnitude limited. 
%       Mass change assumed negligible
%       - This function completes the offline steps after the remaining
%       2PBVPs are solved and then solves the online portion
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [mpinfo, planningPhase] = DSSCParaMotionPlannerFinal(mpinfo, planningPhase)

%% Offline Phase

% Ensure that proper path variables are established
addpath([pwd, '/../']);
addpath([pwd, '/../MachineLearning/']);
addpath([pwd, '/../MachineLearning/Reachability Classifier/']);
addpath([pwd, '/../ObstacleSets/']);
addpath([pwd, '/../../../../planningdata/GenKinoFMT/']);

% Check for Valid Inputs
if planningPhase < 5
    disp('Error occured in input. Data not prepared for final steps')
    disp('planningPhase < 5')
    disp('Exiting DSSCParaMotionPlanningFinal prematurely ...')
    return;
end

% Train Cost Estimator
if planningPhase <= 5
    disp('----------------------------------------')
    disp('Training Cost Estimator')
    mpinfo = DSSCTrainCostEstimator(mpinfo);
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
        [mpinfo.termStates.nGoalSamples+2:mpinfo.nTotSamples]);
    mpinfo.progress.SortNeighborhoods = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
    disp('----------------------------------------')
end

% Introduce Obstacles
if planningPhase <= 8
    disp('----------------------------------------')
    disp('Generating Obstacle Set')
    mpinfo = DSSCGetCuboidObstacles(mpinfo);
    mpinfo.progress.IntroduceObstacles = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
    disp('----------------------------------------')
end


%% Online Phase

disp('----------------------------------------')
disp('Going Online!')
disp('----------------------------------------')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%  Start Online Computation %%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tic

% Introduce start state
mpinfo.stateMat(1,:) = mpinfo.termStates.Xstart;

% Sample Goal Region
mpinfo = GoalRegionSampler(mpinfo);
mpinfo.stateMat(2:mpinfo.termStates.nGoalSamples+1,:) = mpinfo.termStates.Xgoal;

% Classify neighborhoods of start and goal states
mpinfo = ClassifyTerminalNeighborhoods(mpinfo);

% Approximate cost of outgoing neighborhood of start state
 mpinfo.termStates.startEstOutNeighbors = ...
    Estimate2PBVPCosts(mpinfo.termStates.reachableFromXstart,...
    1, [1:mpinfo.nTotSamples]', mpinfo);

% Approximate cost of incoming neighborhoods of goal region
mpinfo.termStates.goalEstInNeighbors = cell(...
    mpinfo.termStates.nGoalSamples, 1);
for i = 1:mpinfo.termStates.nGoalSamples
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

% Optimally connect goal region samples to approximate in-neighboorhoods
for i = 2:mpinfo.termStates.nGoalSamples+1
    mpinfo = Solve2PBVPsNeighborhoodSet(...
        mpinfo.termStates.goalEstInNeighbors{i-1}(:,1),...
        i*ones(size(mpinfo.termStates.goalEstInNeighbors{i-1},1),1),...
        mpinfo.onlineOptions.maxNeighbors, mpinfo);
end

% Run Kinodynamic Fast Marching Trees
mpinfo = RunKinoFMT(mpinfo);

mpinfo.onlineCompTime = toc

planningPhase = planningPhase + 1;

save(mpinfo.savefile);
end


