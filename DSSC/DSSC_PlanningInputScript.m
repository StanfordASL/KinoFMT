% DSSC_PlanningInputScript.m executes offline and online components of
% machine learning-guided, FMT planning
%
%   Ross Allen, ASL, Stanford University
%   Sep 24, 2014
%
% NOTES:
%   - 'mpinfo' stands for motion planning information and holds all
%   relevant data for a motion planning problem (i.e. sampling info, robot
%   info, machine learning info, etc.)
%   - 'system' refers to the robot, dynamics, cost function, etc. combo. It
%   is closely related to 'probinfo' used in a single optimizing problem
%   (2PBVP)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

s=dbstatus;
save('myBreakpoints.mat', 's');
clear all
load('myBreakpoints.mat');
dbstop(s);
clc
close all
addpath([pwd, '/../']);
addpath([pwd, '/../MachineLearning/']);
addpath([pwd, '/../MachineLearning/Reachability Classifier/']);
addpath([pwd, '/../ObstacleSets/']);
addpath([pwd, '/../../../../planningdata/GenKinoFMT/']);


%% Set Motion Planning Options

% Data Storage
savefile = '../../../../planningdata/GenKinoFMT/DSSCTiming_5000x1000x10_Sep29-2014';

% Sampling Options
sampling.nSamples = 5000;      %(#) number of sampled nodes in state space
sampling.stateSampleRange = [
                    -100 100;...        %(m m) range to sample x
                    -100 100;...        %(m m) range to sample y
                    -100 100;...        %(m m) range to sample z
                   	-1.0 1.0;...        %(m/s) range to sample vx
                    -1.0 1.0;...        %(m/s) range to sample vy
                    -1.0 1.0];          %(m/s) range to sample vz  
sampling.nTot2PBVPs = 1000;	%(#) number of 2pt BVPs to solve for trainng ML
sampling.nTrajNodes = 10;	%(#) number of disctrete points for optimal control subproblems

% Robot Parameters
robot.mass = 1.0;           %(kg) mass of craft
robot.thrustMax = 0.1;      %(N) maximum thrust magnitude

% Environment Bounds
environment = [];

% Solver options for offline optimization problems (2PBVPs)
offlineOptions.storeExitFlags = true;
offlineOptions.storeSolverOutput = true;
offlineOptions.solver = optimset(...
    'Algorithm','sqp',...
    'GradObj','on',...
    'GradConstr','on',...
    'DerivativeCheck','off',...
    'Display', 'off',...
    'TolFun', 1e-3,...
    'TolCon', 1e-3,...
    'TolX', 1e-3);
offlineOptions.solver.MaxIter = 100;

% Solver options for online optimization problems (2PBVPs)
onlineOptions.maxNeighbors = 10;
onlineOptions.storeExitFlags = true;
onlineOptions.storeSolverOutput = true;
onlineOptions.solver = optimset(...
    'Algorithm','sqp',...
    'GradObj','on',...
    'GradConstr','on',...
    'DerivativeCheck','off',...
    'Display', 'off',...
    'TolFun', 1e-2,...
    'TolCon', 1e-2,...
    'TolX', 1e-2);
onlineOptions.solver.MaxIter = 10;

% Machine Learn Options
learning.neighborCostThreshold = NaN; 	% (cost) radius of neighborhood  NOTE: this should be changed to be based off average cost
learning.neighborCostQuantile = 0.5;  	% Quntile of all cost data for cost threshold determination
learning.nMLTrainingSamples = 850;      	% (#) number of training examples for ML. NaN->use all samples
learning.nMLTestingSamples = NaN;       	% (#) number of testing examples for ML. NaN-> use all samples not used in testing
learning.neighbor.featureSet = @DSSCFeatureSetNeighbor;
learning.neighbor.train_options = svmset(...
    'MaxIter', 5e5,...
    'polyorder', 3,...
    'kktviolationlevel', 0,...
    'boxconstraint', 0.01);
learning.cost.nTrainingsRuns = 5;            % (#) number of rounds of cost estimator training
learning.cost.blr_taus = [];        % (#) weighting factor to test for batched linear regression
learning.cost.featureSet = @DSSCFeatureSetCost;

% Obstacles
obstacles.obstacleSpace = sampling.stateSampleRange(1:3,:); % dimensions and range of dimensions where obstacles may exist
obstacles.spaceTilingRes = 50;                              % number of partitions to separate sample space
obstacles.generationFunction = @ISSObstacleGenerator;       % function to call for pre-generated, hand-coded obstacles
% obstacles.generationFunction = @ISSObstacleGenerator;       % function to call for pre-generated, hand-coded obstacles
% obstacles.generationFunction = @SimpleCuboidGenerator;       % function to call for pre-generated, hand-coded obstacles
obstacles.nObstacles = NaN;                                 % number of obstacles (only if generation function requires)
obstacles.minCuboidDimension = NaN;                         % minimum dimension of a cuboid(only if generation function requires)
obstacles.maxCuboidDimension = NaN;                         % number of obstacles (only if generation function requires)

% Start state and Goal region
termStates.Xstart = [0 -40 0 0 0 0];      % (m,m,m,m/s,m/s.m/s) initial state
termStates.goalRegion = ...
    [0  0;...           % (m) x-range in goal region
     40 40;...          % (m) y-range in goal region 
    -20 -20;...         % (m) z-range in goal region 
     0  0;...           % (m/s) vx-range in goal region
     0  0;...           % (m/s) vy-range in goal region
     0  0];             % (m/s) vz-range in goal region
termStates.nGoalSamples = 1;


% Consolidate Information
mpinfo.savefile = savefile;
mpinfo.sampling = sampling;
mpinfo.robot = robot;
mpinfo.environment = environment;
mpinfo.offlineOptions = offlineOptions;
mpinfo.onlineOptions = onlineOptions;
mpinfo.learning = learning;
mpinfo.obstacles = obstacles;
mpinfo.termStates = termStates;
clear sampling robot environment offlineOptions learning obstacles
clear termStates savefile onlineOptions


% Call Planner
mpinfo = DSSCMotionPlanner(mpinfo, 0);
