% SimpleQuad_PlanningInputScript.m executes offline and online components of
% machine learning-guided, FMT planning for a quadcopter model
%
%   Ross Allen, ASL, Stanford University
%   Feb 3, 2015
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
savefile = '../../../../planningdata/GenKinoFMT/SimpleQuad_100x2000x10_2015-04-08';
profiler = true;

% Sampling Options
sampling.nSamples = 100;      %(#) number of sampled nodes in state space
sampling.stateSampleRange = [
                    0 	5;...        %(m m) range to sample x
                    0	5;...        %(m m) range to sample y
                    -5	0;...        %(m m) range to sample z
                   	-1.0 1.0;...        %(m/s) range to sample vx
                    -1.0 1.0;...        %(m/s) range to sample vy
                    -1.0 1.0];          %(m/s) range to sample vz  
sampling.nTot2PBVPs = 2000;	%(#) number of 2pt BVPs to solve for trainng ML
sampling.nTrajNodes = 10;	%(#) number of disctrete points for optimal control subproblems

% Robot Parameters
robot.mass = 2.0;           %(kg) quadrotor mass
robot.thrustMax = 60;       %(N) maximum thrust
robot.maxPitchRoll = 45;      %(deg) maximum angular devation from flat

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
learning.neighborCostQuantile = 0.25;  	% Quntile of all cost data for cost threshold determination
learning.nMLTrainingSamples = 1500;      	% (#) number of training examples for ML. NaN->use all samples
learning.nMLTestingSamples = NaN;       	% (#) number of testing examples for ML. NaN-> use all samples not used in testing
learning.neighbor.featureSet = @SimpleQuadFeatureSetNeighbor;
learning.neighbor.train_options = svmset(...
    'MaxIter', 5e5,...
    'polyorder', 3,...
    'kktviolationlevel', 0,...
    'boxconstraint', 0.01);
learning.cost.nTrainingsRuns = 5;            % (#) number of rounds of cost estimator training
learning.cost.blr_taus = [];        % (#) weighting factor to test for batched linear regression
learning.cost.featureSet = @SimpleQuadFeatureSetCost;

% Obstacles
obstacles.obstacleSpace = sampling.stateSampleRange(1:3,:); % dimensions and range of dimensions where obstacles may exist
obstacles.generationFunction = @SingleCentralCuboid;       % function to call for pre-generated, hand-coded obstacles
obstacles.nObstacles = NaN;                                 % number of obstacles (only if generation function requires)
obstacles.minCuboidDimension = 0.5;                         % minimum dimension of a cuboid(only if generation function requires)
obstacles.maxCuboidDimension = 1.0;                         % number of obstacles (only if generation function requires)

% Start state and Goal region
termStates.Xstart = [4 4 -1.0 0 0 0];      % (m,m,m,m/s,m/s.m/s) initial state
termStates.goalRegion = ...
    [1  1;...           % (m) x-range in goal region
     1  1;...          % (m) y-range in goal region 
    -2.5 -2.5;...         % (m) z-range in goal region 
     0  0;...           % (m/s) vx-range in goal region
     0  0;...           % (m/s) vy-range in goal region
     0  0];             % (m/s) vz-range in goal region
termStates.nGoalSamples = 1;

% system definition function
definitionsFunction = @SimpleQuadDefinitions;

% Consolidate Information
mpinfo.savefile = savefile;
mpinfo.profiler = profiler;
mpinfo.sampling = sampling;
mpinfo.robot = robot;
mpinfo.environment = environment;
mpinfo.offlineOptions = offlineOptions;
mpinfo.onlineOptions = onlineOptions;
mpinfo.learning = learning;
mpinfo.obstacles = obstacles;
mpinfo.termStates = termStates;
mpinfo.definitionsFunction = definitionsFunction;
clear sampling robot environment offlineOptions learning obstacles
clear termStates savefile profiler onlineOptions definitionsFunction


% Call Planner
mpinfo = MotionPlannerFramework(mpinfo, 0);
