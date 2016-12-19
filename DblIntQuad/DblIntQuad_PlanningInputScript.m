% DblIntQuad_PlanningInputScript.m accepts inputs and calls the 
%   functionality of machine learning-guided, FMT planning for a 
%   double integrator model for a quadrotor system. Specifically
%   this model uses a control-effort-penalizing cost function
%   (as opposed to a control constrained formulation).
%
%   Ross Allen, ASL, Stanford University
%   Apr 28, 2015
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
addpath([pwd, '/../AgileQuad/']);   % smoother


%% Set Motion Planning Options

% Data Storage
savefile = '../../../../planningdata/GenKinoFMT/DblIntQuad_20151028_200x2000x10';
% Note on naming scheme: 
%       (system)_(nSamples)x(nTot2PBVPs)x(nTrajNodes)_(YYYY)-(MM)-(DD)
profiler = false;   % (boo) set if code profiler should be run

% Sampling Options
sampling.nSamples = 200;      %(#) number of sampled nodes in state space
sampling.nTot2PBVPs = 2000;	%(#) number of 2pt BVPs to solve for trainng ML
sampling.nTrajNodes = 10;	%(#) number of disctrete points for optimal control subproblems
sampling.stateSampleRange = [
                    -0.5    2.5;...        %(m m) range to sample x
                    0	    4.0;...        %(m m) range to sample y
                    -2.0	-1.0;...        %(m m) range to sample z
                   	-1.0 1.0;...        %(m/s) range to sample vx
                    -1.0 1.0;...        %(m/s) range to sample vy
                    -1.0 1.0];          %(m/s) range to sample vz  
sampling.nGoalSamples = 1;

% Robot Parameters
robot.mass = 0.9574;           %(kg) quadrotor mass
robot.thrustMax = 30;       %(N) maximum thrust
robot.maxPitchRoll = 60;      %(deg) maximum angular devation from flat

% Environment Bounds
environment.bounds = [  -0.5    2.8; ...
                        0.14    4.25; ...
                        -2.75   -0.2];

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
offlineOptions.tf_min = 1e-3;   % (s) lower bound of time interval
offlineOptions.tf_max = 100;    % (s) upper bound of time interval
offlineOptions.controlPenaltyWeight = 0.1; % penalty on control effort

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
onlineOptions.tf_min = 1e-3;    % (s) lower bound of time interval
onlineOptions.tf_max = NaN;     % (s) upper bound of time interval
onlineOptions.controlPenaltyWeight = 0.1; % penalty on control effort

% Machine Learn Options
learning.neighborCostThreshold = NaN; 	% (cost) radius of neighborhood  NOTE: this should be changed to be based off average cost
learning.neighborCostQuantile = 0.25;  	% Quntile of all cost data for cost threshold determination
learning.nMLTrainingSamples = 1900;      	% (#) number of training examples for ML. NaN->use all samples
learning.nMLTestingSamples = NaN;       	% (#) number of testing examples for ML. NaN-> use all samples not used in testing
learning.neighbor.featureSet = @DblIntQuadFeatureSetNeighbor;
learning.neighbor.train_options = svmset(...
    'MaxIter', 5e5,...
    'polyorder', 3,...
    'kktviolationlevel', 0,...
    'boxconstraint', 0.01);
learning.cost.nTrainingsRuns = 5;            % (#) number of rounds of cost estimator training
learning.cost.blr_taus = [];        % (#) weighting factor to test for batched linear regression
learning.cost.featureSet = @DblIntQuadFeatureSetCost;

% Obstacles
obstacles.obstacleSpace = sampling.stateSampleRange(1:3,:); % dimensions and range of dimensions where obstacles may exist
obstacles.generationFunction = @IndoorObstacleSet4; % function to call for pre-generated, hand-coded obstacles
obstacles.nPassiveObs = NaN;                                 % number of obstacles (only if generation function requires)
obstacles.minCuboidDimension = 1.0;                         % minimum dimension of a cuboid(only if generation function requires)
obstacles.maxCuboidDimension = 2.0;                         % number of obstacles (only if generation function requires)

% Start state and Goal region
termStates.Xstart = [1.5 0.25 -1.5 0 0 0];      % (m,m,m,m/s,m/s.m/s) initial state
termStates.goalRegion = ...
    [1.25  1.25;...           % (m) x-range in goal region
     0.5  0.5;...          % (m) y-range in goal region 
    -1.5 -1.5;...         % (m) z-range in goal region 
     0  0;...           % (m/s) vx-range in goal region
     0  0;...           % (m/s) vy-range in goal region
     0  0];             % (m/s) vz-range in goal region

% system definition function
definitionsFunction = @DblIntQuadDefinitions;

% Smoother options
smoother.applySmoothing = true; % (bool) boolean to apply smoothing
smoother.yaw = -1.68;    % (rad) constant yaw to be held 
smoother.nCheckNodes = 10;
smoother.timeScaling = 1.1;
smoother.flatPath = false;  % compress 3D trajectory to 2D
% TrajectorySmoother set in systemDefs

% Communication options
% comms.commpath = '/../../AgileQuad/ViconWifiComm/
comms.commfile = '../../AgileQuad/ViconWifiComm/Debug/test.txt';
comms.xmitTrajectory = false;
% CommsWriter set in systemDefs

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
mpinfo.smoother = smoother;
mpinfo.comms = comms;
clear sampling robot environment offlineOptions learning obstacles
clear termStates savefile profiler onlineOptions definitionsFunction
clear smoother comms


% Call Planner
mpinfo = MotionPlannerFramework(mpinfo, 0);

