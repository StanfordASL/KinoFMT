% DblIntQuad_PrecomputeScript.m accepts inputs and calls the 
%   precomputation phase of the machine learning-guided, 
%   FMT planning for a 
%   double integrator model for a quadrotor system. Specifically
%   this model uses a control-effort-penalizing cost function
%   (as opposed to a control constrained formulation).
%
%   Ross Allen, ASL, Stanford University
%   Jun 26, 2015
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
savefile = '../../../../planningdata/GenKinoFMT/DblIntQuad_20151112_1000x50000x5';
% Note on naming scheme: 
%       (system)_(nSamples)x(nTot2PBVPs)x(nTrajNodes)_(YYYY)-(MM)-(DD)

% Sampling Options
sampling.nSamples = 1000;      %(#) number of sampled nodes in state space
sampling.nGoalSamples = 1;      %(#) number of samples from goal region
sampling.nTot2PBVPs = 50000;	%(#) number of 2pt BVPs to solve for trainng ML
sampling.nTrajNodes = 5;	%(#) number of disctrete points for optimal control subproblems
sampling.stateSampleRange = [
                    -0.5       2.5;...        %(m m) range to sample x
                    0       4;...        %(m m) range to sample y
                    -2.0	-1.0;...        %(m m) range to sample z
                   	-1.0 1.0;...        %(m/s) range to sample vx
                    -1.0 1.0;...        %(m/s) range to sample vy
                    -1.0 1.0];          %(m/s) range to sample vz  
% Robot Parameters
robot.mass = 0.9574;           %(kg) quadrotor mass
robot.thrustMax = 30;       %(N) maximum thrust
robot.maxPitchRoll = 60;      %(deg) maximum angular devation from flat

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
offlineOptions.tf_min = 1e-3;   % (s) lower bound of time interval
offlineOptions.tf_max = 100;    % (s) upper bound of time interval
offlineOptions.controlPenaltyWeight = 0.1; % penalty on control effort

% Machine Learn Options
learning.neighborCostThreshold = NaN; 	% (cost) radius of neighborhood  NOTE: this should be changed to be based off average cost
learning.neighborCostQuantile = 0.1;  	% Quntile of all cost data for cost threshold determination
learning.nMLTrainingSamples = 20000;      	% (#) number of training examples for ML. NaN->use all samples
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

% system definition function
definitionsFunction = @DblIntQuadDefinitions;

% Communication options
% comms.commpath = '/../../AgileQuad/ViconWifiComm/
comms.commfile = '../../AgileQuad/ViconWifiComm/Debug/trajectory.txt';
comms.xmit = true;
comms.tcpPort = 10000;  % ensure this alligns with PLANNER_PORT in CommLevel.h
comms.tcpTimeout = 1;   % [s] time until tcp connection attempt times out
% CommsWriter set in systemDefs

% Consolidate Information
mpinfo.savefile = savefile;
mpinfo.sampling = sampling;
mpinfo.robot = robot;
mpinfo.environment = environment;
mpinfo.offlineOptions = offlineOptions;
mpinfo.learning = learning;
mpinfo.definitionsFunction = definitionsFunction;
mpinfo.comms = comms;
clear sampling robot environment offlineOptions learning
clear savefile definitionsFunction
clear comms


% Call Planner
profile on
mpinfo = MotionPlannerFrameworkOffline(mpinfo, 0);
profile viewer
