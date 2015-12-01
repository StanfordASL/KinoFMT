% DblIntQuad_OnlineScript.m calls the online computation phase for 
%   the linearized quadrotor model. Specifically
%   this model uses a control-effort-penalizing cost function
%   (as opposed to a control constrained formulation).
%
%   Ross Allen, ASL, Stanford University
%   Jul 1, 2015
%
% NOTES:
%   - 'mpinfo' stands for motion planning information and holds all
%   relevant data for a motion planning problem (i.e. sampling info, robot
%   info, machine learning info, etc.)
%   - 'system' refers to the robot, dynamics, cost function, etc. combo. It
%   is closely related to 'probinfo' used in a single optimizing problem
%   (2PBVP)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

addpath([pwd, '/../']);
addpath([pwd, '/../MachineLearning/']);
addpath([pwd, '/../MachineLearning/Reachability Classifier/']);
addpath([pwd, '/../ObstacleSets/']);
addpath([pwd, '/../../../../planningdata/GenKinoFMT/']);
addpath([pwd, '/../AgileQuad/']);   % smoother
addpath([pwd, '/../MEXFuncs/']);   % mex functions

% Check for data
if ~exist('mpinfo','var')
    disp('Please load precomputed data');
    return;
end

profiler = true;   % (boo) set if code profiler should be run

% Start state and Goal region
% termStates.Xstart = Xstart;      % (m,m,m,m/s,m/s.m/s) initial state
% termStates.Xstart = NaN;
termStates.Xstart = [1.27, 3.68, -1.19, 0, 0, 0];
termStates.goalRegion = ...
    [1.25  1.25;...           % (m) x-range in goal region
     0.5  0.5;...          % (m) y-range in goal region 
    -1.5 -1.5;...         % (m) z-range in goal region 
     0  0;...           % (m/s) vx-range in goal region
     0  0;...           % (m/s) vy-range in goal region
     0  0];             % (m/s) vz-range in goal region


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
onlineOptions.controlPenaltyWeight = ...
    mpinfo.offlineOptions.controlPenaltyWeight; % penalty on control effort

% Environment
% - rough, rectangular coverage of vicon in ASL lab
bounds =[   -0.5    2.8;...
            0.14    4.25;...
            -2.75   -0.2]; %(m)
        
% Obstacles
% obstacles.obstacleSpace = mpinfo.sampling.stateSampleRange(1:3,:); % dimensions and range of dimensions where obstacles may exist
obstacles.obstacleSpace = bounds; % dimensions and range of dimensions where obstacles may exist
obstacles.generationFunction = @IndoorObstacleSet4; % function to call for pre-generated, hand-coded obstacles
obstacles.nObstacles = NaN;                                 % number of obstacles (only if generation function requires)
obstacles.minCuboidDimension = 1.0;                         % minimum dimension of a cuboid(only if generation function requires)
obstacles.maxCuboidDimension = 2.0;                         % number of obstacles (only if generation function requires)

        
% Smoother options
smoother.applySmoothing = true; % (bool) boolean to apply smoothing
smoother.yaw = -1.68;    % (rad) constant yaw to be held 
smoother.nCheckNodes = 10;   % (#) number of points on smoothed trajectory to check collisions
smoother.timeScaling = 1.1;   % (#) factor to increase time segments from linearized solver
smoother.flatPath = false;
% TrajectorySmoother set in systemDefs

% Consolidate Information
mpinfo.profiler = profiler;
mpinfo.obstacles = obstacles;
mpinfo.termStates = termStates;
mpinfo.onlineOptions = onlineOptions;
mpinfo.environment.bounds = bounds;
mpinfo.smoother = smoother;

clear obstacles termStates onlineOptions bounds profiler smoother

% Call online Solver
mpinfo_complete = MotionPlannerFrameworkOnline(mpinfo);

% Output Condensed Information
clear solnCondensed
i = 1;
solnCondensed{i,1} = mpinfo_complete.sampling.nSamples;    i = i+1;
solnCondensed{i,1} = mpinfo_complete.sampling.nTrajNodes;  i = i+1;
solnCondensed{i,1} = mpinfo_complete.optCost; i = i+1;
solnCondensed{i,1} = length(mpinfo_complete.optPath); i = i+1;
solnCondensed{i,1} = mpinfo_complete.smoother.nSeg; i = i+1;
solnCondensed{i,1} = mpinfo_complete.onlineCompTime; i = i+1;
solnCondensed{i,1} = mpinfo_complete.comms.recv; i = i+1;
solnCondensed{i,1} = mpinfo_complete.comms.xmit; i = i+1;
solnCondensed{i,1} = mpinfo_complete.profiler; i = i+1;
solnCondensed{i,1} = mpinfo_complete.learning.nMLTrainingSamples; i = i+1;
solnCondensed{i,1} = mpinfo_complete.learning.neighbor.training.nErrors; i = i+1;
solnCondensed{i,1} = mpinfo_complete.learning.nMLTestingSamples; i = i+1;
solnCondensed{i,1} = mpinfo_complete.learning.neighbor.testing.nErrors; i = i+1;
solnCondensed{i,1} = mpinfo_complete.learning.neighbor.testing.nTruePositives; i = i+1;
solnCondensed{i,1} = mpinfo_complete.learning.neighbor.testing.nTrueNegatives; i = i+1;
solnCondensed{i,1} = mpinfo_complete.learning.neighbor.testing.nFalsePositives; i = i+1;
solnCondensed{i,1} = mpinfo_complete.learning.neighbor.testing.nFalseNegatives; i = i+1;
solnCondensed{i,1} = mpinfo_complete.learning.neighborCostQuantile; i = i+1;
solnCondensed{i,1} = mpinfo_complete.learning.neighborCostThreshold; i = i+1;
solnCondensed{i,1} = mpinfo_complete.savefile; i = i+1;


