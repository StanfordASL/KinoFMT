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
addpath([pwd, '/../MEXFuncs/']);
addpath([pwd, '/../AgileQuad/']);   % smoother

% Check for data
if ~exist('mpinfo','var')
    disp('Please load precomputed data');
    return;
end

profiler = false;   % (boo) set if code profiler should be run

% Start state and Goal region
%termStates.Xstart = [1.28, 3.83, -1.17, 0, 0, 0];
termStates.Xstart = NaN;
% termStates.goalRegion = ...
%     [1.25  1.25;...           % (m) x-range in goal region
%      0.25  0.25;...          % (m) y-range in goal region 
%     -1.5 -1.5;...         % (m) z-range in goal region 
%      0  0;...           % (m/s) vx-range in goal region
%      0  0;...           % (m/s) vy-range in goal region
%      0  0];             % (m/s) vz-range in goal region
termStates.Xgoal = NaN;
termStates.dtPriorStart = 0.3; % (s) time length to propagate prior measured state forward to estimate start state (should reflect planning computation time)
termStates.startStateEstimator = @DblIntQuadStartStateEstimator;
termStates.checkExistingPlan = @DblIntQuadCheckExistingPlan;


% Solver options for online optimization problems (2PBVPs)
onlineOptions.runMPC = true;
onlineOptions.maxMPCIter = 1000;
onlineOptions.obstacleCheckTime = 0.01; % this should be much less than the PX4_SPLINE_START_DELAY_T_SEC_REL
onlineOptions.xmitHoldOnFailure = true;
onlineOptions.skipCurrentValid = true;
onlineOptions.PX4_SPLINE_START_DELAY_T_SEC_REL = 1.0;  % Note this should exactly match SPLINE_START_DELAY_T_SEC_REL in mc_traj_control_main.cpp

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
nWorkDims = 3;
bounds =[   -0.5    2.9;...
            0.14    4.25;...
            -2.75   -0.2]; %(m)
        
% Obstacles
% obstacles.obstacleSpace = mpinfo.sampling.stateSampleRange(1:3,:); % dimensions and range of dimensions where obstacles may exist
obstacles.obstacleSpace = bounds; % dimensions and range of dimensions where obstacles may exist
obstacles.generationFunction = @IndoorObstacleSet6; % function to call for pre-generated, hand-coded obstacles
obstacles.nPassiveObs = NaN;                                 % number of obstacles (only if generation function requires)
obstacles.minCuboidDimension = NaN;                         % minimum dimension of a cuboid(only if generation function requires)
obstacles.maxCuboidDimension = NaN;                         % number of obstacles (only if generation function requires)
obstacles.maxActiveObs = 20;
obstacles.reduceBounds = 0.10;       % (m) amount obstacle bounds should be reduced to improve potential field control
%obstacles.spheres = [1.2, 2.195, -1.475, 0.5];


% Communications
comms = mpinfo.comms;
comms.connectVicon = true;      % should a connection be established to vicon
comms.xmitTrajectory = true;
comms.tcpPort = 10000;  % ensure this alligns with PLANNER_PORT in CommLevel.h
comms.tcpTimeout = 1;   % [s] time until tcp connection attempt times out
comms.recvXprior = true;        % should recieve the initial state from vicon   
comms.recvGoalRegion = true;    % should the goal region be extracted from vicon
comms.xmitPassiveObs = true;    % should passive (preprogrammed) obstacle be sent to ViconWifiComm
comms.recvActiveObs = true;    % should active (dynamic) obstacles be received from ViconWifiComm
comms.recvCharsPerVal = 9;      % number of characters per value during TCP recieve (note this must match formatting in ViconWifiComm)
        
% Smoother options
smoother.applySmoothing = true; % (bool) boolean to apply smoothing
smoother.yaw = -1.68;    % (rad) constant yaw to be held 
smoother.nCheckNodes = 10;   % (#) number of points on smoothed trajectory to check collisions
smoother.timeScaling = 2.0;   % (#) factor to increase time segments from linearized solver
smoother.flatPath = true;
smoother.collisionSpeedCheck = 2.0; % (#) factor to increase segment time step when segment found in collision
% TrajectorySmoother set in systemDefs

% Consolidate Information
mpinfo.profiler = profiler;
mpinfo.obstacles = obstacles;
mpinfo.termStates = termStates;
mpinfo.onlineOptions = onlineOptions;
mpinfo.environment.nWorkDims = nWorkDims;
mpinfo.environment.bounds = bounds;
mpinfo.comms = comms;
mpinfo.smoother = smoother;

clear obstacles termStates onlineOptions nWorkDims bounds profiler comms smoother

% Call online Solver
if (mpinfo.onlineOptions.runMPC)
    [mpinfo_complete, mpc_info] = MotionPlannerFrameworkOnlineMPC(mpinfo);
else
    mpinfo_complete = MotionPlannerFrameworkOnline(mpinfo);
end


