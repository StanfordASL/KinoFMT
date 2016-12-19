% DblIntQuad_OnlineNonFlightScript.m calls the online computation phase for 
%   the linearized quadrotor model. This version is meant for use
%   for non-flight tests campaigns. Specifically
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
%termStates.Xstart = NaN;
% termStates.Xstart = [1.0, 2.0, 2.0, 0.0, 0.0, 0.0];
% termStates.goalRegion = ...
%     [1.25  1.25;...           % (m) x-range in goal region
%      0.25  0.25;...          % (m) y-range in goal region 
%     -1.5 -1.5;...         % (m) z-range in goal region 
%      0  0;...           % (m/s) vx-range in goal region
%      0  0;...           % (m/s) vy-range in goal region
%      0  0];             % (m/s) vz-range in goal region
% termStates.goalRegion = ...
%     [4.0  4.0;...           % (m) x-range in goal region
%      2.0 2.0;...          % (m) y-range in goal region 
%      2.0 2.0;...         % (m) z-range in goal region 
%      0  0;...           % (m/s) vx-range in goal region
%      0  0;...           % (m/s) vy-range in goal region
%      0  0];             % (m/s) vz-range in goal region

%termStates.Xgoal = NaN;
termStates.dtPriorStart = 0.3; % (s) time length to propagate prior measured state forward to estimate start state (should reflect planning computation time)
termStates.startStateEstimator = @DblIntQuadStartStateEstimator;
termStates.checkExistingPlan = @DblIntQuadCheckExistingPlan;


% Solver options for online optimization problems (2PBVPs)
onlineOptions.runMPC = false;
onlineOptions.maxMPCIter = 1000;
onlineOptions.obstacleCheckTime = 0.01; % this should be much less than the PX4_SPLINE_START_DELAY_T_SEC_REL
onlineOptions.xmitHoldOnFailure = true;
onlineOptions.skipCurrentValid = true;
onlineOptions.PX4_SPLINE_START_DELAY_T_SEC_REL = 1.0;  % Note this should exactly match SPLINE_START_DELAY_T_SEC_REL in mc_traj_control_main.cpp

onlineOptions.maxNeighbors = 300;
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
bounds = [  0.0     20.0;...
            0.0     4.0;...
            0.0     4.0]; %(m)
% bounds = [];

        
% Obstacles
% obstacles.obstacleSpace = mpinfo.sampling.stateSampleRange(1:3,:); % dimensions and range of dimensions where obstacles may exist
obstacles.obstacleSpace = bounds; % dimensions and range of dimensions where obstacles may exist
% obstacles.generationFunction = @IndoorObstacleSet8; % function to call for pre-generated, hand-coded obstacles
obstacles.generationFunction = @EmptyObstacleSetUL; % function to call for pre-generated, hand-coded obstacles
obstacles.nPassiveObs = NaN;                                 % number of obstacles (only if generation function requires)
obstacles.minCuboidDimension = NaN;                         % minimum dimension of a cuboid(only if generation function requires)
obstacles.maxCuboidDimension = NaN;                         % number of obstacles (only if generation function requires)
obstacles.maxActiveObs = 20;
obstacles.reduceBounds = 0.10;       % (m) amount obstacle bounds should be reduced to improve potential field control
%obstacles.spheres = [1.2, 2.195, -1.475, 0.5];


% Communications
comms = mpinfo.comms;
comms.connectVicon = false;      % should a connection be established to vicon
comms.xmitTrajectory = false;
comms.tcpPort = 10000;  % ensure this alligns with PLANNER_PORT in CommLevel.h
comms.tcpTimeout = 1;   % [s] time until tcp connection attempt times out
comms.recvXprior = false;        % should recieve the initial state from vicon   
comms.recvGoalRegion = false;    % should the goal region be extracted from vicon
comms.xmitPassiveObs = false;    % should passive (preprogrammed) obstacle be sent to ViconWifiComm
comms.recvActiveObs = false;    % should active (dynamic) obstacles be received from ViconWifiComm
comms.recvCharsPerVal = 9;      % number of characters per value during TCP recieve (note this must match formatting in ViconWifiComm)
        
% Smoother options
smoother.applySmoothing = true; % (bool) boolean to apply smoothing
smoother.yaw = -1.68;    % (rad) constant yaw to be held 
smoother.nCheckNodes = 10;   % (#) number of points on smoothed trajectory to check collisions
smoother.timeScaling = 1.0;   % (#) factor to increase time segments from linearized solver
smoother.flatPath = false;
smoother.collisionSpeedCheck = 2.0; % (#) factor to increase segment time step when segment found in collision
% TrajectorySmoother set in systemDefs

% Consolidate Information
mpinfo.profiler = profiler;
mpinfo.obstacles = obstacles;
mpinfo.termStates = termStates;
mpinfo.onlineOptions = onlineOptions;
mpinfo.environment.bounds = bounds;
mpinfo.environment.nWorkDims = nWorkDims;
mpinfo.comms = comms;
mpinfo.smoother = smoother;

clear obstacles termStates onlineOptions nWorkDims bounds profiler comms smoother

% Fix neighborhoods in case of old data
mpinfo = DblIntQuadNeighborFixer(mpinfo);
% if ~isfield(mpinfo, 'outNeighborTrimmedSortedIDs') || ...
%         ~isfield(mpinfo, 'inNeighborTrimmedSortedIDs')
%     mpinfo = DblIntQuadNeighborFixer(mpinfo);
% end

% Run a series of trials
clear campaign_info
nSphObs = [1 15 30 40 50 60 70 80 90 100];
% nSphObs = 5;
radSphObs = 1.0;
for campaign_iter = 1:length(nSphObs)
    nTrials = 100;
    clear trial_info mpinfo_trial
    for trial_iter = 1:nTrials
        
        % Randomly generate start state
        mpinfo.termStates.Xstart = [2, 4, 4, 0, 0, 0].*rand(1,6);
        
        % Randomly generate goal state
        mpinfo.termStates.goalRegion = repmat(([2, 4, 4, 0, 0, 0].*rand(1,6)+[18, 0, 0, 0, 0, 0])',1,2);
        
        % Randomly generate obstacle locations
        mpinfo.obstacles.spheres = repmat([14, 4, 4], ...
            nSphObs(campaign_iter),1).*rand(nSphObs(campaign_iter), 3) + ...
            3.0*[ones(nSphObs(campaign_iter),1), zeros(nSphObs(campaign_iter),2)];
        mpinfo.obstacles.spheres = [mpinfo.obstacles.spheres, ...
            radSphObs*ones(nSphObs(campaign_iter),1)];
        
        % Call solver
        [mpinfo_complete, exitCond] = MotionPlannerFrameworkOnline(mpinfo);
        
        % Store data
        % save at every iteration
        trial_info.exitCond(trial_iter) = exitCond;
        trial_info.spheres{trial_iter} = mpinfo_complete.obstacles.spheres;
        trial_info.Xstart{trial_iter} = mpinfo_complete.termStates.Xstart;
        trial_info.Xgoal{trial_iter} = mpinfo_complete.termStates.Xgoal;
        
        % save results if successful
        if (exitCond == 1 || exitCond == -2)
            % store salient results from previous plan
            trial_info.optPath{trial_iter} = mpinfo_complete.optPath;
            trial_info.optCost{trial_iter} = mpinfo_complete.optCost;
            trial_info.treeParents{trial_iter} = mpinfo_complete.treeParents;
            trial_info.onlineCompTime{trial_iter} = mpinfo_complete.onlineCompTime;
            if (exitCond == 1)
                trial_info.Tdel{trial_iter} = mpinfo_complete.smoother.Tdel;
                trial_info.splineCoefs{trial_iter} = mpinfo_complete.smoother.splineCoefs;
                %             mpinfo_trial{trial_iter} = mpinfo_complete;
            end
        end
        
        disp(['iter = ', num2str(trial_iter)]);
        
    end
    
    DblIntQuad_TrialProcessingScript;
    
    campaign_info.avgOptCost{campaign_iter} = avgCost;
    campaign_info.avgCompTime{campaign_iter} = avgTime;
    campaign_info.avgSolnNodes{campaign_iter} = avgNodes;
    campaign_info.failRate{campaign_iter} = failRate;
    campaign_info.nSphObs{campaign_iter} = nSphObs(campaign_iter);
    campaign_info.radSphObs{campaign_iter} = radSphObs;
end




