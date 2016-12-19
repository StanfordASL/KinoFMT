%MotionPlannerFrameworkOnline performs the online computation phase
%   of a motion planning problem for a general system
%
%   Ross Allen, ASL, Stanford University
%
%   Started:    June 26, 2015
%
%   Inputs:
%
%   Outputs:
%       - exit condition:
%           1   =   Successful plan
%           2   =   Goal Satisfaction
%           0   =   Accept existing plan
%           -1  =   FMT failure
%           -2  =   Smoother failure
%           -3  =   Other failure
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
function [mpinfo, exitCond] = MotionPlannerFrameworkOnline(mpinfo)

% Run profiler if requested and not in MPC loop
if (~mpinfo.onlineOptions.runMPC)
    if (mpinfo.profiler)
        profile on
    end
end

% start clock
onlineCompTimer = tic;

% initialize exit condition
exitCond = 1;

% Generate passive obstacles and encode in message to ViconWifiComm
[mpinfo, tcpSendBuf] = ObstacleHandlerPassive(mpinfo);

% Access active obstacle and termState data on ViconWifiComm TCP server
[mpinfo, continuePlanning] = ViconTCPHandler(mpinfo, tcpSendBuf);
if ~continuePlanning
   disp('Planning terminated by ViconTCPHandler due to goal satisfaction');
   exitCond = 2;
   return;
end

% Extract active obstacles from TCP connection to ViconWifiComm
mpinfo = ObstacleHandlerActive(mpinfo);
if (~isfield(mpinfo.obstacles, 'spheres'))
    mpinfo.obstacles.spheres = [];
end

% Estimate planning start state based on current state
[mpinfo, continuePlanning] = EstimatePlannerStartState(mpinfo);
if ~continuePlanning
   disp('Planning terminated because existing plan valid');
   exitCond = 0;
   return;
end
mpinfo.stateMat(1,:) = mpinfo.termStates.Xstart;

% Sample Goal Region
mpinfo = GoalRegionSampler(mpinfo);
mpinfo.stateMat(2:mpinfo.sampling.nGoalSamples+1,:) = mpinfo.termStates.Xgoal;


% Perform error check
mpinfo.onlineError = OnlineErrorCheck(mpinfo);
if mpinfo.inputError.flag > 0
    disp('User input invalid at online initiation. Check onlineError code');
    exitCond = -3;
    return;
end

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
    disp('NEIGHBOR FAILURE: No neighbors found for Xstart');
    mpinfo.fmtFailure = true;
    exitCond = -1;
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
mpinfo = RunKinoFMT_MEX(mpinfo);

% Smooth plan and/or communicate solution
if (~mpinfo.fmtFailure && isfield(mpinfo, 'optCost') && ~isnan(mpinfo.optCost))
    
    % Run smoothing function if it exist
    if (isfield(mpinfo.systemDefs, 'TrajectorySmoother') && ...
            isfield(mpinfo, 'smoother') && mpinfo.smoother.applySmoothing)
        
        mpinfo = mpinfo.systemDefs.TrajectorySmoother(mpinfo);
        
        if ~mpinfo.smoother.valid
            disp('FAILURE: Smoother')
            exitCond = -2;
        end
    end
else
    disp('FAILURE: Planner')
    exitCond = -1;
end

% Run communicator
if (isfield(mpinfo,'comms') && mpinfo.comms.xmitTrajectory)
    mpinfo.systemDefs.CommsWriter(mpinfo);
end

% log computation time
mpinfo.onlineCompTime = toc(onlineCompTimer);

% Run exit protocol if not in an MPC loop
if (~mpinfo.onlineOptions.runMPC)
    % Profiler
    if (mpinfo.profiler)
        profile viewer
        profsave(profile('info'), [mpinfo.savefile, '_profiler'])
    end

    % Close communications
    if (isfield(mpinfo.comms, 'tcpClient') && strcmp(mpinfo.comms.tcpClient.Status, 'open'))
        fclose(mpinfo.comms.tcpClient);
        delete(mpinfo.comms.tcpClient);
    end

    % Save data 
    save([mpinfo.savefile, '_complete']);
end

end
