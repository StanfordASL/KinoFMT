%MotionPlannerFrameworkTimingCalc performs the online computation phase
%   of a motion planning problem for a general system. For timing purposes
%   only
%
%   Ross Allen, ASL, Stanford University
%
%   Started:    Oct 20, 2015
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
function [mpinfo] = MotionPlannerFrameworkTimingCalc(mpinfo, Xstart)

% Ensure that proper path variables are established
addpath([pwd, '/../']);
addpath([pwd, '/../MachineLearning/']);
addpath([pwd, '/../MachineLearning/Reachability Classifier/']);
addpath([pwd, '/../ObstacleSets/']);
addpath([pwd, '/../../../../planningdata/GenKinoFMT/']);

if (isfield(mpinfo,'profiler') && mpinfo.profiler == true)
    profile on
end

tic

% Perform error check
mpinfo.onlineError = OnlineErrorCheck(mpinfo);
if mpinfo.inputError.flag > 0
    disp('User input invalid at online initiation. Check onlineError code');
    return;
end

% Generate obstacle set
mpinfo = GetCuboidObstacles(mpinfo);

% Start state already known
disp('skipping start state determinatioin');
mpinfo.termStates.Xstart = Xstart;
mpinfo.stateMat(1,:) = mpinfo.termStates.Xstart;

% Goal region
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

% Run smoothing function if it exist
if (isfield(mpinfo.systemDefs, 'TrajectorySmoother') && ...
    isfield(mpinfo, 'smoother') && mpinfo.smoother.applySmoothing)
    mpinfo = mpinfo.systemDefs.TrajectorySmoother(mpinfo);
end

mpinfo.onlineCompTime = toc;

% Write data to communication file for transmission to robot
disp('skipping communication')

if (isfield(mpinfo,'profiler') && mpinfo.profiler == true)
    profile viewer
    profsave(profile('info'), [mpinfo.savefile, '_profiler'])
end

% Save data
save([mpinfo.savefile, '_timingCalc']);

end