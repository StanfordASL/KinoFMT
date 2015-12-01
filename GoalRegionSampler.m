% GoalRegionSampler samples the goal region using
% halton sequencing
%
%   Ross Allen, ASL, Stanford University
%   May 19, 2014
%
%   Notes:
%       - Current version assumes goal region is a hyper-rectangle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ mpinfo ] = GoalRegionSampler( mpinfo )

% Unpack variables to be accessed and modified
nGoalSamples = mpinfo.sampling.nGoalSamples;

% Unpack variables to be accessed (not modified)
samplingRange = mpinfo.termStates.goalRegion;
sysdefs = mpinfo.systemDefs;
mpinfo.nSamples = mpinfo.sampling.nSamples;

halton_skip = randi(1e6,1); % randomly generate number of initial halton sequence to omit

Xgoal = HaltonSampling(...
    sysdefs.nStateDims, nGoalSamples, samplingRange, halton_skip, 0, true);

% Consolidate results
mpinfo.termStates.Xgoal = Xgoal;

end