% ReformDataMatrices expands stateMat and evalMat to accommodate the start
% state and goal region samples
%
%   Ross Allen, ASL, Stanford University
%   May 20, 2014
%
%   Outputs:
%       - stateMat: reformatted to accommodate terminal samples
%       - evalMat:  reformatted to accommodate terminal samples
%       - nTotSamples:  total number of samples from state space including
%           start and goal region states
%
%   Notes:
%       - This function slightly violates the "no information about the
%       goal region until online computation" assumption. It assumes you
%       know the goal region is either a single point or a hyper rectangle.
%       If the planner is called properly, this violation of the assumption
%       will not affect timing.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function mpinfo = ReformDataMatrices(mpinfo)

% Unpack variables to be accessed and modified
stateMat = mpinfo.stateMat;
evalMat = mpinfo.evalMat;
nGoalSamples = mpinfo.sampling.nGoalSamples;

% Unpack variables to be accessed (not modified)
nSamples = mpinfo.sampling.nSamples;
nStateDims = mpinfo.systemDefs.nStateDims;


% Total number of samples from state space including terminal states
nTotSamples = nSamples + nGoalSamples + 1;

% Reform Matrices
stateMat = cat(1, NaN*ones(1+nGoalSamples, nStateDims), stateMat);
evalMat = cat(1, zeros(1+nGoalSamples, nSamples), evalMat);
evalMat = cat(2, zeros(nTotSamples, nGoalSamples+1), evalMat);

% Consolidate Results
mpinfo.stateMat = stateMat;
mpinfo.evalMat = evalMat;
mpinfo.nTotSamples = nTotSamples;
end