% ClassifyTerminalNeighborhoods estimates the neighborhoods of the
% start and goal states using the SVM classifier previously trained
%
%   Ross Allen, ASL, Stanford University
%   May 19, 2014
%
%   Notes:
%       - nSamples = number of state samples not including start state or
%       goal region samples
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function mpinfo = ClassifyTerminalNeighborhoods(mpinfo)

% Unpack variables to be accessed and modified
termStates = mpinfo.termStates;

% Unpack variables to be accessed (not modified)
nSamples = mpinfo.sampling.nSamples;
nGoalSamples = mpinfo.sampling.nGoalSamples;
Xstart = termStates.Xstart;
Xgoal = termStates.Xgoal;
stateMat = mpinfo.stateMat;
featureSet = mpinfo.learning.neighbor.featureSet;
svm_output = mpinfo.learning.neighbor.svm_output;

% Classify start states neighborhood
start_neighbors_feature_matrix  = featureSet( ...
    repmat(Xstart,nSamples+nGoalSamples+1,1), stateMat(1:end,:) );
reachableFromXstart = is_reachable( svm_output,...
    start_neighbors_feature_matrix);

% Classify goal state neighborhoods
XgoalReachableFrom = zeros(nSamples+nGoalSamples+1,nGoalSamples);
for i = 1:nGoalSamples
    goal_neighbors_feature_matrix = featureSet( ...
        stateMat(1:end,:), repmat(Xgoal(i,:),nSamples+nGoalSamples+1,1) );
    XgoalReachableFrom(:,i)  = is_reachable(svm_output,...
        goal_neighbors_feature_matrix);
end
XgoalReachableFrom = logical(XgoalReachableFrom);

% Consolidate Results
termStates.reachableFromXstart = reachableFromXstart;
termStates.XgoalReachableFrom = XgoalReachableFrom ;
mpinfo.termStates = termStates;

end

