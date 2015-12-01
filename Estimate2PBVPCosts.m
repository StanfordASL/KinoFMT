% Estimate2PBVPCosts uses the cost estimator that was previously
% trained to estimate the optimal cost of an optimal 2 point boundary value
% problem
%
%   Ross Allen, ASL, Stanford University
%   May 20, 2014
%
%   Inputs:
%       - reachabilityVector = (bool) vector where element i states the
%           reachability of finalIDs(i) from initIDs(i)
%       - initIDs = (#) identification number of initial states of 2PBVPs.
%           initIDs of length=1 calculates cost from state one to all finalIDs
%       - finalIDs = (#) identification number of initial states of 2PBVPs.
%           finalIDs of length=1 calculates cost from all initIDs to finalID.
%
%   Outputs:
%       - estimatedCosts = estimated cost from initIDs to finalIDs
%
%   Notes:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function sortedEstNeighbors = Estimate2PBVPCosts(...
    reachabilityVector, initIDs, finalIDs, mpinfo)

% Unpack variables to be accessed and modified

% Unpack variables to be accessed (not modified)
learning = mpinfo.learning;
stateMat = mpinfo.stateMat;
useUnweightedBLR = learning.cost.useUnweightedBLR;
featureSet = learning.cost.featureSet;

% Check inputs
if length(reachabilityVector) == length(initIDs) || ...
        length(reachabilityVector) == length(finalIDs)
else
    disp('Length of reachabilityVector must match length of initIDs or')
    disp('finalIDs.')
    disp('Exiting Estimate2PBVPCosts prematurely...')
    return;
end

% Determine case being solved and calculate feature vectors
estimated_neighbors = find(reachabilityVector);
n_neighbors = length(estimated_neighbors);
if length(initIDs) == 1 
    cost_features  = featureSet( ...
    repmat(stateMat(initIDs,:),n_neighbors,1),...
    stateMat(estimated_neighbors,:) );
elseif length(finalIDs) == 1
    cost_features  = featureSet( ...
    stateMat(estimated_neighbors,:),...
    repmat(stateMat(finalIDs,:),n_neighbors,1) );
elseif length(initIDs) == length(finalIDs)
    cost_features  = featureSet( ...
    stateMat(initIDs(estimated_neighbors),:),...
    stateMat(finalIDs(estimated_neighbors),:));  
else
    disp('Dimension mismatch between initIDs and finalIDs.')
    disp('Exiting Estimate2PBVPCosts prematurely...')
    return;
end

% Estimate Costs
if useUnweightedBLR
    BLR = learning.cost.unweightedBLR;
    estCosts =...
        [ones(length(cost_features(:,1)),1),...
        cost_features]*BLR.theta;
else
    BLR = learning.cost.weightedBLR;
    [~, estCosts] = batchWeightedLinearRegression(...
        BLR.xData, BLR.yData, BLR.tau,...
        cost_features, BLR.xExtent);
end

% Align costs with respective state IDs
reachableStateIDs = find(reachabilityVector);
estNeighbors = [reachableStateIDs, estCosts];

% Sort estimated costs
[~, sortedIX] = sort(estCosts);

% Output
sortedEstNeighbors = estNeighbors(sortedIX,:);


end