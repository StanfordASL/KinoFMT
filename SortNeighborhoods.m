% SortNeighborhoods sorts into ascending order the cost of incoming and
% outgoing neighborhoods based on the set of 2PBVPs that have been solved
%
%   Ross Allen, ASL, Stanford University
%   May 20, 2014
%
%   Inputs:
%       - sortingRange:     range of IDs for which neighborhoods will be
%           sorted
%   Outputs:
%       - outNeighborCell:  cell array holding the state IDs and associated
%           outgoing costs to connect each point to every other point for
%           which a 2PBVP has been solved
%       - inNeighborCell:   cell array holding the state IDs and associated
%           incoming costs to connect each point to every other point for
%           which a 2PBVP has been solved
%		- NeighborSortedIDs:	store the IDs of neighbors in ascending 
%			order, ensuring they are of type uint32
%
%   Notes:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function mpinfo = SortNeighborhoods(mpinfo, sortingRange)

% Unpack variables to be accessed and modified

% Unpack variables to be accessed (not modified)
nTotSamples = mpinfo.nTotSamples;

% Sort
outNeighborCell = cell(nTotSamples,1); % outgoing neighbors in ascending cost
outNeighborSortedIDs = cell(nTotSamples,1);	% outgoing neighbors sorted by ID
outNeighborTrimmedSortedIDs = cell(nTotSamples,1); % outgoing neighbors trimmed to cost threshold sorted by ID
inNeighborCell = cell(nTotSamples,1); % incoming neighbors in ascending cost
inNeighborSortedIDs = cell(nTotSamples,1); % incoming neighbors sorted by ID
inNeighborTrimmedSortedIDs = cell(nTotSamples,1); % incoming neighbors trimmed to cost threshold sorted by ID

for i = sortingRange
    outNeighborIDs = find(mpinfo.evalMat(i,:) > 0)';
    outNeighborCosts = mpinfo.costMat(mpinfo.evalMat(i,outNeighborIDs));
    outNeighbors = [outNeighborIDs, outNeighborCosts];
    [~,sortIX] = sort(outNeighbors(:,2));
    outNeighbors = outNeighbors(sortIX,:);
    outNeighborCell{i,1} = outNeighbors;
    outNeighborSortedIDs{i,1} = uint32(outNeighborIDs);
    if isempty(outNieghbors)
        outNeighborTrimmedSortedIDs{i,1} = [];
    else
        threshInd = find(outNeighbors(:,2) > mpinfo.learning.neighborCostThreshold, 1);
        if isempty(threshInd)
            outNeighborTrimmedSortedIDs{i,1} = outNeighborSortedIDs{i,1};
        else
            outNeighborTrimmedSortedIDs{i,1} = uint32(sort(outNeighbors(1:threshInd-1, 1)));
        end
    end
    
    inNeighborIDs = find(mpinfo.evalMat(:,i) > 0);
    inNeighborCosts = mpinfo.costMat(mpinfo.evalMat(inNeighborIDs,i));
    inNeighbors = [inNeighborIDs, inNeighborCosts];
    [~,sortIX] = sort(inNeighbors(:,2));
    inNeighbors = inNeighbors(sortIX,:);
    inNeighborCell{i,1} = inNeighbors;
    inNeighborSortedIDs{i,1} = uint32(inNeighborIDs);
    if isempty(inNeighborCell{i,1})
        inNeighborTrimmedSortedIDs{i,1} = [];
    else
        threshInd = find(inNeighbors(:,2) > mpinfo.learning.neighborCostThreshold, 1);
        if isempty(threshInd)
            inNeighborTrimmedSortedIDs{i,1} = inNeighborSortedIDs{i,1};
        else
            inNeighborTrimmedSortedIDs{i,1} = uint32(sort(inNeighbors(1:threshInd-1, 1)));
        end
    end
end

% Consolidate Results
mpinfo.outNeighborCell = outNeighborCell;
mpinfo.inNeighborCell = inNeighborCell;
mpinfo.outNeighborSortedIDs = outNeighborSortedIDs;
mpinfo.inNeighborSortedIDs = inNeighborSortedIDs;
mpinfo.outNeighborTrimmedSortedIDs = outNeighborTrimmedSortedIDs;
mpinfo.inNeighborTrimmedSortedIDs = inNeighborTrimmedSortedIDs;


end


