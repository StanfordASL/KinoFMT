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
inNeighborCell = cell(nTotSamples,1); % incoming neighbors in ascending cost
inNeighborSortedIDs = cell(nTotSamples,1); % incoming neighbors sorted by ID

for i = sortingRange
    outNeighborIDs = find(mpinfo.evalMat(i,:) > 0)';
    outNeighborCosts = mpinfo.costMat(mpinfo.evalMat(i,outNeighborIDs));
    outNeighbors = [outNeighborIDs, outNeighborCosts];
    [~,sortIX] = sort(outNeighbors(:,2));
    outNeighbors = outNeighbors(sortIX,:);
    outNeighborCell{i,1} = outNeighbors;
    outNeighborSortedIDs{i,1} = uint32(outNeighborIDs);
    
    inNeighborIDs = find(mpinfo.evalMat(:,i) > 0);
    inNeighborCosts = mpinfo.costMat(mpinfo.evalMat(inNeighborIDs,i));
    inNeighbors = [inNeighborIDs, inNeighborCosts];
    [~,sortIX] = sort(inNeighbors(:,2));
    inNeighbors = inNeighbors(sortIX,:);
    inNeighborCell{i,1} = inNeighbors;
    inNeighborSortedIDs{i,1} = uint32(inNeighborIDs);
end

% Consolidate Results
mpinfo.outNeighborCell = outNeighborCell;
mpinfo.inNeighborCell = inNeighborCell;
mpinfo.outNeighborSortedIDs = outNeighborSortedIDs;
mpinfo.inNeighborSortedIDs = inNeighborSortedIDs;

end


