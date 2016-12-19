% DblIntQuadNeighborFixer creates a TrimmedSortedIDs neighborhood for 
%   old data files that don't have it
%
% AUTHOR:   Ross Allen
% DATE:     Mar 3, 2016
%
% NOTES:
%   - outNeighborTrimmedSortedIDs = outgoing neighbors trimmed to cost threshold sorted by ID
%   - inNeighborTrimmedSortedIDs = incoming neighbors trimmed to cost threshold sorted by ID
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function mpinfo = DblIntQuadNeighborFixer(mpinfo)

nTotSamples = mpinfo.nTotSamples;

mpinfo.outNeighborTrimmedSortedIDs = cell(nTotSamples,1);
mpinfo.inNeighborTrimmedSortedIDs = cell(nTotSamples,1);
for i = 1:nTotSamples
    
    % outgoing neighborhood
    if isempty(mpinfo.outNeighborCell{i,1})
        mpinfo.outNeighborTrimmedSortedIDs{i,1} = [];
    else
        threshInd = find(mpinfo.outNeighborCell{i,1}(:,2) > ...
            mpinfo.learning.neighborCostThreshold, 1);
        if isempty(threshInd)
            mpinfo.outNeighborTrimmedSortedIDs{i,1} = ...
                mpinfo.outNeighborSortedIDs{i,1};
        else
            mpinfo.outNeighborTrimmedSortedIDs{i,1} = ...
                uint32(sort(mpinfo.outNeighborCell{i,1}(1:threshInd-1, 1)));
        end
    end
    
    % incoming neighborhood
    if isempty(mpinfo.inNeighborCell{i,1})
        mpinfo.inNeighborTrimmedSortedIDs{i,1} = [];
    else
        threshInd = find(mpinfo.inNeighborCell{i,1}(:,2) > ...
            mpinfo.learning.neighborCostThreshold, 1);
        if isempty(threshInd)
            mpinfo.inNeighborTrimmedSortedIDs{i,1} = ...
                mpinfo.inNeighborSortedIDs{i,1};
        else
            mpinfo.inNeighborTrimmedSortedIDs{i,1} = ...
                uint32(sort(mpinfo.inNeighborCell{i,1}(1:threshInd-1, 1)));
        end
    end
end