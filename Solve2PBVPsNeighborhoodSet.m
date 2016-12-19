% Solve2PBVPsNeighborhoodSet solves the optimal two point boundary value
% problem between a state and it's incoming or outgoing neighborhood
%
%   Ross Allen, ASL, Stanford University
%   May 21, 2014
%
%   Inputs:
%       - initIDs = (#) identification number of initial states of 2PBVPs.
%           initIDs of length=1 calculates cost from state one to all finalIDs
%       - finalIDs = (#) identification number of initial states of 2PBVPs.
%           finalIDs of length=1 calculates cost from all initIDs to finalID.
%       - maxNeighbors = (#) maximum number of neighbors to evaluate 
%
%   Functionality:
%
%   Notes:
%       - the labeling of n2PBVPs and nTot2PBVPs is misleading. nTot2PBVPs
%       indicates the original, total number of 2PBVPs attempted, where
%       n2PBVPs were the number of successful 2PBVPs out of those. However
%       n2PBVPs was used as the CaseNum count so it grew after the initial
%       BVP solution set
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function mpinfo = Solve2PBVPsNeighborhoodSet(...
    initIDs, finalIDs, maxNeighbors, mpinfo)

% Unpack variables to be accessed and modified
n2PBVPs = mpinfo.sampling.n2PBVPs;
%NOTE: modified in loop to avoid having to making additional copies
%evalMat
%trajMat
%controlMat 
%outNeighborCell
%inNeighborCell

% Unpack variables to accessed (not modified)
sysDefs = mpinfo.systemDefs;
nStateDims = sysDefs.nStateDims;
nControlDims = sysDefs.nControlDims;
stateLabels = sysDefs.stateLabels;
controlLabels = sysDefs.controlLabels;

% set parameters that are constant for all runs
numerics.n_nodes = mpinfo.sampling.nTrajNodes;
robot = mpinfo.robot;
environment = mpinfo.environment;
options = mpinfo.onlineOptions;
options.print_summary = false;
options.plot_results = false;
options.costThreshold = mpinfo.learning.neighborCostThreshold;

% Check inputs
nCurProbs = length(initIDs);
if length(finalIDs) ~= nCurProbs
    disp('Dimension mismatch between initIDs and finalIDs.')
    disp('Exiting Solve2PBVPsNeighborhoodSet prematurely...')
    return;
end

neighbor_counter = 0;
curTrajMat = zeros(nCurProbs, nStateDims, mpinfo.sampling.nTrajNodes);
curCtrlMat = zeros(nCurProbs, nControlDims, mpinfo.sampling.nTrajNodes);
nCur2PBVPs = 0;
for j = 1:nCurProbs
    
    % skip if solving trivial pair (initial state = final state
    if initIDs(j) == finalIDs(j)
        continue;
    end
    
    % Set initial and final boundary values
    boundary_values.t0 = 0;
    for k = 1:nStateDims
        boundary_values.([stateLabels{k},'0']) = ...
            mpinfo.stateMat(initIDs(j),k);
        boundary_values.([stateLabels{k},'f']) = ...
            mpinfo.stateMat(finalIDs(j),k);
    end
    
    % Clear previous problem
    clear bvpinfo
    bvpinfo.numerics = numerics;
    bvpinfo.robot = robot;
    bvpinfo.boundary_values = boundary_values;
    bvpinfo.environment = environment;
    bvpinfo.options = options;
    
    % Call Solver
    bvpinfo = BVPOptimizer(bvpinfo, mpinfo.systemDefs);
    
    % Save Cost, Trajectory and Control and neighborhood
    if bvpinfo.solution.exitflag > 0
        neighbor_counter = neighbor_counter + 1;
        n2PBVPs = n2PBVPs+1;
        nCur2PBVPs = nCur2PBVPs+1;
        curCost = bvpinfo.solution.cost;
        % evaluation number
        mpinfo.evalMat(initIDs(j),finalIDs(j)) = n2PBVPs;
        % cost
        mpinfo.costMat(n2PBVPs,1) = curCost;

        % extract state and control trajectory
        % state trajectory
        for k = 1:nStateDims
            curTrajMat(nCur2PBVPs,k,:) = ...
                bvpinfo.solution.(stateLabels{k});
        end
        % control trajectory
        for k = 1:nControlDims
            curCtrlMat(nCur2PBVPs,k,:) = ...
                bvpinfo.solution.(controlLabels{k});
        end
%         if isfield(mpinfo.systemDefs, 'CustomBVPOptimizer')
%             mpinfo.trajMat(n2PBVPs,:,:) = reshape(bvpinfo.solution.optTraj,...
%                 1,nStateDims,mpinfo.sampling.nTrajNodes);
%             mpinfo.controlMat(n2PBVPs,:,:) = reshape(bvpinfo.solution.optCtrl,...
%                 1,nControlDims,mpinfo.sampling.nTrajNodes);
%         else
%             % state trajectory
%             for k = 1:nStateDims
%                 mpinfo.trajMat(n2PBVPs,k,:) = ...
%                     bvpinfo.solution.(stateLabels{k});
%             end
%             % control trajectory
%             for k = 1:nControlDims
%                 mpinfo.controlMat(n2PBVPs,k,:) =...
%                     bvpinfo.solution.(controlLabels{k});
%             end
%         end
        
        % Insert into sorted outgoing neighborhood for initID
        if isempty(mpinfo.outNeighborCell{initIDs(j)})
            mpinfo.outNeighborCell{initIDs(j)} = [finalIDs(j) curCost];
            mpinfo.outNeighborSortedIDs{initIDs(j)} = uint32(finalIDs(j));
            if curCost < mpinfo.learning.neighborCostThreshold
                mpinfo.outNeighborTrimmedSortedIDs{initIDs(j)} = uint32(finalIDs(j));
            end
        else
			% Sorted by Cost
            insertIX = find(mpinfo.outNeighborCell{initIDs(j)}(:,2) > curCost, 1);
            if insertIX == 1
                mpinfo.outNeighborCell{initIDs(j)} =...
                    [[finalIDs(j) curCost];...
                    mpinfo.outNeighborCell{initIDs(j)}];
            elseif isempty(insertIX)
                mpinfo.outNeighborCell{initIDs(j)} =...
                    [mpinfo.outNeighborCell{initIDs(j)};...
                    [finalIDs(j) curCost]];
            else
                mpinfo.outNeighborCell{initIDs(j)} =...
                    [mpinfo.outNeighborCell{initIDs(j)}(1:insertIX-1,:);...
                    [finalIDs(j) curCost];...
                    mpinfo.outNeighborCell{initIDs(j)}(insertIX:end,:)];
            end
            clear insertIX;
            
			% Sorted by ID
			insertIX = find(mpinfo.outNeighborSortedIDs{initIDs(j)} > finalIDs(j), 1);
            if insertIX == 1
                mpinfo.outNeighborSortedIDs{initIDs(j)} =...
                    uint32([finalIDs(j);...
                    mpinfo.outNeighborSortedIDs{initIDs(j)}]);
            elseif isempty(insertIX)
                mpinfo.outNeighborSortedIDs{initIDs(j)} =...
                    uint32([mpinfo.outNeighborSortedIDs{initIDs(j)};...
                    finalIDs(j)]);
            else
                mpinfo.outNeighborSortedIDs{initIDs(j)} =...
                    uint32([mpinfo.outNeighborSortedIDs{initIDs(j)}(1:insertIX-1);...
                    finalIDs(j);...
                    mpinfo.outNeighborSortedIDs{initIDs(j)}(insertIX:end)]);
            end
            clear insertIX;
            
            % Sorted by ID and trimmed to the cost threshold
            if curCost < mpinfo.learning.neighborCostThreshold
                insertIX = find(mpinfo.outNeighborTrimmedSortedIDs{initIDs(j)} > finalIDs(j), 1);
                if insertIX == 1
                    mpinfo.outNeighborTrimmedSortedIDs{initIDs(j)} =...
                        uint32([finalIDs(j);...
                        mpinfo.outNeighborTrimmedSortedIDs{initIDs(j)}]);
                elseif isempty(insertIX)
                    mpinfo.outNeighborTrimmedSortedIDs{initIDs(j)} =...
                        uint32([mpinfo.outNeighborTrimmedSortedIDs{initIDs(j)};...
                        finalIDs(j)]);
                else
                    mpinfo.outNeighborTrimmedSortedIDs{initIDs(j)} =...
                        uint32([mpinfo.outNeighborTrimmedSortedIDs{initIDs(j)}(1:insertIX-1);...
                        finalIDs(j);...
                        mpinfo.outNeighborTrimmedSortedIDs{initIDs(j)}(insertIX:end)]);
                end
                clear insertIX;
            end
        end
        
        % Insert into sorted incoming neighborhood for finalID
        if isempty(mpinfo.inNeighborCell{finalIDs(j)})
            mpinfo.inNeighborCell{finalIDs(j)} = [initIDs(j) curCost];
            mpinfo.inNeighborSortedIDs{finalIDs(j)} = uint32(initIDs(j));
            if curCost < mpinfo.learning.neighborCostThreshold
                mpinfo.inNeighborTrimmedSortedIDs{finalIDs(j)} = uint32(initIDs(j));
            end
        else
            % Sorted by Cost
            insertIX = find(mpinfo.inNeighborCell{finalIDs(j)}(:,2) > curCost, 1);
            if insertIX == 1
                mpinfo.inNeighborCell{finalIDs(j)} =...
                    [[initIDs(j) curCost];...
                    mpinfo.inNeighborCell{finalIDs(j)}];
            elseif isempty(insertIX)
                mpinfo.inNeighborCell{finalIDs(j)} =...
                    [mpinfo.inNeighborCell{finalIDs(j)};...
                    [initIDs(j) curCost]];
            else
                mpinfo.inNeighborCell{finalIDs(j)} =...
                    [mpinfo.inNeighborCell{finalIDs(j)}(1:insertIX-1,:);...
                    [initIDs(j) curCost];...
                    mpinfo.inNeighborCell{finalIDs(j)}(insertIX:end,:)];
            end
            clear insertIX;
            
            % Sorted by ID
            insertIX = find(mpinfo.inNeighborSortedIDs{finalIDs(j)} > initIDs(j), 1);
            if insertIX == 1
                mpinfo.inNeighborSortedIDs{finalIDs(j)} =...
                    uint32([initIDs(j);...
                    mpinfo.inNeighborSortedIDs{finalIDs(j)}]);
            elseif isempty(insertIX)
                mpinfo.inNeighborSortedIDs{finalIDs(j)} =...
                    uint32([mpinfo.inNeighborSortedIDs{finalIDs(j)};...
                    initIDs(j)]);
            else
                mpinfo.inNeighborSortedIDs{finalIDs(j)} =...
                    uint32([mpinfo.inNeighborSortedIDs{finalIDs(j)}(1:insertIX-1);...
                    initIDs(j);...
                    mpinfo.inNeighborSortedIDs{finalIDs(j)}(insertIX:end)]);
            end
            clear insertIX;
            
            % Sorted by ID and trimmed to the cost threshold
            if curCost < mpinfo.learning.neighborCostThreshold
                insertIX = find(mpinfo.inNeighborTrimmedSortedIDs{finalIDs(j)} > initIDs(j), 1);
                if insertIX == 1
                    mpinfo.inNeighborTrimmedSortedIDs{finalIDs(j)} =...
                        uint32([initIDs(j);...
                        mpinfo.inNeighborTrimmedSortedIDs{finalIDs(j)}]);
                elseif isempty(insertIX)
                    mpinfo.inNeighborTrimmedSortedIDs{finalIDs(j)} =...
                        uint32([mpinfo.inNeighborTrimmedSortedIDs{finalIDs(j)};...
                        initIDs(j)]);
                else
                    mpinfo.inNeighborTrimmedSortedIDs{finalIDs(j)} =...
                        uint32([mpinfo.inNeighborTrimmedSortedIDs{finalIDs(j)}(1:insertIX-1);...
                        initIDs(j);...
                        mpinfo.inNeighborTrimmedSortedIDs{finalIDs(j)}(insertIX:end)]);
                end
                clear insertIX;
            end
        end
    end
    
    % Check if max number of neighbors reached
    if neighbor_counter >= maxNeighbors
        break;
    end
end

% remove exess storage for curTraj and curCtrl matrices
curTrajMat(nCur2PBVPs+1:end,:,:) = [];
curCtrlMat(nCur2PBVPs+1:end,:,:) = [];

% concatentate curTrajMat onto mpinfo.trajMat only once to avoid repeated
% memory allocations
mpinfo.trajMat = cat(1,mpinfo.trajMat,curTrajMat);
mpinfo.controlMat = cat(1,mpinfo.controlMat,curCtrlMat);

% Consolidate Results
mpinfo.sampling.n2PBVPs = n2PBVPs;
%NOTE: modified in loop to avoid having to making additional copies
%evalMat
%trajMat
%controlMat 
%outNeighborCell
%inNeighborCell

end
