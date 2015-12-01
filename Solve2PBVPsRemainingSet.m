% Solve2PBVPsRemainingSet solves the remaining set of optimal
% boundary value problems based on whether or not the neighborhood
% classifier predicts them to be reachable from one another
%
%   Ross Allen, ASL, Stanford University
%   Feb 3, 2015
%
%   Functionality:
%       - loops through all possible 2PBVPs
%       - determines which have not been evaluated
%       - if SVM predicts a final state is reachable from an initial state,
%       then the corresponding optimal bvp is solved
%       - upadtes n2PBVPs to reflect the number of feasible optimal bvps
%       solved
%       
%
%   Notes:
%       - 'IDs' refer to the row index of a state in stateMat
%       - 'CaseNums' refers to the case number of a successful 2PBVP problem 
%       (i.e. the row index of a specific initStateIDs and finalStateIDs pair)
%       - n2PBVPs specifically refers to the number of successful 2PBVPs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ mpinfo ] = Solve2PBVPsRemainingSet( mpinfo )

% Unpack variables to accessed and modified
n2PBVPs = mpinfo.sampling.n2PBVPs;         %(#) number of 2pt BVPs to solve for trainng ML

% modified in loop to avoid having to copy
% mpinfo.costMat;
% mpinfo.evalMat;
% mpinfo.trajMat;
% mpinfo.controlMat;

% Unpack variables to be accessed (not modified)
sampling = mpinfo.sampling;
nSamples = sampling.nSamples;       %(#) number of sampled nodes in state space
nStateDims = mpinfo.systemDefs.nStateDims;
nControlDims = mpinfo.systemDefs.nControlDims;
stateMat = mpinfo.stateMat;
nTrajNodes = sampling.nTrajNodes;	%(#) number of disctrete points for optimal control subproblems
svm_output = mpinfo.learning.neighbor.svm_output;
stateLabels = mpinfo.systemDefs.stateLabels;
controlLabels = mpinfo.systemDefs.controlLabels;

% Set parameters that constant for all runs
numerics.n_nodes = nTrajNodes;
robot = mpinfo.robot;
environment = mpinfo.environment;
options = mpinfo.offlineOptions;
options.print_summary = false; % (boo)
options.plot_results = false;  % (boo)
options.costThreshold = mpinfo.learning.neighborCostThreshold;

% Loop through remaining optimal control problems (2PBVPs)
for i = 1:nSamples
    disp(i)
    for j = 1:nSamples
%         disp([num2str(i), ', ', num2str(j)])
        if i ~= j && mpinfo.evalMat(i,j) == 0 % non trivial and has not been evaluated
            curInitState = stateMat(i,:);
            curFinalState = stateMat(j,:);
            curReachPred = is_reachable(svm_output,...
                mpinfo.learning.neighbor.featureSet(...
                curInitState, curFinalState));
            if curReachPred == 1

				% set boundary values
                clear boundary_values
				boundary_values.t0 = 0;
				for k = 1:nStateDims
					boundary_values.([stateLabels{k},'0']) = ...
						curInitState(k);
					boundary_values.([stateLabels{k},'f']) = ...
						curFinalState(k);
				end
                
                % Clear previous problem
                clear bvpinfo
                bvpinfo.numerics = numerics;
                bvpinfo.robot = robot;
                bvpinfo.boundary_values = boundary_values;
                bvpinfo.environment = environment;
                bvpinfo.options = options;

                % Call solver
                bvpinfo = BVPOptimizer(bvpinfo, mpinfo.systemDefs);

                % Record solution if feasible
                if bvpinfo.solution.exitflag > 0
                    n2PBVPs = n2PBVPs+1;
					% record evaluation pair
                    mpinfo.evalMat(i,j) = n2PBVPs;
					% record cost
                    mpinfo.costMat(n2PBVPs,1) = bvpinfo.solution.cost;
                    % record state trajectory
					for k = 1:nStateDims
						mpinfo.trajMat(n2PBVPs, k, :) = ...
							bvpinfo.solution.(stateLabels{k});
					end
					% record control trajectory
					for k = 1:nControlDims
						mpinfo.controlMat(n2PBVPs, k, :) = ...
							bvpinfo.solution.(controlLabels{k});
					end
                end
            end
        end
    end
end


% Store Variables for return
mpinfo.sampling.n2PBVPs = n2PBVPs;
end
