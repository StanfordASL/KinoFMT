% DSSCSolve2PBVPsRemainingSet solves the remaining set of optimal
% boundary value problems based on whether or not the neighborhood
% classifier predicts them to be reachable from one another
%
%   Ross Allen, ASL, Stanford University
%   Sep 24, 2014
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

function [ mpinfo ] = DSSCSolve2PBVPsRemainingSet( mpinfo )

% Unpack variables to accessed and modified
n2PBVPs = mpinfo.sampling.n2PBVPs;         %(#) number of 2pt BVPs to solve for trainng ML
costMat = mpinfo.costMat;
evalMat = mpinfo.evalMat;
trajMat = mpinfo.trajMat;
controlMat = mpinfo.controlMat;

% Unpack variables to be accessed (not modified)
sampling = mpinfo.sampling;
nSamples = sampling.nSamples;       %(#) number of sampled nodes in state space
stateMat = mpinfo.stateMat;
nTrajNodes = sampling.nTrajNodes;	%(#) number of disctrete points for optimal control subproblems
svm_output = mpinfo.learning.neighbor.svm_output;


% Set parameters that constant for all runs
numerics.n_nodes = nTrajNodes;
robot = mpinfo.robot;
options = mpinfo.offlineOptions;
options.print_summary = false; % (boo)
options.plot_results = false;  % (boo)
environment = mpinfo.environment;


% Loop through remaining optimal control problems (2PBVPs)
for i = 1:nSamples
    disp(i)
    for j = 1:nSamples
%         disp([num2str(i), ', ', num2str(j)])
        if i ~= j && evalMat(i,j) == 0 % non trivial and has not been evaluated
            curInitState = stateMat(i,:);
            curFinalState = stateMat(j,:);
            curReachPred = is_reachable(svm_output,...
                mpinfo.learning.neighbor.featureSet(...
                curInitState, curFinalState));
            if curReachPred == 1
                clear boundary_values
                boundary_values.x0 = curInitState(1);
                boundary_values.y0 = curInitState(2);
                boundary_values.z0 = curInitState(3);
                boundary_values.xdot0 = curInitState(4);
                boundary_values.ydot0 = curInitState(5);
                boundary_values.zdot0 = curInitState(6);
                boundary_values.t0 = 0;
                boundary_values.xf = curFinalState(1);
                boundary_values.yf = curFinalState(2);
                boundary_values.zf = curFinalState(3);
                boundary_values.xdotf = curFinalState(4);
                boundary_values.ydotf = curFinalState(5);
                boundary_values.zdotf = curFinalState(6);
                
                % Clear previous problem
                clear bvpinfo
                bvpinfo.numerics = numerics;
                bvpinfo.robot = robot;
                bvpinfo.boundary_values = boundary_values;
                bvpinfo.environment = environment;
                bvpinfo.options = options;
                % Call solver
                bvpinfo = DSSCOptimizer(bvpinfo);
                % Record solution if feasible
                if bvpinfo.solution.exitflag > 0
                    n2PBVPs = n2PBVPs+1;
                    evalMat(i,j) = n2PBVPs;
                    costMat(n2PBVPs,1) = bvpinfo.solution.cost;
                    trajMat(n2PBVPs,1,:) = bvpinfo.solution.x;
                    trajMat(n2PBVPs,2,:) = bvpinfo.solution.y;
                    trajMat(n2PBVPs,3,:) = bvpinfo.solution.z;
                    trajMat(n2PBVPs,4,:) = bvpinfo.solution.xdot;
                    trajMat(n2PBVPs,5,:) = bvpinfo.solution.ydot;
                    trajMat(n2PBVPs,6,:) = bvpinfo.solution.zdot;
                    controlMat(n2PBVPs,1,:) = bvpinfo.solution.ux;
                    controlMat(n2PBVPs,2,:) = bvpinfo.solution.uy;
                    controlMat(n2PBVPs,3,:) = bvpinfo.solution.uz;
                    controlMat(n2PBVPs,4,:) = bvpinfo.solution.eta;
                end
            end
        end
    end
end


% Store Variables for return
mpinfo.costMat = costMat;
mpinfo.trajMat = trajMat;
mpinfo.evalMat = evalMat;
mpinfo.controlMat = controlMat;
mpinfo.sampling.n2PBVPs = n2PBVPs;
end
