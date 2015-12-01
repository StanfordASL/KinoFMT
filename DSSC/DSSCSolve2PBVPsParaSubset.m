% DSSCSolve2PBVPsParaSubset solves as subset of the remaing optimal
% boundary value problems based on whether or not the neighborhood
% classifier predicts them to be reachable from one another. Solving
% only a subset is for parallelization
%
%   Ross Allen, ASL, Stanford University
%   Sep 25, 2014
%
%   Functionality:
%       
%
%   Notes:
%       - 'IDs' refer to the row index of a state in stateMat
%       - 'CaseNums' refers to the case number of a successful 2PBVP problem 
%       (i.e. the row index of a specific initStateIDs and finalStateIDs pair)
%       - pN2PBVPs specifically refers to the number of successful 2PBVPs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ pN2PBVPs, pCostMat, pEvalList, pTrajMat, pControlMat ] = ...
    DSSCSolve2PBVPsParaSubset( mpinfo, initStateRange )

% Ensure that proper path variables are established
addpath([pwd, '/../']);
addpath([pwd, '/../MachineLearning/']);
addpath([pwd, '/../MachineLearning/Reachability Classifier/']);
addpath([pwd, '/../ObstacleSets/']);

% Unpack variables to accessed and modified

% Unpack variables to be accessed (not modified)
sampling = mpinfo.sampling;
nSamples = sampling.nSamples;       %(#) number of sampled nodes in state space
stateMat = mpinfo.stateMat;
evalMat = mpinfo.evalMat;
nTrajNodes = sampling.nTrajNodes;	%(#) number of disctrete points for optimal control subproblems
svm_output = mpinfo.learning.neighbor.svm_output;


% Set parameters that constant for all runs
numerics.n_nodes = nTrajNodes;
robot = mpinfo.robot;
options = mpinfo.offlineOptions;
options.print_summary = false; % (boo)
options.plot_results = false;  % (boo)
environment = mpinfo.environment;


% Loop through subset of  optimal control problems (2PBVPs)
pN2PBVPs = 0;
for i = initStateRange
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
                    pN2PBVPs = pN2PBVPs+1;
                    pEvalList(pN2PBVPs,:) = [i,j];
                    pCostMat(pN2PBVPs,1) = bvpinfo.solution.cost;
                    pTrajMat(pN2PBVPs,1,:) = bvpinfo.solution.x;
                    pTrajMat(pN2PBVPs,2,:) = bvpinfo.solution.y;
                    pTrajMat(pN2PBVPs,3,:) = bvpinfo.solution.z;
                    pTrajMat(pN2PBVPs,4,:) = bvpinfo.solution.xdot;
                    pTrajMat(pN2PBVPs,5,:) = bvpinfo.solution.ydot;
                    pTrajMat(pN2PBVPs,6,:) = bvpinfo.solution.zdot;
                    pControlMat(pN2PBVPs,1,:) = bvpinfo.solution.ux;
                    pControlMat(pN2PBVPs,2,:) = bvpinfo.solution.uy;
                    pControlMat(pN2PBVPs,3,:) = bvpinfo.solution.uz;
                    pControlMat(pN2PBVPs,4,:) = bvpinfo.solution.eta;
                end
            end
        end
    end
end

end
