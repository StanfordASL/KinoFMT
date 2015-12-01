% SimpleUAVSolve2PBVPsInitialSet solves a subseto f all possible optimal 
% boundary value problems that are generated from sampling the state space.
% This subset is used to train the machine learning neighborhood
% classifier.
%
%   Ross Allen, ASL, Stanford University
%   Apr 30, 2014
%
%   Functionality:
%       - Sample 2PBVPs to be used in neighborhood classifier training and
%       testing
%       - Instantiates costMat, trajMat, controlMat and evalMat
%       - Solves sampled 2PBVPs
%       - Checks valid total case numbers
%
%   Notes:
%       - 'IDs' refer to the row index of a state in stateMat
%       - 'CaseNums' refers to the case number of a successful 2PBVP problem 
%       (i.e. the row index of a specific initStateIDs and finalStateIDs pair)
%       - n2PBVPs specifically refers to the number of successful 2PBVPs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ mpinfo ] = SimpleUAVSolve2PBVPsInitialSet( mpinfo)

% Unpack variables
sampling = mpinfo.sampling;
nSamples = sampling.nSamples;     %(#) number of sampled nodes in state space
nTot2PBVPs = sampling.nTot2PBVPs;	%(#) number of 2pt BVPs to solve for trainng ML
nTrajNodes = sampling.nTrajNodes;	%(#) number of disctrete points for optimal control subproblems
nStateDims = mpinfo.systemDefs.nStateDims;  % (#) dimension of state space
nControlDims = mpinfo.systemDefs.nControlDims; % (#) dimension of control space

% Check inputs
if nTot2PBVPs > nSamples^2
    disp('number of 2pt BVPs cannot exceed number of samples squared');
    disp('Prematurely exiting SimpleUAVSolve2PBVPs...')
    return
end

% Randomly sample 2-point boundary value problems first round solutions
initStateIDs = randsample(nSamples, nTot2PBVPs, true);
finalStateIDs = randsample(nSamples, nTot2PBVPs, true);
[unique2PBVPs, uniqueTotCaseNums, ~] = ...
    unique([initStateIDs, finalStateIDs],'rows');
nUnique2PBVPs = size(unique2PBVPs,1);
nonUniqueTotCaseNums = setdiff(1:nUnique2PBVPs, uniqueTotCaseNums);
disp(['There are ', num2str(abs(nTot2PBVPs-nUnique2PBVPs)),...
    ' non-unique 2pt BVPs'])
if nTot2PBVPs-nUnique2PBVPs ~= 0
    disp('Total case number of non unique 2PBVP = ')
    disp(nonUniqueTotCaseNums)
end

% Instantiate matrices to hold results
costMat = NaN*ones(nTot2PBVPs, 1);
trajMat = NaN*ones(nTot2PBVPs, nStateDims, nTrajNodes);
controlMat = NaN*ones(nTot2PBVPs, nControlDims, nTrajNodes);
evalMat = zeros(nSamples, nSamples); % indicates if pair of sampled states
            % is used in a 2PBVP. If so, value indicates row index for
            % cost and traj mat. Zero otherwise. Negative value indicates
            % failed 2PBVP and information is stored separately

% Set parameters that constant for all runs
numerics.n_nodes = nTrajNodes;
robot = mpinfo.robot;
options = mpinfo.offlineOptions;
options.print_summary = false; % (boo)
options.plot_results = false;  % (b00)
environment = mpinfo.environment;

% Loop through all optimal control problems (2PBVPs)
curLinearIndex = 0;
failed2PBVPs = [];
stateMat = mpinfo.stateMat;
for i = 1:nTot2PBVPs
    
    curLinearIndex = curLinearIndex + 1;
    indi = initStateIDs(i);
    indf = finalStateIDs(i);
    evalMat(indi,indf) = curLinearIndex;
    disp([num2str(curLinearIndex), ' -> ', num2str(indi), ', ', num2str(indf)])
    
    % 2PBVP initial state
    bv1 = stateMat(indi,:);
    boundary_values.x0 = bv1(1);
    boundary_values.y0 = bv1(2);
    boundary_values.z0 = bv1(3);
    boundary_values.theta0 = bv1(4);
    boundary_values.t0 = 0;
    
    % 2PBVP final state
    bv2 = stateMat(indf,:);
    boundary_values.xf = bv2(1);
    boundary_values.yf = bv2(2);
    boundary_values.zf = bv2(3);
    boundary_values.thetaf = bv2(4);
    
    % Check if initial and final state are identical
    if indi == indf
        costMat(curLinearIndex,1) = 0;
        trajMat(curLinearIndex,:,:) = repmat(bv1, [1, 1, nTrajNodes]);
        controlMat(curLinearIndex,:,:) = zeros(1,nControlDims,nTrajNodes);
        continue;
    end
    
    % Clear previous problem then consolidate current
    clear probinfo
    probinfo.numerics = numerics;
    probinfo.robot = robot;
    probinfo.boundary_values = boundary_values;
    probinfo.environment = environment;
    probinfo.options = options;
    
    % Call Solver
    probinfo = SimpleUAVOptimizer(probinfo);
    
    % Save Cost, Trajectory and Control
    if probinfo.solution.exitflag <= 0
        % Infeasible trajectory. Decriment curLinearIndex and store
        % information about failed 2PBVP for analysis
        curLinearIndex = curLinearIndex - 1;
        failed2PBVPs(end+1,:) = [indi, indf, probinfo.solution.exitflag];
        evalMat(indi,indf) = -size(failed2PBVPs,1);
    else
        costMat(curLinearIndex,1) = probinfo.solution.cost;
        trajMat(curLinearIndex,1,:) = probinfo.solution.x;
        trajMat(curLinearIndex,2,:) = probinfo.solution.y;
        trajMat(curLinearIndex,3,:) = probinfo.solution.z;
        trajMat(curLinearIndex,4,:) = probinfo.solution.theta;
        controlMat(curLinearIndex,1,:) = probinfo.solution.utheta;
        controlMat(curLinearIndex,2,:) = probinfo.solution.uz;
    end
    
end

% Remove excess from matrices
costMat(curLinearIndex+1:nTot2PBVPs,:) = [];
trajMat(curLinearIndex+1:nTot2PBVPs,:,:) = [];
controlMat(curLinearIndex+1:nTot2PBVPs,:,:) = [];

% Rename and remove variables
n2PBVPs = curLinearIndex;   % number of succesful 2pt BVPs

% Determine valid total case numbers
validTotCaseNumsBool = zeros(nTot2PBVPs,1);
validTotCaseNums = [];
for i = 1:nTot2PBVPs
    if evalMat(initStateIDs(i,1), finalStateIDs(i,1)) > 0
        validTotCaseNumsBool(i,1) = 1;
        validTotCaseNums(end+1,1) = i;
    end
end
if length(validTotCaseNums) ~= n2PBVPs
    disp('There is an inconsistency between nTot2PBVPs and (valid) n2PBVPS.')
    disp('Diagnose before continuing.')
    disp('Prematurely exiting SimpleUAVSolve2PBVPs...')
    return
end

% Store Variables for return
mpinfo.stateMat = stateMat;
mpinfo.costMat = costMat;
mpinfo.trajMat = trajMat;
mpinfo.evalMat = evalMat;
mpinfo.controlMat = controlMat;
mpinfo.sampling.n2PBVPs = n2PBVPs;
mpinfo.sampling.failed2PBVPs = failed2PBVPs;
mpinfo.sampling.initStateIDs = initStateIDs;
mpinfo.sampling.finalStateIDs = finalStateIDs;
mpinfo.sampling.unique2PBVPs = unique2PBVPs;
mpinfo.sampling.uniqueTotCaseNums = uniqueTotCaseNums;
mpinfo.sampling.nUnique2PBVPs = nUnique2PBVPs;
mpinfo.sampling.nonUniqueTotCaseNums = nonUniqueTotCaseNums;
mpinfo.sampling.validTotCaseNumsBool = validTotCaseNumsBool;
mpinfo.sampling.validTotCaseNums = validTotCaseNums;
end

