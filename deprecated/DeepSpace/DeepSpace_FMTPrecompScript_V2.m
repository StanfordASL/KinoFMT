% DeepSpace_FMTPrecompScript.m peforms the precomputation necessary to
% solve a kinodynamic motion planning problem with Fast Marching Trees
%
%   Ross Allen, ASL, Stanford University
%   Mar 10, 2014
%
%   V2:
%   - Seperates initial and final states for training examples instead of
%   solving forward and backward problems of each state pair (n^2 examples)
%   so that the state space can be more densily sampled
%   - Separates out infeasible/infinite cost cases. For this problem there
%   should be no infeasible 2PBVP problems so any that appear are an error
%   in the solver and should not be used for training. Kept for analysis of
%   what might be causing failure, though
%   - Tightened tolerances on solution
%   - Saves .mat file for ease of use in other Matlab functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%NOTE: CHANGES NOT YET IMPLEMENTED, JUST NOTED FOR FUTURE REFERENCE

clear
clc
close all

% Add paths
addpath([pwd, '/../']);

% Output File
outputFile = '../../../../planningdata/GenKinoFMT/DeepSpacePrecomputeV2_1000x5000x10_Mar11-2014';

% Problem Parameters
nSamples = 1000;                    %(#) number of sampled nodes in statespace
nTot2PBVPs = 5000;                   %(#) number of 2pt BVPs to solve for trainng ML
nTrajNodes = 10;               %(#) number of disctrete points for optimal control subproblems
nStateDims = 6;                     %(#) dimension of state space
nControlDims = 4;                   %(#) number of control variables
mass = 1;                         %(kg) spacecraft mass
ThrustMax = 0.1;                    %(N) maximum thrust

% Check inputs
if nTot2PBVPs > nSamples^2
    disp('number of 2pt BVPs cannot exceed number of samples squared');
    return
end

% Sample Vertex Set
xSampleRange = [-100 100];        %(m m) range to sample x
ySampleRange = [-100 100];        %(m m) range to sample y
zSampleRange = [-100 100];        %(m m) range to sample z
xdotSampleRange = [-1 1];   %(m/s) range to sample xdot
ydotSampleRange = [-1 1];   %(m/s) range to sample ydot
zdotSampleRange = [-1 1];   %(m/s) range to sample zdot
config_space = [xSampleRange; ySampleRange; zSampleRange;...
    xdotSampleRange; ydotSampleRange; zdotSampleRange];
halton_skip = randi(1e6,1); % randomly generate number of initial halton sequence to omit
stateMat = HaltonSampling(nStateDims, nSamples, config_space, halton_skip, 0, true);
% stateMat = V;

% Randomly sample 2-point boundary value problems for ML training
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

% Setup parameters common to all optimal control problems
numerics.n_nodes = nTrajNodes;
environment = [];   % no environmental constraints
robot.ThrustMax = ThrustMax;
robot.mass = mass;
options.print_summary = false; % (boo)
options.plot_results = false;  % (b00)
options.storeExitFlags = true;
options.storeSolverOutput = true;
options.solver = optimset('Algorithm','sqp','GradObj','on',...
    'GradConstr','on','DerivativeCheck','off', 'Display', 'off',...
    'TolFun', 1e-4, 'TolCon', 1e-4, 'TolX', 1e-4);
options.solver.MaxIter = 200;

% Instantiate matrices to hold results
costMat = NaN*ones(nTot2PBVPs, 1);
trajMat = NaN*ones(nTot2PBVPs, nStateDims, nTrajNodes);
controlMat = NaN*ones(nTot2PBVPs, nControlDims, nTrajNodes);
evalMat = zeros(nSamples, nSamples); % indicates if pair of sampled states
            % is used in a 2PBVP. If so, value indicates row index for
            % cost and traj mat. Zero otherwise. Negative value indicates
            % failed 2PBVP and information is stored separately

% Loop through all optimal control problems
curLinearIndex = 0;
failed2PBVPs = [];
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
    boundary_values.xdot0 = bv1(4);
    boundary_values.ydot0 = bv1(5);
    boundary_values.zdot0 = bv1(6);
    boundary_values.t0 = 0;
    
    % 2PBVP final state
    bv2 = stateMat(indf,:);
    boundary_values.xf = bv2(1);
    boundary_values.yf = bv2(2);
    boundary_values.zf = bv2(3);
    boundary_values.xdotf = bv2(4);
    boundary_values.ydotf = bv2(5);
    boundary_values.zdotf = bv2(6);
    
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
    probinfo = DeepSpaceOptimizer(probinfo);
    
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
        trajMat(curLinearIndex,4,:) = probinfo.solution.xdot;
        trajMat(curLinearIndex,5,:) = probinfo.solution.ydot;
        trajMat(curLinearIndex,6,:) = probinfo.solution.zdot;
        controlMat(curLinearIndex,1,:) = probinfo.solution.ux;
        controlMat(curLinearIndex,2,:) = probinfo.solution.uy;
        controlMat(curLinearIndex,3,:) = probinfo.solution.uz;
        controlMat(curLinearIndex,4,:) = probinfo.solution.eta;
    end
    
end

% Remove excess from matrices
costMat(curLinearIndex+1:nTot2PBVPs,:) = [];
trajMat(curLinearIndex+1:nTot2PBVPs,:,:) = [];
controlMat(curLinearIndex+1:nTot2PBVPs,:,:) = [];

% Rename and remove variables
n2PBVPs = curLinearIndex;   % number of succesful 2pt BVPs
clear probinfo boundary_values bv1 bv2 i indf indi curLinearIndex
clear xSampleRange ySamplerange zSampleRange xdotSampleRange ydotSampleRange zdotSampleRange
clear ThrustMax mass

% Write to File
save(outputFile);

