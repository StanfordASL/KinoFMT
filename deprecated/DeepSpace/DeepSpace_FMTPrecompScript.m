% DeepSpace_FMTPrecompScript.m peforms the precomputation necessary to
% solve a kinodynamic motion planning problem with Fast Marching Trees
%
%   Ross Allen, ASL, Stanford University
%   Jan 28, 2014
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clear
clc
close all

% Output File
outputFile = '../../../planningdata/GenKinoFMT/DeepSpacePrecompute_200x10_Jan28-2014_';

% Problem Parameters
n_samples = 200;                    %(#) number of sampled nodes in statespace
n_trajInterpPts = 10;               %(#) number of disctrete points for optimal control subproblems
n_states = 6;                     %(#) dimension of state space
n_controls = 4;                   %(#) number of control variables
mass = 1;                         %(kg) spacecraft mass
ThrustMax = 0.1;                    %(N) maximum thrust

% Sample Vertex Set
xSampleRange = [-10 10];        %(m m) range to sample x
ySampleRange = [-10 10];        %(m m) range to sample y
zSampleRange = [-10 10];        %(m m) range to sample z
xdotSampleRange = [-1 1];   %(m/s) range to sample xdot
ydotSampleRange = [-1 1];   %(m/s) range to sample ydot
zdotSampleRange = [-1 1];   %(m/s) range to sample zdot
config_space = [xSampleRange; ySampleRange; zSampleRange;...
    xdotSampleRange; ydotSampleRange; zdotSampleRange];
halton_skip = randi(1e6,1); % randomly generate number of initial halton sequence to omit
V = HaltonSampling(n_states, n_samples, config_space, halton_skip, 0, true);

% Setup parameters common to all optimal control problems
numerics.n_nodes = n_trajInterpPts;
environment = [];   % no environmental constraints
robot.ThrustMax = ThrustMax;
robot.mass = mass;
options.print_summary = false; % (boo)
options.plot_results = false;  % (b00)
options.storeExitFlags = true;
options.storeSolverOutput = true;
options.solver = optimset('Algorithm','sqp','GradObj','on',...
    'GradConstr','on','DerivativeCheck','off', 'Display', 'off',...
    'TolFun', 5e-2, 'TolCon', 1e-4, 'TolX', 5e-2);
% options.solver.MaxFunEvals = Inf; 
options.solver.MaxIter = 200;

% Instantiate matrices to hold results
stateMatrix = NaN*ones(n_samples, 1+n_states);
costMatrix = NaN*ones(n_samples, n_samples,2);
trajectoryMatrix = NaN*ones(n_samples, n_states, n_trajInterpPts, n_samples);
controlMatrix = NaN*ones(n_samples, n_controls, n_trajInterpPts, n_samples);

% Loop through all optimal control problems
for i = 1:n_samples
    
    bv1 = V(i,:);
    boundary_values.x0 = bv1(1);
    boundary_values.y0 = bv1(2);
    boundary_values.z0 = bv1(3);
    boundary_values.xdot0 = bv1(4);
    boundary_values.ydot0 = bv1(5);
    boundary_values.zdot0 = bv1(6);
    boundary_values.t0 = 0;
    
    stateMatrix(i, :) = [i V(i,:)];
    
    for j = 1:n_samples
        disp([num2str(i) ', ' num2str(j)])
        
        % When initial and final boundary conditions are identical the
        % optimal trajectory is to do nothing for zero cost. This is
        % because the optimal control problem has free final time therefore
        % your initial and final conditions are satisfied simultaneously
        if i == j
            costMatrix(i,j,1) = 0;
            trajectoryMatrix(j,1,:,i) = bv1(1)*ones(1,1,n_trajInterpPts,1);
            trajectoryMatrix(j,2,:,i) = bv1(2)*ones(1,1,n_trajInterpPts,1);
            trajectoryMatrix(j,3,:,i) = bv1(3)*ones(1,1,n_trajInterpPts,1);
            trajectoryMatrix(j,4,:,i) = bv1(4)*ones(1,1,n_trajInterpPts,1);
            trajectoryMatrix(j,5,:,i) = bv1(5)*ones(1,1,n_trajInterpPts,1);
            trajectoryMatrix(j,6,:,i) = bv1(6)*ones(1,1,n_trajInterpPts,1);
            controlMatrix(j,:,:,i) =  zeros(1,n_controls,n_trajInterpPts,1);           
            continue;
        end
        
        % Set bounday values
        bv2 = V(j,:);
        boundary_values.xf = bv2(1);
        boundary_values.yf = bv2(2);
        boundary_values.zf = bv2(3);
        boundary_values.xdotf = bv2(4);
        boundary_values.ydotf = bv2(5);
        boundary_values.zdotf = bv2(6);
        
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
            % Infeasible trajectory
            costMatrix(i,j,1) = Inf;
            trajectoryMatrix(j,1,:,i) = NaN;
            trajectoryMatrix(j,2,:,i) = NaN;
            trajectoryMatrix(j,3,:,i) = NaN;
            trajectoryMatrix(j,4,:,i) = NaN;
            trajectoryMatrix(j,5,:,i) = NaN;
            trajectoryMatrix(j,6,:,i) = NaN;
            controlMatrix(j,1,:,i) = NaN;
            controlMatrix(j,2,:,i) = NaN;
            controlMatrix(j,3,:,i) = NaN;
            controlMatrix(j,4,:,i) = NaN;
        else
            costMatrix(i,j,1) = probinfo.solution.cost;
            trajectoryMatrix(j,1,:,i) = probinfo.solution.x;
            trajectoryMatrix(j,2,:,i) = probinfo.solution.y;
            trajectoryMatrix(j,3,:,i) = probinfo.solution.z;
            trajectoryMatrix(j,4,:,i) = probinfo.solution.xdot;
            trajectoryMatrix(j,5,:,i) = probinfo.solution.ydot;
            trajectoryMatrix(j,6,:,i) = probinfo.solution.zdot;
            controlMatrix(j,1,:,i) = probinfo.solution.ux;
            controlMatrix(j,2,:,i) = probinfo.solution.uy;
            controlMatrix(j,3,:,i) = probinfo.solution.uz;
            controlMatrix(j,4,:,i) = probinfo.solution.eta;
        end
    end
    
    % Sort cost of trajectory to other nodes for neighborhood determination
    [~, sortedIX] = sort(costMatrix(i,:,1));
    costMatrix(i,:,2) = sortedIX;
end

% Write to File
Matrix2Text(stateMatrix, strcat(outputFile,'StateID.txt'));
Matrix2Text(costMatrix, strcat(outputFile,'Cost.txt'));
Matrix2Text(trajectoryMatrix, strcat(outputFile,'Trajectory.txt'));
Matrix2Text(controlMatrix, strcat(outputFile,'Control.txt'));