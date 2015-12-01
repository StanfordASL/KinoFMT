% Dubins_FMT_PrecompScript_V2.m peforms the precomputation necessary to
% solve a kinodynamic motion planning problem with Fast Marching Trees
%
%   Ross Allen, ASL, Stanford University
%   Jan 20, 2014
%
%   NOTES:
%       - V2 removes any reference of initial and goal states since it is
%       assumed to be unknown information until online computation
%       - V2 changes to multidimensional matrix representation of
%       trajectory instead of cell array for ease of transfer to C++
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clear
clc
close all

% Output File
outputFile = '../../../planningdata/GenKinoFMT/DubinsPrecomputeV2_3x5_Jan21-2014_';

% Problem Parameters
n_samples = 3;                    %(#) number of sampled nodes in statespace
n_trajInterpPts = 5;               %(#) number of disctrete points for optimal control subproblems
xbounds = [-10 10];                 %(m m) x boundaries
ybounds = [-10 10];                 %(m m) y boundaries
n_states = 3;                     %(#) dimension of state space
n_controls = 1;                   %(#) number of control variables
vel = 1;                            %(m/s) velocity of dubins
turnrate = 45;                      %(deg/s) max turnrate of dubins

% Sample Vertex Set
config_space = [xbounds; ybounds; [-360 360]];
V = HaltonSampling(n_states, n_samples, config_space, 0, 0, true);

% Setup parameters common to all optimal control problems
numerics.n_nodes = n_trajInterpPts;
environment.xbounds = xbounds;
environment.ybounds = ybounds;
options.print_summary = false; % (boo)
options.plot_results = false;  % (b00)
options.storeExitFlags = true;
options.storeSolverOutput = true;
options.solver = optimset('Algorithm','sqp','GradObj','on',...
    'GradConstr','on','DerivativeCheck','off', 'Display', 'off',...
    'TolFun', 1e-6, 'TolCon', 1e-6, 'TolX', 1e-6);
options.solver.MaxFunEvals = Inf; 
options.solver.MaxIter = 1000;
robot.V = vel;
robot.turnrate = turnrate;

% Instantiate matrices to hold results
stateID = NaN*ones(n_samples, 1+n_states);
costMatrix = NaN*ones(n_samples, n_samples,2);
trajectoryMatrix = NaN*ones(n_samples, n_states, n_trajInterpPts, n_samples);
controlMatrix = NaN*ones(n_samples, n_controls, n_trajInterpPts, n_samples);

% Loop through all optimal control problems
for i = 1:n_samples
    
    bv1 = V(i,:);
    boundary_values.x0 = bv1(1);
    boundary_values.y0 = bv1(2);
    boundary_values.theta0 = bv1(3);
    boundary_values.t0 = 0;
    
    stateID(i, :) = [i V(i,:)];
    
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
            controlMatrix(j,1,:,i) =  zeros(1,1,n_trajInterpPts,1);           
            continue;
        end
        
        % Set bounday values
        bv2 = V(j,:);
        boundary_values.xf = bv2(1);
        boundary_values.yf = bv2(2);
        boundary_values.thetaf = bv2(3);
        
        % Clear previous problem then consolidate current
        clear dubprob
        dubprob.numerics = numerics;
        dubprob.robot = robot;
        dubprob.boundary_values = boundary_values;
        dubprob.environment = environment;
        dubprob.options = options;
        
        % Call Solver
        dubprob = DubinsOptimizer(dubprob);
        
        % Save Cost, Trajectory and Control
        costMatrix(i,j,1) = dubprob.solution.cost;
        if dubprob.solution.exitflag <= 0
            costMatrix(i,j,1) = Inf;
        end
        trajectoryMatrix(j,1,:,i) = dubprob.solution.x;
        trajectoryMatrix(j,2,:,i) = dubprob.solution.y;
        trajectoryMatrix(j,3,:,i) = dubprob.solution.theta;
        controlMatrix(j,1,:,i) = dubprob.solution.u;
    end
    
    % Sort cost of trajectory to other nodes for neighborhood determination
    [~, sortedIX] = sort(costMatrix(i,:,1));
    costMatrix(i,:,2) = sortedIX;
end

% Write to File
Matrix2Text(stateID, strcat(outputFile,'StateID.txt'));
Matrix2Text(costMatrix, strcat(outputFile,'Cost.txt'));
Matrix2Text(trajectoryMatrix, strcat(outputFile,'Trajectory.txt'));
Matrix2Text(controlMatrix, strcat(outputFile,'Control.txt'));
