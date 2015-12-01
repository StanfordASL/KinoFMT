%DubinsOptimizer accepts information about a Dubins Car Problem and returns
%the optimal path as computed by fmincon
%
%   Ross Allen, ASL, Stanford University
%
%   Started:        10/17/2013
%
%   Inputs:     robot             user robot data structure 
%               - V                 (m/s) constant speed of car
%               - turnrate          (deg/s) maximum turning rate
%
%               environment         user environment constraints
%               - xbounds           (m, m) boundaries in x direction
%               - ybounds           (m, m) boundaries in y direction
%
%               boundary_values     user initial state data structure
%               - x0                (m) initial x-pos
%               - y0                (m) initial y-pos
%               - theta0            (deg) intial heading
%               - t0                (s) initial time (usually 0)
%               - xf                (m) final x-pos
%               - yf                (m) final y-pos
%               - thetaf            (deg) final heading
%
%               options             user solution options properties data structure
%               - N                 (#) order of polynomial approximation (nodes = N+1)
%
%   Outputs:    dubprob             output data structure containing collected input data,
%                                   derived data, and solution data 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dubprob = DubinsOptimizer( dubprob )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Pre-processing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Scale Factors
dubprob = DubinsScaleFactors(dubprob);

% Numerical Information
dubprob = DubinsNumericData(dubprob);

% Boundary Value Information
dubprob = DubinsBoundaryValuesData(dubprob);

% Robot Information
dubprob = DubinsRobotData(dubprob);

% Environment Information
dubprob = DubinsEnvironmentData(dubprob);

% Print Problem Information
err = DubinsPrintSummary(dubprob);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initial Guess of Solution
[vars_0, dubprob] = DubinsInitialGuess(dubprob);

% Linear Constraints
[A, b, Aeq, beq, lb, ub] = DubinsLinearConstraints(dubprob);

% Solver Options
solver_options = dubprob.options.solver;

% Call Solver
[vars_opt J_opt exitflag solveroutput] = ...
    fmincon(@(vars) DubinsCostFunction(vars, dubprob),...
    vars_0, A, b, Aeq, beq, lb, ub, ...
    @(vars) DubinsNonLinearConstraints(vars, dubprob), solver_options);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\
%% Post-Processing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Dimensionalize and store solution
if (isfield(dubprob.options, 'storeSolverOutput') && dubprob.options.storeSolverOutput)
    dubprob = DubinsRecordSolution(vars_opt, J_opt, dubprob, exitflag, solveroutput);
elseif (isfield(dubprob.options, 'storeExitFlag') && dubprob.options.storeExitFlag)
    dubprob = DubinsRecordSolution(vars_opt, J_opt, dubprob, exitflag);
else
    dubprob = DubinsRecordSolution(vars_opt, J_opt, dubprob);
end


% Plot Results
if dubprob.options.plot_results
    err = DubinsPlotResults(dubprob);
end





end

