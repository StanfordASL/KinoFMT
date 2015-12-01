%DeepSpaceOptimizer accepts information about a Deep Space Spacecraft 
%Problem and returns the optimal path as computed by fmincon
%
%   Ross Allen, ASL, Stanford University
%
%   Started:        12/6/2013
%
%   Inputs:     robot             user robot data structure 
%               -
%
%               environment         user environment constraints
%               - 
%
%               boundary_values     user initial state data structure
%               - 
%
%               options             user solution options properties data structure
%               - N                 (#) order of polynomial approximation (nodes = N+1)
%
%   Outputs:    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function probinfo = DeepSpaceOptimizer( probinfo )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Pre-processing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Scale Factors
probinfo = DeepSpaceScaleFactors(probinfo);

% Numerical Information
probinfo = DeepSpaceNumericData(probinfo);

% Boundary Value Information
probinfo = DeepSpaceBoundaryValuesData(probinfo);

% Robot Information
probinfo = DeepSpaceRobotData(probinfo);

% Environment Information
probinfo = DeepSpaceEnvironmentData(probinfo);

% Print Problem Information
err = DeepSpacePrintSummary(probinfo);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initial Guess of Solution
[vars_0, probinfo] = DeepSpaceInitialGuess(probinfo);

% Linear Constraints
[A, b, Aeq, beq, lb, ub] = DeepSpaceLinearConstraints(probinfo);

% Solver Options
solver_options = probinfo.options.solver;

% Call Solver
[vars_opt J_opt exitflag solveroutput] = ...
    fmincon(@(vars) DeepSpaceCostFunction(vars, probinfo),...
    vars_0, A, b, Aeq, beq, lb, ub, ...
    @(vars) DeepSpaceNonLinearConstraints(vars, probinfo), solver_options);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\
%% Post-Processing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Dimensionalize and store solution
if (isfield(probinfo.options, 'storeSolverOutput') && probinfo.options.storeSolverOutput)
    probinfo = DeepSpaceRecordSolution(vars_opt, J_opt, probinfo, exitflag, solveroutput);
elseif (isfield(probinfo.options, 'storeExitFlag') && probinfo.options.storeExitFlag)
    probinfo = DeepSpaceRecordSolution(vars_opt, J_opt, probinfo, exitflag);
else
    probinfo = DeepSpaceRecordSolution(vars_opt, J_opt, probinfo);
end


% Plot Results
if probinfo.options.plot_results
    err = DeepSpacePlotResults(probinfo);
end





end