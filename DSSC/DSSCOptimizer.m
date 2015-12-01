%DSSCOptimizer accepts information about a Deep Space Spacecraft 
%Problem and returns the optimal path as computed by fmincon
%
%   Ross Allen, ASL, Stanford University
%
%   Started:        9/24/2014
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
%
%   Notes:
%       - Dynamics based on simplified dubins+single integrator. See
%       Hwangbo & Kuffner's Efficient Two Phase 3D Motion Planning for
%       Small Fixed-wing UAVs for details
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function probinfo = DSSCOptimizer( probinfo )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Pre-processing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Scale Factors
probinfo = DSSCScaleFactors(probinfo);

% Numerical Information
probinfo = DSSCNumericData(probinfo);

% Boundary Value Information
probinfo = DSSCBoundaryValuesData(probinfo);

% Robot Information
probinfo = DSSCRobotData(probinfo);

% Environment Information
probinfo = DSSCEnvironmentData(probinfo);

% Print Problem Information
err = DSSCPrintSummary(probinfo);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initial Guess of Solution
[vars_0, probinfo] = DSSCInitialGuess(probinfo);

% Linear Constraints
[A, b, Aeq, beq, lb, ub] = DSSCLinearConstraints(probinfo);

% Solver Options
solver_options = probinfo.options.solver;

% Call Solver
[vars_opt J_opt exitflag solveroutput] = ...
    fmincon(@(vars) DSSCCostFunction(vars, probinfo),...
    vars_0, A, b, Aeq, beq, lb, ub, ...
    @(vars) DSSCNonLinearConstraints(vars, probinfo), solver_options);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\
%% Post-Processing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Dimensionalize and store solution
if (isfield(probinfo.options, 'storeSolverOutput') && probinfo.options.storeSolverOutput)
    probinfo = DSSCRecordSolution(vars_opt, J_opt, probinfo, exitflag, solveroutput);
elseif (isfield(probinfo.options, 'storeExitFlag') && probinfo.options.storeExitFlag)
    probinfo = DSSCRecordSolution(vars_opt, J_opt, probinfo, exitflag);
else
    probinfo = DSSCRecordSolution(vars_opt, J_opt, probinfo);
end


% Plot Results
if probinfo.options.plot_results
    err = DSSCPlotResults(probinfo);
end





end
