%BVPOptimizer accepts information about a general 2pt boundary value
%Problem and returns the optimal path as computed by fmincon
%
%   Ross Allen, ASL, Stanford University
%
%   Started:        Mar 30, 2015
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
%       - optFuncs should only be called in BVPOptimizer or 
%           CustomBVPOptimizer to restrict potential format errors.
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function probinfo = BVPOptimizer( probinfo, optFuncs )

if isfield(optFuncs, 'CustomBVPOptimizer')

	% call customized solver
	probinfo = optFuncs.CustomBVPOptimizer(probinfo, optFuncs);

else
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%% Pre-processing
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
	ScaleFactors = optFuncs.ScaleFactors;
	NumericData = optFuncs.NumericData;
	BoundaryValuesData = optFuncs.BoundaryValuesData;
	RobotData = optFuncs.RobotData;
	EnvironmentData = optFuncs.EnvironmentData;
	PrintSummary = optFuncs.PrintSummary;
	InitialGuess = optFuncs.InitialGuess;
	LinearConstraints = optFuncs.LinearConstraints;
	CostFunction = optFuncs.CostFunction;
	NonLinearConstraints = optFuncs.NonLinearConstraints;
	RecordSolution = optFuncs.RecordSolution;
	PlotResults = optFuncs.PlotResults;
	
	% Scale Factors
	probinfo = ScaleFactors(probinfo);
	
	% Numerical Information
	probinfo = NumericData(probinfo);
	
	% Boundary Value Information
	probinfo = BoundaryValuesData(probinfo);
	
	% Robot Information
	probinfo = RobotData(probinfo);
	
	% Environment Information
	probinfo = EnvironmentData(probinfo);
	
	% Print Problem Information
	err = PrintSummary(probinfo);
	
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%% Solution
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
	% Initial Guess of Solution
	[vars_0, probinfo] = InitialGuess(probinfo);
	
	% Linear Constraints
	[A, b, Aeq, beq, lb, ub] = LinearConstraints(probinfo);
	
	% Solver Options
	solver_options = probinfo.options.solver;
	
	% Call Solver
	[vars_opt J_opt exitflag solveroutput] = ...
	    fmincon(@(vars) CostFunction(vars, probinfo),...
	    vars_0, A, b, Aeq, beq, lb, ub, ...
	    @(vars) NonLinearConstraints(vars, probinfo), solver_options);
	
	
	
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\
	%% Post-Processing
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
	% Dimensionalize and store solution
	if (isfield(probinfo.options, 'storeSolverOutput') && probinfo.options.storeSolverOutput)
	    probinfo = RecordSolution(vars_opt, J_opt, probinfo, exitflag, solveroutput);
	elseif (isfield(probinfo.options, 'storeExitFlag') && probinfo.options.storeExitFlag)
	    probinfo = RecordSolution(vars_opt, J_opt, probinfo, exitflag);
	else
	    probinfo = RecordSolution(vars_opt, J_opt, probinfo);
	end
	
	
	% Plot Results
	if probinfo.options.plot_results
	    err = PlotResults(probinfo);
	end
	
end

end
