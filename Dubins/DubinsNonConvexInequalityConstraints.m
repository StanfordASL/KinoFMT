%   DubinsNonConvexInequalityConstraints.m: Computes the values for non-convex inequality
%   constraints
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        12/2/2013
%
%   Inputs:         vars            vector containing problem variable values
%                   dubprob         global variable holding problem parameters
%
%   Outputs:        g_nonconvineq           Value of the inequality constraints 
%
%   Note:           This function assumes the convention g(x) <= 0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function g_nonconvineq = DubinsNonConvexInequalityConstraints(vars, dubprob) 

% There are no non-convex inequality constraints
g_nonconvineq = [];

end