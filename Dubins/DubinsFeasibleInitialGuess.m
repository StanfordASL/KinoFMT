%   DubinsFeasibleInitialGuess.m: Generate a starting guess for
%   optimization by solving a feasibility problem with affine equality and
%   convex inequality constraints
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        10/21/2013
%
%   Inputs:         dubprob         global dubprob data structure   
%
%   Outputs:        vars_0           starting guess for optimization 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [vars_0, dubprob] = DubinsFeasibleInitialGuess(dubprob, dubaffeq, dubconvineq)

n = dubprob.numerics.n_vars;

cvx_begin
    variable vars_0(n,1)
    
    minimize(0)
    
    subject to
        dubaffeq(vars_0, dubprob) == 0;
        dubconvineq(vars_0, dubprob) <= 0;
cvx_end

% feasible and more accurate final time
vars_0(end) = norm(dubprob.boundary_values.non_dim.final(1:2)-...
    dubprob.boundary_values.non_dim.init(1:2))/dubprob.scale.V;

dubprob.initial_guess.non_dim = vars_0;

return;