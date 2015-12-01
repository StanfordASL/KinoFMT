%   DubinsLinearConstraints.m: Compute linear equality and inequality constraints,
%                               including upper and lower bounds
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        10/21/2013
%
%   Inputs:         dubprob         global data structure     
%
%   Outputs:        A, b            linear inequality constraint matrix, vector (Ax <= b)
%                   Aeq, beq        linear equality constraint matrix, vector (Aeqx = beq)
%                   lb, ub          global lower and upper bound vectors 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [A, b, Aeq, beq, lb, ub] = DubinsLinearConstraints(dubprob)

    nvars = dubprob.numerics.n_vars; 
    range = dubprob.numerics.range;
    
    lb = -Inf.*ones(nvars,1); ub = -lb;
    A = []; b = []; Aeq = []; beq = [];

    % State Constraints (Environmental Bounds)
    lb(range.x) = dubprob.environment.non_dim.xbounds(1);
    lb(range.y) = dubprob.environment.non_dim.ybounds(1);
    ub(range.x) = dubprob.environment.non_dim.xbounds(2);
    ub(range.y) = dubprob.environment.non_dim.ybounds(2);
    
    % Control Constraints (Robotic Limitations)
    lb(range.u) = -dubprob.robot.non_dim.turnrate;
    ub(range.u) = dubprob.robot.non_dim.turnrate;
    
    % Boundary Values (initial and final states)
    ub(range.x(1)) = dubprob.boundary_values.non_dim.init(1);
    lb(range.x(1)) = dubprob.boundary_values.non_dim.init(1);
    ub(range.y(1)) = dubprob.boundary_values.non_dim.init(2);
    lb(range.y(1)) = dubprob.boundary_values.non_dim.init(2);
    ub(range.theta(1)) = dubprob.boundary_values.non_dim.init(3);
    lb(range.theta(1)) = dubprob.boundary_values.non_dim.init(3);
    ub(range.x(end)) = dubprob.boundary_values.non_dim.final(1);
    lb(range.x(end)) = dubprob.boundary_values.non_dim.final(1);
    ub(range.y(end)) = dubprob.boundary_values.non_dim.final(2);
    lb(range.y(end)) = dubprob.boundary_values.non_dim.final(2);
    if length(dubprob.boundary_values.non_dim.final) == 3
        ub(range.theta(end)) = dubprob.boundary_values.non_dim.final(3);
        lb(range.theta(end)) = dubprob.boundary_values.non_dim.final(3);
    end
    
    % travel time >= t0
    lb(dubprob.numerics.range.tf) = dubprob.boundary_values.non_dim.t0;
          
return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%