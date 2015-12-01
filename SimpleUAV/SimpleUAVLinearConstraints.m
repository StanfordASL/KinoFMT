%   SimpleUAVLinearConstraints.m: Compute linear equality and inequality constraints,
%                               including upper and lower bounds
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        4/22/2014
%
%   Inputs:         probinfo         global data structure     
%
%   Outputs:        A, b            linear inequality constraint matrix, vector (Ax <= b)
%                   Aeq, beq        linear equality constraint matrix, vector (Aeqx = beq)
%                   lb, ub          global lower and upper bound vectors 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [A, b, Aeq, beq, lb, ub] = SimpleUAVLinearConstraints(probinfo)

    nvars = probinfo.numerics.n_vars; 
    range = probinfo.numerics.range;
    
    lb = -Inf.*ones(nvars,1); ub = -lb;
    A = []; b = []; Aeq = []; beq = [];

    % Control Constraints (Robotic Limitations)
    lb(range.utheta) = -probinfo.robot.non_dim.turnrate;
    ub(range.utheta) = probinfo.robot.non_dim.turnrate;
    lb(range.uz) = -probinfo.robot.non_dim.climbrate;
    ub(range.uz) = probinfo.robot.non_dim.climbrate;
    
    % Boundary Values (initial and final states)
    ub(range.x(1)) = probinfo.boundary_values.non_dim.init(1);
    lb(range.x(1)) = probinfo.boundary_values.non_dim.init(1);
    ub(range.y(1)) = probinfo.boundary_values.non_dim.init(2);
    lb(range.y(1)) = probinfo.boundary_values.non_dim.init(2);
    ub(range.z(1)) = probinfo.boundary_values.non_dim.init(3);
    lb(range.z(1)) = probinfo.boundary_values.non_dim.init(3);
    ub(range.theta(1)) = probinfo.boundary_values.non_dim.init(4);
    lb(range.theta(1)) = probinfo.boundary_values.non_dim.init(4);
    ub(range.x(end)) = probinfo.boundary_values.non_dim.final(1);
    lb(range.x(end)) = probinfo.boundary_values.non_dim.final(1);
    ub(range.y(end)) = probinfo.boundary_values.non_dim.final(2);
    lb(range.y(end)) = probinfo.boundary_values.non_dim.final(2);
    ub(range.z(end)) = probinfo.boundary_values.non_dim.final(3);
    lb(range.z(end)) = probinfo.boundary_values.non_dim.final(3);
    
    % Boundary Values: Final Heading
    % NOTE: this may lead to 'wind-up' problem since it enforces a single
    % value instead of any equivalent angle (e.g. 0 == 2pi). May want to
    % comment out and enforce a nonlinear constraint
%     if length(probinfo.boundary_values.non_dim.final) == 4
%         ub(range.theta(end)) = probinfo.boundary_values.non_dim.final(4);
%         lb(range.theta(end)) = probinfo.boundary_values.non_dim.final(4);
%     end
    
    % travel time >= t0
    lb(probinfo.numerics.range.tf) = probinfo.boundary_values.non_dim.t0;
          
return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%