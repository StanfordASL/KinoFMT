%   DSSCLinearConstraints.m: Compute linear equality and inequality constraints,
%                               including upper and lower bounds
%
%   Ross Allen, ASL, Stanford University 
%
%  Started:        9/24/2014
%
%   Inputs:         probinfo         global data structure     
%
%   Outputs:        A, b            linear inequality constraint matrix, vector (Ax <= b)
%                   Aeq, beq        linear equality constraint matrix, vector (Aeqx = beq)
%                   lb, ub          global lower and upper bound vectors 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [A, b, Aeq, beq, lb, ub] = DSSCLinearConstraints(probinfo)

    nvars = probinfo.numerics.n_vars; 
    range = probinfo.numerics.range;
    ones_vector = ones(probinfo.numerics.n_nodes,1);
    
    lb = -Inf.*ones(nvars,1); ub = -lb;
    A = []; b = []; Aeq = []; beq = [];

    % State Constraints (Environmental Bounds)
%     lb(range.x) = probinfo.environment.non_dim.xbounds(1);
%     lb(range.y) = probinfo.environment.non_dim.ybounds(1);
%     ub(range.x) = probinfo.environment.non_dim.xbounds(2);
%     ub(range.y) = probinfo.environment.non_dim.ybounds(2);
    
    % Throttle Constraints
    lb(range.eta) = 0;
    ub(range.eta) = 1;
    
    % Thrust Vector Constraint
%     ub(range.ux) = ones_vector;                % ux(i) <= 1.0
%     lb(range.ux) = -ones_vector;               % ux(i) >= -1.0
%     ub(range.uy) = ones_vector;                % uy(i) <= 1.0
%     lb(range.uy) = -ones_vector;               % uy(i) >= -1.0
%     ub(range.uz) = ones_vector;                % uz(i) <= 1.0
%     lb(range.uz) = -ones_vector;               % uz(i) >= -1.0
    
    % Boundary Values (initial and final states)
    ub(range.x(1)) = probinfo.boundary_values.non_dim.init(1);
    lb(range.x(1)) = probinfo.boundary_values.non_dim.init(1);
    ub(range.y(1)) = probinfo.boundary_values.non_dim.init(2);
    lb(range.y(1)) = probinfo.boundary_values.non_dim.init(2);
    ub(range.z(1)) = probinfo.boundary_values.non_dim.init(3);
    lb(range.z(1)) = probinfo.boundary_values.non_dim.init(3);
    ub(range.xdot(1)) = probinfo.boundary_values.non_dim.init(4);
    lb(range.xdot(1)) = probinfo.boundary_values.non_dim.init(4);
    ub(range.ydot(1)) = probinfo.boundary_values.non_dim.init(5);
    lb(range.ydot(1)) = probinfo.boundary_values.non_dim.init(5);
    ub(range.zdot(1)) = probinfo.boundary_values.non_dim.init(6);
    lb(range.zdot(1)) = probinfo.boundary_values.non_dim.init(6);
    ub(range.x(end)) = probinfo.boundary_values.non_dim.final(1);
    lb(range.x(end)) = probinfo.boundary_values.non_dim.final(1);
    ub(range.y(end)) = probinfo.boundary_values.non_dim.final(2);
    lb(range.y(end)) = probinfo.boundary_values.non_dim.final(2);
    ub(range.z(end)) = probinfo.boundary_values.non_dim.final(3);
    lb(range.z(end)) = probinfo.boundary_values.non_dim.final(3);
    if length(probinfo.boundary_values.non_dim.final) > 3
        ub(range.xdot(end)) = probinfo.boundary_values.non_dim.final(4);
        lb(range.xdot(end)) = probinfo.boundary_values.non_dim.final(4);
        ub(range.ydot(end)) = probinfo.boundary_values.non_dim.final(5);
        lb(range.ydot(end)) = probinfo.boundary_values.non_dim.final(5);
        ub(range.zdot(end)) = probinfo.boundary_values.non_dim.final(6);
        lb(range.zdot(end)) = probinfo.boundary_values.non_dim.final(6);
    end
    if length(probinfo.boundary_values.non_dim.final) > 6
        ub(range.eta(end)) = probinfo.boundary_values.non_dim.final(7);
        lb(range.eta(end)) = probinfo.boundary_values.non_dim.final(7);
    end
    
    % travel time >= t0
    lb(probinfo.numerics.range.tf) = probinfo.boundary_values.non_dim.t0;
          
return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
