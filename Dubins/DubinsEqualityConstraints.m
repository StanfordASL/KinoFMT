%   DubinsEqualityConstraints.m: Computes the values for equality
%   constraints
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        12/1/2013
%
%   Inputs:         vars            vector containing problem variable values
%                   dubprob         global variable holding problem parameters
%
%   Outputs:        h_eq            Value of the equality constraints 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Note: Seperate Affine and non-affine constraints later


function h_eq = DubinsEqualityConstraints(vars, dubprob)

range = dubprob.numerics.range;

% Boundary Values (initial and final states)
h_boundvals = [...
    vars(range.x(1)) - dubprob.boundary_values.non_dim.init(1);...
    vars(range.y(1)) - dubprob.boundary_values.non_dim.init(2);...
    vars(range.theta(1)) - dubprob.boundary_values.non_dim.init(3);...
    vars(range.x(end)) - dubprob.boundary_values.non_dim.final(1);...
    vars(range.y(end)) - dubprob.boundary_values.non_dim.final(2)];
if length(dubprob.boundary_values.non_dim.final) == 3
    h_boundvals = [h_boundvals;...
        vars(range.theta(end)) - dubprob.boundary_values.non_dim.final(3)];
end

% Dynamics
V = dubprob.robot.non_dim.V;
[x,y,theta,u,tf] = DubinsDecomposeVariables(vars, dubprob.numerics.range);
t0 = dubprob.boundary_values.non_dim.t0;
ddt = dubprob.numerics.D/(tf-t0);
h_dynamics = [ddt*x - V*cos(theta);...
    ddt*y - V*sin(theta);...
    ddt*theta - u];

% Consolidate
h_eq = [h_boundvals; h_dynamics];

end