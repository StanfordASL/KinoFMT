%   DubinsInequalityConstraints.m: Computes the values for inequality
%   constraints
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        12/1/2013
%
%   Inputs:         vars            vector containing problem variable values
%                   dubprob         global variable holding problem parameters
%
%   Outputs:        g_ineq           Value of the inequality constraints 
%
%   Note:           This function assumes the convention g(x) <= 0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function g_ineq = DubinsInequalityConstraints(vars, dubprob) 

range = dubprob.numerics.range;

% Control Constraints (Robotic Limitations)
g_control = [...
    -dubprob.robot.non_dim.turnrate - vars(range.u);
    vars(range.u) - dubprob.robot.non_dim.turnrate];

% Travel time >= t0
g_time = dubprob.boundary_values.non_dim.t0 - vars(range.tf);

% Consolidate
g_ineq = [g_control; g_time];

% State Constraints (Environmental Bounds
if ~isinf(dubprob.environment.non_dim.xbounds(1))
    g_ineq = [g_ineq; dubprob.environment.non_dim.xbounds(1) - vars(range.x)];
end
if ~isinf(dubprob.environment.non_dim.ybounds(1))
    g_ineq = [g_ineq; dubprob.environment.non_dim.ybounds(1) - vars(range.y)];
end
if ~isinf(dubprob.environment.non_dim.xbounds(2))
    g_ineq = [g_ineq; vars(range.x) - dubprob.environment.non_dim.xbounds(2)];
end
if ~isinf(dubprob.environment.non_dim.ybounds(2))
    g_ineq = [g_ineq; vars(range.y) - dubprob.environment.non_dim.ybounds(2)];
end
end