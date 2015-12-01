%   DubinsDynamicConstraints.m: Computes dynamics of dubins to enforce as a
%   constraint
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        12/1/2013
%
%   Inputs:         vars            vector containing problem variable values
%                   dubprob         global variable holding problem parameters
%
%   Outputs:        h_dynamics      Dynamics evaluated as dx/dt - f(x,t) = 0 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Note: Seperate Affine and non-affine constraints later


function h_dynamics = DubinsDynamicConstraints(vars, dubprob)

% Dynamics
V = dubprob.robot.non_dim.V;
[x,y,theta,u,tf] = DubinsDecomposeVariables(vars, dubprob.numerics.range);
t0 = dubprob.boundary_values.non_dim.t0;
ddt = dubprob.numerics.D/(tf-t0);
h_dynamics = [ddt*x - V*cos(theta);...
    ddt*y - V*sin(theta);...
    ddt*theta - u];

end