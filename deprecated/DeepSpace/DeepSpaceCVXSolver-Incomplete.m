%   DeepSpaceCVXSolver.m: solve the deep space spacecraft problem with CVX
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        Mar 14, 2014
%
%   Inputs:         
%
%   Outputs:      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% !!!!!!!!!!!!!!!!!!!!!!!!!!!! NOTE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% I don't think it is possible to formulate this as a convex problem
% due to the free final time constraint. Saving this unfinised code incase
% I decided otherwise later

function [vars_opt J_opt exitflag solveroutput] = DeepSpaceCVXSolver(probinfo)

n = probinfo.numerics.n_vars;
t0 = probinfo.boundary_values.non_dim.t0;
Tmax = probinfo.robot.non_dim.ThrustMax;
mass = probinfo.robot.non_dim.mass;
ddt = probinfo.numerics.D/(tf-t0);

cvx_begin
    variables vars(n,1)
    [x,y,z,xdot,ydot,zdot,ux,uy,uz,eta,tf] = ...
        DeepSpaceDecomposeVariables(vars, probinfo.numerics.range);
    minimize( vars(n,1) )
    subject to
        ddt*x - xdot == 0;
        ddt*y - ydot == 0;
        ddt*z - zdot == 0;
        ddt*xdot - (Tmax/mass).*eta.*ux == 0;
        ddt*ydot - (Tmax/mass).*eta.*uy == 0;
        ddt*zdot - (Tmax/mass).*eta.*uz == 0;
        
   
return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%