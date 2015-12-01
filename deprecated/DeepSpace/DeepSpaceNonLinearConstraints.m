%   DeepSpaceNonLinearConstraints.m: Compute nonlinear constraints and gradients 
%
%   Ross Allem, ASL, Stanford University
%   Based on work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        12/7/2013
%
%   Inputs:         vars            optimization design variable vector
%                   probinfo         supporting data 
%
%   Outputs:        c, ceq          inequality, equality constraint values
%                   dcdx, dceqdx    inequality, equality constraint Jacobians 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c, ceq, dcdx, dceqdx] = DeepSpaceNonLinearConstraints(vars,probinfo)

    % Equality Constraints
    [ceq, ncons] = NonlinearEqualityConstraints(vars, probinfo);
    dceqdx = ComplexStepGradients(@NonlinearEqualityConstraints,vars,ncons, probinfo);
    
    % Inequality Constraints
    [c, ncons] = NonlinearInequalityConstraints(vars, probinfo);
    % dcdx = ComplexStepGradients(@NonlinearInequalityConstraints,vars,ncons);
    dcdx = [];
    
return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ceq, ncons] = NonlinearEqualityConstraints(vars,probinfo)

    % Extract Variables
    [x,y,z,xdot,ydot,zdot,ux,uy,uz,eta,tf] = DeepSpaceDecomposeVariables(vars, probinfo.numerics.range);
    t0 = probinfo.boundary_values.non_dim.t0;
    Tmax = probinfo.robot.non_dim.ThrustMax;
    mass = probinfo.robot.non_dim.mass;
    
    % Differentiation Operator (non dim)
    ddt = probinfo.numerics.D/(tf-t0);
    
    % Equations of Motion (kinodynamic constraints)
    ceq = [ddt*x - xdot;...
           ddt*y - ydot;...
           ddt*z - zdot;...
           ddt*xdot - (Tmax/mass).*eta.*ux;...
           ddt*ydot - (Tmax/mass).*eta.*uy;...
           ddt*zdot - (Tmax/mass).*eta.*uz;];
       
    % Unit vector constraint
    ceq = [ceq;...
        ux.^2 + uy.^2 + uz.^2 - ones(probinfo.numerics.n_nodes,1)];
    
    % Number of Constraints
    ncons = length(ceq);

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c, ncons] = NonlinearInequalityConstraints(vars,probinfo)

    % Inequality Constraints
    c = [];
    
    ncons = length(c);

return;