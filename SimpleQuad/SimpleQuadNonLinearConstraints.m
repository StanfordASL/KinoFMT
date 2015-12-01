%   SimpleQuadNonLinearConstraints.m: Compute nonlinear constraints and gradients 
%
%   Ross Allem, ASL, Stanford University
%   Based on work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        Mar 30, 2015
%
%   Inputs:         vars            optimization design variable vector
%                   probinfo         supporting data 
%
%   Outputs:        c, ceq          inequality, equality constraint values
%                   dcdx, dceqdx    inequality, equality constraint Jacobians 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c, ceq, dcdx, dceqdx] = SimpleQuadNonLinearConstraints(vars,probinfo)

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
    [x,y,z,vx,vy,vz,ux,uy,uz,eta,tf] = SimpleQuadDecomposeVariables(vars, probinfo.numerics.range);
    t0 = probinfo.boundary_values.non_dim.t0;
    Tmax = probinfo.robot.non_dim.thrustMax;
    mass = probinfo.robot.non_dim.mass;
    g = 9.81*probinfo.scale.t^2/probinfo.scale.R;
    
    % Differentiation Operator (non dim)
    ddt = probinfo.numerics.D/(tf-t0);
    
    % Equations of Motion (kinodynamic constraints)
    ceq = [ddt*x - vx;...
           ddt*y - vy;...
           ddt*z - vz;...
           ddt*vx - (Tmax/mass).*eta.*ux;...
           ddt*vy - (Tmax/mass).*eta.*uy;...
           ddt*vz - (Tmax/mass).*eta.*uz - g*ones(probinfo.numerics.n_nodes,1);];
       
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
