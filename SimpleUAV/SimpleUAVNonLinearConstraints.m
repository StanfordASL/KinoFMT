%   SimpleUAVNonLinearConstraints.m: Compute nonlinear constraints and gradients 
%
%   Ross Allem, ASL, Stanford University
%   Based on work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        4/22/2014
%
%   Inputs:         vars            optimization design variable vector
%                   probinfo         supporting data 
%
%   Outputs:        c, ceq          inequality, equality constraint values
%                   dcdx, dceqdx    inequality, equality constraint Jacobians 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c, ceq, dcdx, dceqdx] = SimpleUAVNonLinearConstraints(vars,probinfo)

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
    [x,y,z,theta,utheta,uz,tf] = SimpleUAVDecomposeVariables(vars, probinfo.numerics.range);
    t0 = probinfo.boundary_values.non_dim.t0;
    Vhor = probinfo.robot.non_dim.Vhor;
    
    % Differentiation Operator (non dim)
    ddt = probinfo.numerics.D/(tf-t0);
    
    % Equations of Motion (kinodynamic constraints)
    ceq = [ddt*x - Vhor*cos(theta);...
           ddt*y - Vhor*sin(theta);...
           ddt*z - uz;...
           ddt*theta - utheta];
       
    % Final Heading
    if length(probinfo.boundary_values.non_dim.final) == 4
        ceq = [ceq;...
            sin(theta(end))-sin(probinfo.boundary_values.non_dim.final(4));...
            cos(theta(end))-cos(probinfo.boundary_values.non_dim.final(4))];
%         ceq = [ceq;...
%             mod(theta(end),probinfo.boundary_values.non_dim.final(4))];
    end
    
    % Number of Constraints
    ncons = length(ceq);

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c, ncons] = NonlinearInequalityConstraints(vars,probinfo)

    % Inequality Constraints
    c = [];
    
    ncons = length(c);

return;