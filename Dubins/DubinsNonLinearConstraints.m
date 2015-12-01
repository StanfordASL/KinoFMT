%   DubinsNonLinearConstraints.m: Compute nonlinear constraints and gradients 
%
%   Ross Allem, ASL, Stanford University
%   Based on work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        10/21/2013
%
%   Inputs:         vars            optimization design variable vector
%                   dubprob         supporting data 
%
%   Outputs:        c, ceq          inequality, equality constraint values
%                   dcdx, dceqdx    inequality, equality constraint Jacobians 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c, ceq, dcdx, dceqdx] = DubinsNonLinearConstraints(vars,dubprob)

    % Equality Constraints
    [ceq, ncons] = NonlinearEqualityConstraints(vars, dubprob);
    dceqdx = ComplexStepGradients(@NonlinearEqualityConstraints,vars,ncons, dubprob);
    
    % Inequality Constraints
    [c, ncons] = NonlinearInequalityConstraints(vars, dubprob);
    % dcdx = ComplexStepGradients(@NonlinearInequalityConstraints,vars,ncons);
    dcdx = [];
    
return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ceq, ncons] = NonlinearEqualityConstraints(vars,dubprob)

    % Extract Variables
    V = dubprob.robot.non_dim.V;
    [x,y,theta,u,tf] = DubinsDecomposeVariables(vars, dubprob.numerics.range);
    t0 = dubprob.boundary_values.non_dim.t0;
    
    % Differentiation Operator (non dim)
    ddt = dubprob.numerics.D/(tf-t0);
    
    % Equations of Motion
    ceq = [ddt*x - V*cos(theta);...
        ddt*y - V*sin(theta);...
        ddt*theta - u];
    
    % Goal Heading Constraint
%     headf = wrapTo2Pi(theta(end)) - ...
%         dubprob.boundary_values.non_dim.final(3);
%     headf = [sin(theta(end)) - sin(dubprob.boundary_values.non_dim.final(3));...
%              cos(theta(end)) - cos(dubprob.boundary_values.non_dim.final(3))];   
%     ceq = [ceq; headf];
    
    % Number of Constraints
    ncons = length(ceq);

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c, ncons] = NonlinearInequalityConstraints(vars,dubprob)

    % Inequality Constraints
    c = [];
    
    ncons = length(c);

return;

