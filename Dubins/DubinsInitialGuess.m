%   DubinsInitialGuess.m: Generate a starting guess for optimization 
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        10/21/2013
%
%   Inputs:         dubprob         global dubprob data structure   
%
%   Outputs:        vars_0           starting guess for optimization 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [vars_0, dubprob] = DubinsInitialGuess(dubprob)

n = dubprob.numerics.n_nodes;

X0 = dubprob.boundary_values.non_dim.init;
Xf = dubprob.boundary_values.non_dim.final;
ones_vector = ones(n,1);

% interp between initial and goal positions
pos_0 = interp1([1; n], [X0(1:2)'; Xf(1:2)'], [1:n]'); 
x_0 = pos_0(:,1);
y_0 = pos_0(:,2);

% Constant heading
theta_0 = atan2(Xf(2)-X0(2), Xf(1)-X0(1))*ones_vector;

% Control (turnrate)
u_0 = 0*ones_vector;

% Final time
tf_0 = norm(Xf(1:2)-X0(1:2))/dubprob.scale.V;

% Assemble Variables
vars_0 = DubinsAssembleVariables(x_0, y_0, theta_0, u_0, tf_0);

dubprob.initial_guess.non_dim = vars_0;

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%