%   SimpleUAVInitialGuess.m: Generate a starting guess for optimization 
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        4/22/2014
%
%   Inputs:         probinfo         global probinfo data structure   
%
%   Outputs:        vars_0           starting guess for optimization 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [vars_0, probinfo] = SimpleUAVInitialGuess(probinfo)

n = probinfo.numerics.n_nodes;

X0 = probinfo.boundary_values.non_dim.init;
Xf = probinfo.boundary_values.non_dim.final;
t0 = probinfo.boundary_values.non_dim.t0;
ones_vector = ones(n,1);

% interp between initial and goal positions
pos_0 = interp1([1; n], [X0(1:3)'; Xf(1:3)'], [1:n]'); 
x_0 = pos_0(:,1);
y_0 = pos_0(:,2);
z_0 = pos_0(:,3);

% Final time
tf_0 = norm(Xf(1:3)-X0(1:3))/probinfo.scale.V + ...
    probinfo.boundary_values.non_dim.t0;

% Heading
theta_0 = atan2(Xf(2)-X0(2), Xf(1)-X0(1))*ones_vector;


% Control (turnrate)
utheta_0 = 0*ones_vector;

% Control (climbrate)
uz_0 = (Xf(3)-X0(3))/(tf_0-probinfo.boundary_values.non_dim.t0)*...
    ones_vector;

% Assemble Variables
vars_0 = SimpleUAVAssembleVariables(x_0, y_0, z_0, theta_0,...
    utheta_0, uz_0, tf_0);

probinfo.initial_guess.non_dim = vars_0;

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%