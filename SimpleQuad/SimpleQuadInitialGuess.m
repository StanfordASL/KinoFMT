%   SimpleQuadInitialGuess.m: Generate a starting guess for optimization 
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        Mar 30, 2015
%
%   Inputs:         probinfo         global probinfo data structure   
%
%   Outputs:        vars_0           starting guess for optimization 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [vars_0, probinfo] = SimpleQuadInitialGuess(probinfo)

n = probinfo.numerics.n_nodes;

X0 = probinfo.boundary_values.non_dim.init;
Xf = probinfo.boundary_values.non_dim.final;
ones_vector = ones(n,1);

% interp between initial and goal positions
pos_0 = interp1([1; n], [X0(1:3)'; Xf(1:3)'], [1:n]'); 
x_0 = pos_0(:,1);
y_0 = pos_0(:,2);
z_0 = pos_0(:,3);

% Constant velocity
unit_vector = Xf(1:3) - X0(1:3);
distance = norm(unit_vector);
tf_0 = distance/(probinfo.scale.R/probinfo.scale.t)+...
    probinfo.boundary_values.non_dim.t0;
unit_vector = unit_vector/distance;
speed = distance/(tf_0-probinfo.boundary_values.non_dim.t0);
vel = speed*unit_vector;
vx_0 = vel(1)*ones_vector;
vy_0 = vel(2)*ones_vector;
vz_0 = vel(3)*ones_vector;

% bang-bang Thrust
eta_0 = ones_vector;
ux_0 = unit_vector(1)*ones_vector;
ux_0(floor(n/2)+1:end) = -ux_0(floor(n/2)+1:end);
uy_0 = unit_vector(2)*ones_vector;
uy_0(floor(n/2)+1:end) = -uy_0(floor(n/2)+1:end);
uz_0 = unit_vector(3)*ones_vector;
uz_0(floor(n/2)+1:end) = -uz_0(floor(n/2)+1:end);

% Assemble Variables
vars_0 = SimpleQuadAssembleVariables(x_0, y_0, z_0, vx_0, vy_0,...
    vz_0, ux_0, uy_0, uz_0, eta_0, tf_0);

probinfo.initial_guess.non_dim = vars_0;

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
