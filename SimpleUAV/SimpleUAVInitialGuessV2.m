%   SimpleUAVInitialGuessV2.m: Generate a starting guess for optimization 
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        4/29/2014
%
%   Inputs:         probinfo         global probinfo data structure   
%
%   Outputs:        vars_0           starting guess for optimization 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [vars_0, probinfo] = SimpleUAVInitialGuessV2(probinfo)

n = probinfo.numerics.n_nodes;
climbrate = probinfo.robot.climbrate;
turnrate = probinfo.robot.turnrate;
Vhor = probinfo.robot.Vhor;

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
nloops = min(floor((Xf(3)-X0(3))/(sqrt((Xf(2)-X0(2))^2 + (Xf(1)-X0(1))^2)/...
    Vhor*climbrate)),...
    ceil((Xf(3)-X0(3))*turnrate/(2*pi()*climbrate)));
loopsign = sign(Xf(4));
if loopsign == 0
    loopsign = 1;
end
if length(Xf) == 4
    theta_0 = interp1([1; n], [X0(4); loopsign*nloops*2*pi()+Xf(4)], [1:n]');
else
    theta_0 = interp1([1; n], [X0(4); nloops*2*pi()+atan2(Xf(2)-X0(2), Xf(1)-X0(1))], [1:n]');
end

% Control (turnrate)
t = probinfo.scale.t*(tf_0 - t0)*probinfo.numerics.t;   % s
t = t + probinfo.scale.t*t0;
utheta_0 = [diff(theta_0)./diff(t); 0];
utheta_0 = min(utheta_0, turnrate);
utheta_0 = max(utheta_0, -turnrate);


% Control (climbrate)
uz_0 = (Xf(3)-X0(3))/(tf_0-probinfo.boundary_values.non_dim.t0)*...
    ones_vector;
uz_0 = min(uz_0, climbrate);
uz_0 = max(uz_0, -climbrate);

% Assemble Variables
vars_0 = SimpleUAVAssembleVariables(x_0, y_0, z_0, theta_0,...
    utheta_0, uz_0, tf_0);

probinfo.initial_guess.non_dim = vars_0;

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%