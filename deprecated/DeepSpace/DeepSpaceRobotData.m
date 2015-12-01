%   DeepSpaceRobotData.m: Compute derives launch site data and non-dimensionalize 
%
%   Ross Allen, ASL, Stanford University
%   Based on work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        12/6/2013
%
%   Inputs:         probinfo.robot       user defined data about space craft   
%
%   Outputs:        probinfo.robot       updated / non-dimensional data 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function probinfo = DeepSpaceRobotData(probinfo)

veh = probinfo.robot;
scale = probinfo.scale;

% Thrust
probinfo.robot.non_dim.ThrustMax = veh.ThrustMax*scale.t^2/(scale.R*scale.m);

% mass
probinfo.robot.non_dim.mass = veh.mass/scale.m;

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%