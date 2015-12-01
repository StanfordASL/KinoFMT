%   SimpleUAVRobotData.m: Compute derives launch site data and non-dimensionalize 
%
%   Ross Allen, ASL, Stanford University
%   Based on work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        4/22/2014
%
%   Inputs:         probinfo.robot       user defined data about space craft   
%
%   Outputs:        probinfo.robot       updated / non-dimensional data 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function probinfo = SimpleUAVRobotData(probinfo)

veh = probinfo.robot;
scale = probinfo.scale;

% Velocity
probinfo.robot.non_dim.Vhor = veh.Vhor/scale.V;

% Max turn rate
probinfo.robot.non_dim.turnrate = veh.turnrate*scale.t;

% Max climb rate
probinfo.robot.non_dim.climbrate = veh.climbrate/scale.V;

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%