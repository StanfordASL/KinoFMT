%   SimpleQuadRobotData.m: Compute derives launch site data and non-dimensionalize 
%
%   Ross Allen, ASL, Stanford University
%   Based on work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        Mar 30, 2015
%
%   Inputs:         probinfo.robot       user defined data about space craft   
%
%   Outputs:        probinfo.robot       updated / non-dimensional data 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function probinfo = SimpleQuadRobotData(probinfo)

veh = probinfo.robot;
scale = probinfo.scale;

% Thrust
probinfo.robot.non_dim.thrustMax = veh.thrustMax*scale.t^2/(scale.R*scale.m);

% mass
probinfo.robot.non_dim.mass = veh.mass/scale.m;

% Angular deviation from nominal
probinfo.robot.uzMax = cosd(veh.maxPitchRoll);

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
