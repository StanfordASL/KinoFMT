%   DubinsRobotData.m: Compute derives launch site data and non-dimensionalize 
%
%   Ross Allen, ASL, Stanford University
%   Based on work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        10/18/2013
%
%   Inputs:         dubprob.robot       user launch site data   
%
%   Outputs:        dubprob.robot       updated / non-dimensional data 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dubprob = DubinsRobotData(dubprob)

veh = dubprob.robot;
scale = dubprob.scale;

% Velocity
dubprob.robot.non_dim.V = veh.V/scale.V;

% Turn rate
dubprob.robot.non_dim.turnrate = veh.turnrate*pi()/180*scale.t;

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%