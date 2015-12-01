%   DubinsScaleFactors.m: Calculate quantities used for non-dimensionalization to
%   condition problem for better numerical calculations
%
%   Author: Ross Allen, ASL, Stanford University
%   Based on Work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        10/17/2013
%
%   Inputs:         dubprob            user defined parameters of dubins problem       
%
%   Outputs:        dubprob.scale      non-dimensionalization quantities               

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dubprob = DubinsScaleFactors(dubprob)

dubprob.scale.R = dubprob.robot.V/(dubprob.robot.turnrate*pi/180);
dubprob.scale.V = dubprob.robot.V;                                       % (m/s)
dubprob.scale.t = dubprob.scale.R/dubprob.scale.V;                          % (s)


return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%