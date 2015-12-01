%   DubinsEnvironmentData.m: Compute derives launch site data and non-dimensionalize 
%
%   Ross Allen, ASL, Stanford University
%   Based on work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        10/18/2013
%
%   Inputs:         dubprob.environment       user launch site data   
%
%   Outputs:        dubprob.environment       updated / non-dimensional data 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dubprob = DubinsEnvironmentData(dubprob)

env = dubprob.environment;
scale = dubprob.scale;

% Enivorment Boundaries
dubprob.environment.non_dim.xbounds = env.xbounds/scale.R;
dubprob.environment.non_dim.ybounds = env.ybounds/scale.R;

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%