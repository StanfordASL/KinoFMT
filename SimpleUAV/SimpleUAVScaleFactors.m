%   SimpleUAVScaleFactors.m: Calculate quantities used for non-dimensionalization to
%   condition problem for better numerical calculations
%
%   Author: Ross Allen, ASL, Stanford University
%   Based on Work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        4/17/2013
%
%   Inputs:         probinfo            user defined parameters of dubins problem       
%
%   Outputs:        probinfo.scale      non-dimensionalization quantities               

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function probinfo = SimpleUAVScaleFactors(probinfo)
bv = probinfo.boundary_values;

% Check for well-defined problem
R = probinfo.robot.Vhor/probinfo.robot.turnrate;
if R <= 0 || isinf(R)
    disp(' ')
    disp('Problem definition is poorly defined.')
    disp('Must have horizontal velocity (Vhor) > 0 and turnrate > 0')
    disp(' ')
    probinfo.scale.R = NaN;
    probinfo.scale.t = NaN;
    return;
end


% Length and Time
probinfo.scale.R = R;
probinfo.scale.t = R/probinfo.robot.Vhor;
probinfo.scale.V = probinfo.scale.R/probinfo.scale.t;

                            
return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%