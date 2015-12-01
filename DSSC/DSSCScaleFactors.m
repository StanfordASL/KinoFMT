%   DSSCScaleFactors.m: Calculate quantities used for non-dimensionalization to
%   condition problem for better numerical calculations
%
%   Author: Ross Allen, ASL, Stanford University
%   Based on Work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        9/24/2013
%
%   Inputs:         probinfo            user defined parameters of dubins problem       
%
%   Outputs:        probinfo.scale      non-dimensionalization quantities               

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function probinfo = DSSCScaleFactors(probinfo)
bv = probinfo.boundary_values;

% Check for well-defined problem
a = probinfo.robot.thrustMax/probinfo.robot.mass;
if a <= 0 || isinf(a) || probinfo.robot.mass <=0
    disp(' ')
    disp('Problem definition is poorly defined.')
    disp('Must have thrustMax > 0 and mass > 0')
    disp(' ')
    probinfo.scale.R = NaN;
    probinfo.scale.t = NaN;
    return;
end

% Mass
probinfo.scale.m = probinfo.robot.mass;

% Length and Time
R = norm([bv.x0-bv.xf, bv.y0-bv.yf, bv.z0-bv.zf]);
if R == 0
    if isfield(probinfo.boundary_values,'xdotf') && ...
            isfield(probinfo.boundary_values,'ydotf') &&...
            isfield(probinfo.boundary_values,'zdotf')
        V = norm([bv.xdot0-bv.xdotf, bv.ydot0-bv.ydotf, bv.zdot0-bv.zdotf]);
        if V == 0
            disp(' ')
            disp('Problem definition is trivial')
            disp('Need different initial and final conditions')
            disp(' ')
            probinfo.scale.R = NaN;
            probinfo.scale.t = NaN;
        else
            probinfo.scale.t = V/a;
            probinfo.scale.R = V*probinfo.scale.t;
        end
    else
        disp(' ')
        disp('Problem definition is trivial')
        disp('Need different initial and final conditions')
        disp(' ')
        probinfo.scale.R = NaN;
        probinfo.scale.t = NaN;
    end
else
    probinfo.scale.R = R;
    probinfo.scale.t = sqrt(2*probinfo.scale.R/a);
end 
                            
return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
