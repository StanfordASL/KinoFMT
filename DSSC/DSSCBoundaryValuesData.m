%   DSSCBoundaryValuesData.m: Non-dimensionalize boundary values 
%
%   Ross Allen, ASL, Stanford University
%   Based on work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        9/24/2014
%
%   Inputs:         probinfo.boundary_values        user defined initial and final states data   
%
%   Outputs:        probinfo.boundary_values        updated / non-dimensional data 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function probinfo = DSSCBoundaryValuesData(probinfo)

bv = probinfo.boundary_values;
scale = probinfo.scale;

% Initial State: note heading mapped to [-pi pi]
probinfo.boundary_values.init = [bv.x0; bv.y0; bv.z0;...
     bv.xdot0; bv.ydot0; bv.zdot0];
probinfo.boundary_values.non_dim.init = [bv.x0/scale.R;...
    bv.y0/scale.R; bv.z0/scale.R; bv.xdot0*scale.t/scale.R;...
    bv.ydot0*scale.t/scale.R; bv.zdot0*scale.t/scale.R];                            % non-dim
probinfo.boundary_values.non_dim.t0 = bv.t0/scale.t;


% Final State
probinfo.boundary_values.final = [bv.xf; bv.yf; bv.zf];
probinfo.boundary_values.non_dim.final = [bv.xf/scale.R;...
    bv.yf/scale.R; bv.zf/scale.R];
if isfield(bv, 'xdotf')
    probinfo.boundary_values.final = [probinfo.boundary_values.final; ...
        bv.xdotf; bv.ydotf; bv.zdotf];
    probinfo.boundary_values.non_dim.final = [probinfo.boundary_values.non_dim.final; ...
        bv.xdotf*scale.t/scale.R; bv.ydotf*scale.t/scale.R;...
        bv.zdotf*scale.t/scale.R];
end
if isfield(bv, 'etaf')
    probinfo.boundary_values.final = [probinfo.boundary_values.final; ...
        bv.etaf];
    probinfo.boundary_values.non_dim.final = [probinfo.boundary_values.non_dim.final; ...
        bv.etaf];
end

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
