%   SimpleUAVBoundaryValuesData.m: Non-dimensionalize boundary values 
%
%   Ross Allen, ASL, Stanford University
%   Based on work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        4/22/2014
%
%   Inputs:         probinfo.boundary_values        user defined initial and final states data   
%
%   Outputs:        probinfo.boundary_values        updated / non-dimensional data 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function probinfo = SimpleUAVBoundaryValuesData(probinfo)

bv = probinfo.boundary_values;
scale = probinfo.scale;

% Initial State: note heading mapped to [-pi pi]
probinfo.boundary_values.init = [bv.x0; bv.y0; bv.z0; wrapToPi(bv.theta0)];
probinfo.boundary_values.non_dim.init = [bv.x0/scale.R;...
    bv.y0/scale.R; bv.z0/scale.R; wrapToPi(bv.theta0)];                            % non-dim
probinfo.boundary_values.non_dim.t0 = bv.t0/scale.t;

% Final State: note heading mapped to [-pi pi]
if isfield(bv, 'thetaf')
    probinfo.boundary_values.final = [bv.xf; bv.yf; bv.zf; wrapToPi(bv.thetaf)];
    probinfo.boundary_values.non_dim.final = [bv.xf/probinfo.scale.R;...
        bv.yf/probinfo.scale.R; bv.zf/probinfo.scale.R; wrapToPi(bv.thetaf)];                            % non-dim
else
    probinfo.boundary_values.final = [bv.xf; bv.yf; bv.zf];
    probinfo.boundary_values.non_dim.final = [bv.xf/probinfo.scale.R;...
        bv.yf/probinfo.scale.R; bv.zf/probinfo.scale.R];                            % non-dim
end

return;