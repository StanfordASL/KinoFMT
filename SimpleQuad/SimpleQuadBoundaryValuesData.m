%   SimpleQuadBoundaryValuesData.m: Non-dimensionalize boundary values 
%
%   Ross Allen, ASL, Stanford University
%   Based on work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        Mar 30, 2015
%
%   Inputs:         probinfo.boundary_values        user defined initial and final states data   
%
%   Outputs:        probinfo.boundary_values        updated / non-dimensional data 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function probinfo = SimpleQuadBoundaryValuesData(probinfo)

bv = probinfo.boundary_values;
scale = probinfo.scale;

% Initial State: 
probinfo.boundary_values.init = [bv.x0; bv.y0; bv.z0;...
     bv.vx0; bv.vy0; bv.vz0];
probinfo.boundary_values.non_dim.init = [bv.x0/scale.R;...
    bv.y0/scale.R; bv.z0/scale.R; bv.vx0*scale.t/scale.R;...
    bv.vy0*scale.t/scale.R; bv.vz0*scale.t/scale.R];                            % non-dim
probinfo.boundary_values.non_dim.t0 = bv.t0/scale.t;


% Final State
probinfo.boundary_values.final = [bv.xf; bv.yf; bv.zf];
probinfo.boundary_values.non_dim.final = [bv.xf/scale.R;...
    bv.yf/scale.R; bv.zf/scale.R];
if isfield(bv, 'vxf')
    probinfo.boundary_values.final = [probinfo.boundary_values.final; ...
        bv.vxf; bv.vyf; bv.vzf];
    probinfo.boundary_values.non_dim.final = [probinfo.boundary_values.non_dim.final; ...
        bv.vxf*scale.t/scale.R; bv.vyf*scale.t/scale.R;...
        bv.vzf*scale.t/scale.R];
end
if isfield(bv, 'etaf')
    probinfo.boundary_values.final = [probinfo.boundary_values.final; ...
        bv.etaf];
    probinfo.boundary_values.non_dim.final = [probinfo.boundary_values.non_dim.final; ...
        bv.etaf];
end

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
