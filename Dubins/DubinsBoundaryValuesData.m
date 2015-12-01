%   DubinsBoundaryValuesData.m: Compute derives launch site data and non-dimensionalize 
%
%   Ross Allen, ASL, Stanford University
%   Based on work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        10/18/2013
%
%   Inputs:         dubprob.boundary_values        user launch site data   
%
%   Outputs:        dubprob.boundary_values        updated / non-dimensional data 
%
%   CHANGES: Ross Allen
%   - Changed initial velocity to be in xy-plane instead of yz-plane

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dubprob = DubinsBoundaryValuesData(dubprob)

bv = dubprob.boundary_values;

% Initial State: note heading mapped to [-pi pi]
dubprob.boundary_values.init = [bv.x0; bv.y0; wrapTo180(bv.theta0)];
dubprob.boundary_values.non_dim.init = [bv.x0/dubprob.scale.R;...
    bv.y0/dubprob.scale.R; wrapToPi(bv.theta0*pi()/180)];                            % non-dim
dubprob.boundary_values.non_dim.t0 = bv.t0/dubprob.scale.t;

% Final State: note heading mapped to [-pi pi]
if isfield(bv, 'thetaf')
    dubprob.boundary_values.final = [bv.xf; bv.yf; wrapTo180(bv.thetaf)];
    dubprob.boundary_values.non_dim.final = [bv.xf/dubprob.scale.R;...
        bv.yf/dubprob.scale.R; wrapToPi(bv.thetaf*pi()/180)];                            % non-dim
else
    dubprob.boundary_values.final = [bv.xf; bv.yf];
    dubprob.boundary_values.non_dim.final = [bv.xf/dubprob.scale.R;...
        bv.yf/dubprob.scale.R];                            % non-dim

end

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%