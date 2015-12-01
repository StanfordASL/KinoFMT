%   DubinsAffineEqualityConstraints.m: Computes the values for affine equality
%   constraints
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        12/2/2013
%
%   Inputs:         vars            vector containing problem variable values
%                   dubprob         global variable holding problem parameters
%
%   Outputs:        h_affeq        Value of the affine equality constraints 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Note: Seperate Affine and non-affine constraints later


function h_affeq = DubinsAffineEqualityConstraints(vars, dubprob)

range = dubprob.numerics.range;

% Boundary Values (initial and final states)
h_boundvals = [...
    vars(range.x(1)) - dubprob.boundary_values.non_dim.init(1);...
    vars(range.y(1)) - dubprob.boundary_values.non_dim.init(2);...
    vars(range.theta(1)) - dubprob.boundary_values.non_dim.init(3);...
    vars(range.x(end)) - dubprob.boundary_values.non_dim.final(1);...
    vars(range.y(end)) - dubprob.boundary_values.non_dim.final(2)];
if length(dubprob.boundary_values.non_dim.final) == 3
    h_boundvals = [h_boundvals;...
        vars(range.theta(end)) - dubprob.boundary_values.non_dim.final(3)];
end


% Consolidate
h_affeq = [h_boundvals];

end