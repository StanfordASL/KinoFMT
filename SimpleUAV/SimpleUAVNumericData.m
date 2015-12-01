%   SimpleUAVNumericData.m: Compute Chebyshev differentiation, non-dimensional time vector, 
%                   variable and constraint counts & ranges
%
%   Author: Ross Allen, ASL, Stanford University
%   Based on Work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        4/17/2014
%
%   Inputs:         n           number of design points per stage      
%
%   Outputs:        .t           cosine-spaced time vector
%                   .D           differentiation matrix 
%                   .w           Clenshaw-Curtis weights
%                   .n_vars      total number of variables 
%                   .N           order of polynomial approximation
%                   .range       indices for variable specific variables
%
%  Notes:           Previous versions included an integration matrix.
%  Francisco Capristran found numerical problems with integration and
%  discontinued use.
%
%  Notes:           'N' defines the order of the interpreting polynomials,
%  whtich is consisten with the literature. 'n_nodes' should be used for number
%  of nodes: n_nodes = N+1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function probinfo = SimpleUAVNumericData(probinfo)

% accounting
n = probinfo.numerics.n_nodes;
N = n-1; % order of polynomial approximation
probinfo.numerics.N = N;
probinfo.numerics.n_vars = 6*n+1;

% variable ranges
probinfo.numerics.range.x = 1:n;
probinfo.numerics.range.y = n+1:2*n;
probinfo.numerics.range.z = 2*n+1:3*n;
probinfo.numerics.range.theta = 3*n+1:4*n;
probinfo.numerics.range.utheta = 4*n+1:5*n;
probinfo.numerics.range.uz = 5*n+1:6*n;
probinfo.numerics.range.tf = 6*n+1;         % allow for free final time


% Chebyshev Pseudospectral Method values
[probinfo.numerics.t, probinfo.numerics.D, probinfo.numerics.w] = ...
    ChebyshevPseudoNumerics(N);

return;