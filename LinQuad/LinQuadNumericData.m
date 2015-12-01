%   LinQuadNumericData.m: Compute Chebyshev differentiation, non-dimensional time vector, 
%                   variable and constraint counts & ranges
%
%   Author: Ross Allen, ASL, Stanford University
%   Based on Work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        Oct 17, 2015
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
%
%   Notes:          I think is almost complete irrelevant for the linearized
%   quadroto model since chebyshev+sqp is not used to optimize
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function probinfo = LinQuadNumericData(probinfo)

% accounting
n = probinfo.numerics.n_nodes;
N = n-1; % order of polynomial approximation
probinfo.numerics.N = N;
probinfo.numerics.n_vars = 10*n+1;

% variable ranges
probinfo.numerics.range.x = 1:n;
probinfo.numerics.range.y = n+1:2*n;
probinfo.numerics.range.z = 2*n+1:3*n;
probinfo.numerics.range.vx = 3*n+1:4*n;
probinfo.numerics.range.vy = 4*n+1:5*n;
probinfo.numerics.range.vz = 5*n+1:6*n;
probinfo.numerics.range.phi     = 6*n+1:7*n;
probinfo.numerics.range.theta   = 7*n+1:8*n;
probinfo.numerics.range.dphi    = 8*n+1:9*n;
probinfo.numerics.range.dtheta  = 9*n+1:10*n;
probinfo.numerics.range.uf = 10*n+1:11*n;
probinfo.numerics.range.ux = 11*n+1:12*n;
probinfo.numerics.range.uy = 12*n+1:13*n;
probinfo.numerics.range.tf = 13*n+1;         % allow for free final time


% Chebyshev Pseudospectral Method values
[probinfo.numerics.t, probinfo.numerics.D, probinfo.numerics.w] = ...
    ChebyshevPseudoNumerics(N);

return;
