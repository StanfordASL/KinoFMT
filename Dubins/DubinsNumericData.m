%   DubinsNumericData.m: Compute Chebyshev differentiation, non-dimensional time vector, 
%                   variable and constraint counts & ranges
%
%   Author: Ross Allen, ASL, Stanford University
%   Based on Work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        10/17/2013
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

function dubprob = DubinsNumericData(dubprob)

% accounting
N = dubprob.numerics.n_nodes-1; % order of polynomial approximation
dubprob.numerics.N = N;
dubprob.numerics.n_vars = 4*(N+1)+1;

% variable ranges
dubprob.numerics.range.x = 1:N+1;
dubprob.numerics.range.y = N+2:2*(N+1);
dubprob.numerics.range.theta = 2*(N+1)+1:3*(N+1);
dubprob.numerics.range.u = 3*(N+1)+1:4*(N+1);
dubprob.numerics.range.tf = 4*(N+1)+1;


% Chebyshev Pseudospectral Method values
[dubprob.numerics.t, dubprob.numerics.D, dubprob.numerics.w] = ...
    ChebyshevPseudoNumerics(N);

return;