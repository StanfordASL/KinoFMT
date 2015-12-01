% generates exponential of optimal trajectory given an optimal 
%   final time
%
%   Author: Ross Allen
%
%   Date: Apr 14, 2015
%
%   Inputs:
%       t   = (s) time of evaluation
%       tau  = (s) optimal final time
%       x0  = (m,m,m,m/s,m/s,m/s) initial state
%       x1  = (m,m,m,m/s,m/s,m/s) final state
%       wR  = (s^5/m^2) weighting factor between time minimization
%               and control minimization
%       g   = (m/s/s) gravity (should be positive)
%       d_tau = (optional) term from eqn 14 of Webb and van den Berg.
%               can be passed to accelerate computation
%
%   Outputs:
%
%   Notes:
%       - See Webb and van den Berg: Kinodynamic RRT*
%       - See Schmerling, Janson, Pavone: Optimal Sampling Based 
%           Motion Planning under Differential Constraints: 
%           the Drift Case with Linear Affine Dynamics
%       - Assumes dynamics matrix, A, and control matrix, B, are
%           that of a double integrator
%       - Assumes constant vector, c, is in NED frame
%       - Assumes penalty on control norm-squared (R = wR*eye(3,3))
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function x_t = DblIntQuadOptState( t, tau, x0, x1, wR, g, d_tau )

% error checking
if nargin < 6 || nargin > 7
    disp('inappropriate number of arguments')
    x_t = NaN;
end

if tau < 0 || t < 0
    disp('t and tau must be non-negative. Exiting DblIntQuadOptState');
    x_t = NaN;
    return;
end

if tau < t || isinf(tau)
    disp('t must be less than or equal to tau and');
    disp('tau must be finite. Exiting DblIntQuadOptState');
    x_t = NaN;
    return;
end

if size(x0,1) ~= 6 || size(x0,2) ~= 1 || ...
    size(x1,1) ~= 6 || size(x1,2) ~= 1

    disp('x0 and x1 must be 6x1. Exiting DblIntQuadOptState');
    x_t = NaN;
    return;
end

if wR <= 0 
    disp('wR must be non-negative. Exiting DblIntQuadOptState');
    x_t = NaN;
    return;
end

% tau_star terms
if nargin == 7
else
    xbar_tau = DblIntQuadXbar(tau, x0, g);
    G_tau = DblIntQuadGramian(tau, wR);
    d_tau = G_tau\(x1-xbar_tau);
end

% t terms
xbar_t = DblIntQuadXbar(t, x0, g);
G_t = DblIntQuadGramian(t, wR);

% exponential term
expAtran = eye(6,6);
expAtran(4,1) = tau-t;
expAtran(5,2) = tau-t;
expAtran(6,3) = tau-t;

% evaluate equation 7 of Schmerling et. al.
x_t = xbar_t + G_t*expAtran*d_tau;
 
end
