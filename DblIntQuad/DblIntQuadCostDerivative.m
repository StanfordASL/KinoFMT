% calculates derivative of cost function
%
%   Author: Ross Allen
%
%   Date: Apr 13, 2015
%
%   Inputs:
%       t   = (s) evaulation time at which gramian is evaluated
%       x0  = (m,m,m,m/s,m/s,m/s) initial state
%       x1  = (m,m,m,m/s,m/s,m/s) final state
%       g = (m/s/s) gravity
%       wR  = (s^5/m^2) weighting factor between time minimization
%               and control minimization
%
%   Outputs:
%
%   Notes:
%       - See Webb and van den Berg: Kinodynamic RRT*, eqn 13
%       - Assumes dynamics matrix, A, and control matrix, B, are
%           that of a double integrator
%       - Assumes NED inertial frame (therefore g is positive)
%       - Assumes penalty on control norm-squared (R = wR*eye(3,3))
%       - Note this is of the full cost function (time + control 
%           effort) not the utilized cost (final time) which is used
%           for motion planning
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Jprime = DblIntQuadCostDerivative( t, x0, x1, wR, g )

if t < 0
    disp('t must be non-negative. Exiting DblIntQuadCostDerivative');
    Jprime = NaN;
    return;
end

if wR <= 0 
    disp('wR must be non-negative. Exiting DblIntQuadCostDerivative');
    Jprime = NaN;
    return;
end

if size(x0,1) ~= 6 || size(x0,2) ~= 1 || ...
    size(x1,1) ~= 6 || size(x1,2) ~= 1
    
    disp('x0 and x1 must be 6x1. Exiting DblIntQuadCostDerivative')
    Jprime = NaN;
    return;
end

if g < 0 
    disp('g must be pos for NED frame. Exiting DblIntQuadCostDerivative');
    Jprime = NaN;
    return;
end

A = zeros(6,6);
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;

B = zeros(6,3);
B(4,1) = 1;
B(5,2) = 1;
B(6,3) = 1;

c = zeros(6,1);
c(6,1) = g;

R_inv = (1/wR)*eye(3,3);

G = DblIntQuadGramian( t, wR);
xbar = DblIntQuadXbar( t, x0, g);
d = G\(x1-xbar);

Jprime = 1 - 2*(A*x1 +c)'*d - d'*B*R_inv*B'*d;

end

