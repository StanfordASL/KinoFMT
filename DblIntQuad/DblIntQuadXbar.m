% calculates xbar_t as defined in Kinodynamic RRT*
%
%   Author: Ross Allen
%
%   Date: Apr 10, 2015
%
%   Inputs:
%       t   = (s) evaulation time at which xbar_t is being evaluated
%       x0  = (m,m,m,m/s,m/s,m/s) initial state
%       g   = (m/s/s) gravity constant 
%               (should be positive for NED frame)
%
%   Outputs:
%
%   Notes:
%       - See Webb and van den Berg: Kinodynamic RRT*
%       - Assumes dynamics matrix, A, is a double integrator
%       - Assumes a constantc vector, c, is gravity in a NED frame
%           (i.e. z-axis positive down therefore g is positive)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function xbar_t = DblIntQuadXbar( t, x0, g )

if t < 0
    disp('t must be positive. Exiting DblIntQuadXbar');
    xbar_t = NaN;
    return;
end

if size(x0,1) ~= 6 || size(x0,2) ~= 1
    disp('x0 must be 6x1. Exiting DblIntQuadXbar');
    xbar_t = NaN;
    return;
end

if g < 0
    disp('g must be positive for NED frame. Exiting DblIntQuadXbar')
    xbar_t = NaN;
    return;
end

expAt = [   1 0 0 t 0 0;...
            0 1 0 0 t 0;...
            0 0 1 0 0 t;...
            0 0 0 1 0 0;...
            0 0 0 0 1 0;...
            0 0 0 0 0 1];

xbar_t = expAt*x0 + [0; 0; 0.5*g*t*t; 0; 0; g*t];

end
