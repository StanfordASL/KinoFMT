% calculates weighted controllability grammian as defined in 
%   Kinodynamic RRT* paper
%
%   Author: Ross Allen
%
%   Date: Apr 13, 2015
%
%   Inputs:
%       t   = (s) evaulation time at which gramian is evaluated
%       wR  = (s^5/m^2) weighting factor between time minimization
%               and control minimization
%
%   Outputs:
%
%   Notes:
%       - See Webb and van den Berg: Kinodynamic RRT*
%       - Assumes dynamics matrix, A, and control matrix, B, are
%           that of a double integrator
%       - Assumes penalty on control norm-squared (R = wR*eye(3,3))
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function G_t = DblIntQuadGramian( t, wR )

if t < 0
    disp('t must be non-negative. Exiting DblIntQuadGramian');
    G_t = NaN;
    return;
end

if wR <= 0 
    disp('wR must be non-negative. Exiting DblIntQuadGramian');
    G_t = NaN;
    return;
end

c1 = (1/3)*t^3;
c2 = 0.5*t^2;

G_t = (1/wR)*[...
        c1  0   0   c2  0   0;...
        0   c1  0   0   c2  0;...
        0   0   c1  0   0   c2;...
        c2  0   0   t   0   0;...
        0   c2  0   0   t   0;...
        0   0   c2  0   0   t];        

end
