% FMTPath.m calculates the shortest path from a to be based on a predeveloped
% tree
%
%   Ross Allen, ASL, Stanford University
%   Nov 22, 2013
%
%   INPUTS:
%
%   OUTPUTS:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [P] = FMTPath(xinit, Vgoal, E, V_cost2come)

[~, xend] = min(V_cost2come(Vgoal));
xend = Vgoal(xend);
x2 = xend;
x1 = find(E(:,2)==x2);
x1 = E(x1,1);
P = [x1 x2];
while x1 ~= xinit
    x2 = x1;
    x1 = find(E(:,2)==x2);
    x1 = E(x1,1);
    P = [x1 x2; P];
end

end