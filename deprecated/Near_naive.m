% Near_naive.m returns the list of nearest neighbors to a given node based on a
% cost matrix computed ahead of time. This function is "naive" because it
% looks through all the other nodes. Future versions will search more
% intelligently.
%
%   Ross Allen, ASL, Stanford University
%   Nov 20, 2013
%
%   INPUTS:
%       z               Current node for which neigboor is generated
%       rn              Cost neighborhood search radius/threshold
%       costMatrix      Matrix containing the cost to connect any two nodes
%
%   OUTPUTS:
%       Nz              List of indices for nodes that are in the neighboorhod of z
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Nz] = Near_naive(costMatrix, z, rn)

costArr = costMatrix(z,:);
Nz = find(costArr <= rn);
Nz(find(Nz==z,1)) = [];

end