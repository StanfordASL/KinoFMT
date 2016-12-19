% ObstacleHandlerActive retrieves active obstacles 
%
%   Ross Allen, ASL, Stanford University
%   Feb 2016
%
%   Notes:
%       - currently all active obstacles are represented as spheres
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function mpinfo = ObstacleHandlerActive(mpinfo)
    
% Extract active obstacle data (assumed formatted after start state and
% goal region
% expected form: [start state (nStateDims), goal region (2*nStateDims), active obsacles (4*nActiveObs)]
if (mpinfo.comms.recvActiveObs)
   mpinfo.obstacles.spheres = reshape(...
       mpinfo.comms.recvBuffer(3*mpinfo.systemDefs.nStateDims+1:end), 4, [])';
end

end