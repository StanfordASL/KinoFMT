% DblIntQuadCheckExistingPlan collision checks between Xprior and
% XtargetEst to determine if the existing plan is valid. If plan is valid,
% planning cycle is skipped
%
%	AUTHOR: Ross Allen
%	DATE:	Feb 2016
%
%   INPUTS:
%
%   OUTPUTS:
%
%	NOTES:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function continuePlanning = DblIntQuadCheckExistingPlan (mpinfo)

collisionFree = CollisionCheckerMEX(...
                    mpinfo.termStates.Xprior(1:3), ...
                    mpinfo.termStates.XtargetEst(1:3),...
                    mpinfo.environment.bounds,...
                    mpinfo.obstacles.cuboids.ulVerts',...
                    mpinfo.obstacles.spheres');
                
continuePlanning = ~collisionFree;

end


