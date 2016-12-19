%   EstimatePlannerStartState uses current state information to estimate
%   the state of the system by the end of the planner computation time
%
%   Ross Allen, ASL, Stanford University
%   Feb 5, 2016
%
%   Notes:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [mpinfo, continuePlanning] = EstimatePlannerStartState(mpinfo)
    
continuePlanning = true;

% initialize prior state
mpinfo.termStates.Xprior = mpinfo.termStates.Xstart;

% Extract current state from vicon(assumed always at head of buffer)
if (mpinfo.comms.recvXprior)
    
    mpinfo.termStates.Xprior = ...
        mpinfo.comms.recvBuffer(1:mpinfo.systemDefs.nStateDims);

end

% Check existing plan for validity (collision check Xprior - XtargetEst
if (mpinfo.onlineOptions.skipCurrentValid && isfield(mpinfo.termStates, 'XtargetEst'))
    continuePlanning = mpinfo.termStates.checkExistingPlan(mpinfo);
    if ~continuePlanning
        return;
    end
end

% Estimate where system will be at completion of planning, therefore where 
% plan should be initiated
mpinfo = mpinfo.termStates.startStateEstimator(mpinfo);


end