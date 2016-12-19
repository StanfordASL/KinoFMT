% DblIntQuadStartStateEstimator estimates the state from which planning should 
%   begin based on a prior state
%   
%
%   Author: Ross Allen
%   Date:   Feb 2016
%
%   Inputs:
%
%   Outputs:
%
%   Notes:
%       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function mpinfo = DblIntQuadStartStateEstimator(mpinfo)

    dt = mpinfo.termStates.dtPriorStart;    % time step to propegrate prior state to start state
    Xprior = mpinfo.termStates.Xprior;
    if (mpinfo.termStates.dtPriorStart > 0)
        
        % constant velocity state propegation of measured state at MPC
        % iteration
        XstartProp = zeros(1, 6);
        XstartProp(1) = Xprior(1) + Xprior(4)*dt;
        XstartProp(2) = Xprior(2) + Xprior(5)*dt;
        XstartProp(3) = Xprior(3) + Xprior(6)*dt;
        XstartProp(4) = Xprior(4);
        XstartProp(5) = Xprior(5);
        XstartProp(6) = Xprior(6);
        
        % nominal state from solution during previous MPC iteration
        % Check if a nominal start state exists from prior MPC iteration
%         if isfield(mpinfo.termStates, 'XstartNom') && ~any(isnan(mpinfo.termStates.XstartNom))
%            XstartNom = mpinfo.termStates.XstartNom;
%         else
%             % if no prior MPC iteration, then nominal state should be
%             % measure prior state
%             XstartNom = mpinfo.termStates.Xprior;
%         end
%         
%         % average
%         nomprop = mpinfo.termStates.weightNomProp;
%         if nomprop < 0
%             nomprop = 0;
%         elseif nomprop > 1
%             nomprop = 1;
%         end
%         mpinfo.termStates.Xstart = nomprop*XstartNom + (1-nomprop)*XstartProp;
        
        mpinfo.termStates.Xstart = XstartProp;
        mpinfo.termStates.Xstart(4) = 0;
        mpinfo.termStates.Xstart(5) = 0;
        mpinfo.termStates.Xstart(6) = 0;
        
    else 
        mpinfo.termStates.Xstart = mpinfo.termStates.Xprior;
    end
end