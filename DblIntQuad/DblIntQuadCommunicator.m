%DblIntQuadCommunicator writes spline data to a file that can be accessed by
%the wifi communication application
%
%   Ross Allen, ASL, Stanford University
%
%   Started:    June 5, 2015
%
%   Inputs:     
%
%   Outputs:
%
%   Notes:
%       - this should be phased out in favor of the ViconTCPHandler
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function DblIntQuadCommunicator(mpinfo)

if (mpinfo.fmtFailure || (isfield(mpinfo, 'smoother') && ~mpinfo.smoother.valid))
    if (mpinfo.onlineOptions.xmitHoldOnFailure)
        % Transmit hold position in event of failure
        mpinfo.smoother.nSeg = 1;
        mpinfo.smoother.Tdel = 1;
        mpinfo.smoother.nCoef = 10; % this has to be hard coded because the DblIntQuadSplineSmoother hard codes the degree to 9
        mpinfo.smoother.splineCoefs = zeros(10,4);
        mpinfo.smoother.splineCoefs(1,1:3) = mpinfo.termStates.Xprior(1:3);
        try
            % this should execute if smoother was attempted
            mpinfo.smoother.splineCoefs(1,4) = mpinfo.smoother.options.yaw;
        catch
            % this should execute if planner failed
            mpinfo.smoother.splineCoefs(1,4) = mpinfo.smoother.yaw;
        end
    else
        return;
   end
end


commfile = mpinfo.comms.commfile;

% suffix for incomplete file
commfileP = strcat(commfile, '-partial');

% number of segments (and should overwrite old data)
dlmwrite(commfileP, mpinfo.smoother.nSeg);

% timing data
dlmwrite(commfileP, mpinfo.smoother.Tdel', '-append', 'delimiter', ',', 'precision', 4);

% number of coefficients
dlmwrite(commfileP, mpinfo.smoother.nCoef, '-append');

% coefficients of spline
dlmwrite(commfileP, mpinfo.smoother.splineCoefs', '-append', 'delimiter', ',', 'precision', 4);

% rename to completed file
movefile(commfileP, commfile);


end
