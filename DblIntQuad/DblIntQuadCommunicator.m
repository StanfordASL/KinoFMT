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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function DblIntQuadCommunicator(mpinfo)

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
