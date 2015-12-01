%InitialErrorcheck validates user inputs
%
%   Ross Allen, ASL, Stanford University
%
%   Started:        Apr 7, 2015
%
%   Inputs:
%
%   Outputs:
%
%   Notes:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function err = InitialErrorCheck(mpinfo)

err.flag = 0;
err.string = 'valid input';

% Check that sampling range is properly formatted
if size(mpinfo.sampling.stateSampleRange,2) ~= 2
    err.flag = 1;
    err.string = 'stateSampleRange inproperly sized';
    return;
end
if any(mpinfo.sampling.stateSampleRange(:,1) > ...
        mpinfo.sampling.stateSampleRange(:,2))
   err.flag = 1;
   err.string = 'stateSampleRange min greater than max'; 
   return;
end

% Check that machine learning training examples is less than initial 
% number of 2PBVPs solved
if mpinfo.sampling.nTot2PBVPs < mpinfo.learning.nMLTrainingSamples
   err.flag = 1;
   err.string = 'nTot2PBVPs mus be greater than nMLTrainingSamples';
   return;
elseif mpinfo.learning.nMLTrainingSamples\mpinfo.sampling.nTot2PBVPs < 0.5
    disp('WARNING: less than half of 2PBVPs used for ML training')
end

% Check that number of 2PBVPs to be solved is less than or equal
% to number of sample pairs
if mpinfo.sampling.nTot2PBVPs > mpinfo.sampling.nSamples^2
    err.flag = 1;
    err.string = 'nTot2PBVPs must be less than nSamples^2';
    return;
end
