% StateSampler samples a specified range of a state space using
% halton sequencing
%
%   Ross Allen, ASL, Stanford University
%   Feb 2, 2015
%
%   Notes:
%       - Samples state space and forms stateMat
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ mpinfo ] = DSSCStateSampler( mpinfo )

sysdefs = mpinfo.systemDefs;
sampling = mpinfo.sampling;

halton_skip = randi(1e6,1); % randomly generate number of initial halton sequence to omit

mpinfo.stateMat = HaltonSampling(...
    sysdefs.nStateDims, sampling.nSamples, sampling.stateSampleRange,...
    halton_skip, 0, true);

end

