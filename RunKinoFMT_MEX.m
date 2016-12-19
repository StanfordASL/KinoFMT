% RunKinoFMT_MEX executes the kinodynamic Fast Marching Trees algorithm 
%   via a mex function for speed
%
%   Ross Allen, ASL, Stanford University
%   Nov 09, 2015
%
%   Inputs:
%
%   Functionality:
%
%   Notes:
%      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function mpinfo = RunKinoFMT_MEX(mpinfo)

% Unpack and preprocess variables for type matching
evalMat = uint64(mpinfo.evalMat);


% Call mex function to solve 
[optPath, optCost, parents, failure] = KinoFMTMEX(...
	evalMat,...								%(uint64) [nSample, nSample] matrix holding case numbers for 2PBVP solutions
	mpinfo.costMat,...						%(double) [nBVPs, 1] vector holding cost of each 2PBVP		
	mpinfo.trajMat,...						%(double) [nBVPs, 1] vector holding cost of each 2PBVP
    mpinfo.environment.nWorkDims,...         %(int)  number of dimensions in the workspace
    mpinfo.environment.bounds,...			%(double) [nConfDims,2] matrix containing the environmental bounds
	mpinfo.obstacles.cuboids.ulVerts',...	%(double) [nConfDims,2*nObs] matrix storing upper and lower vertices of m obstacles
    mpinfo.obstacles.spheres',...           %(double) [nConfDims+1,nSphObs] matrix storing position and radius of sphereical obstacles
	mpinfo.outNeighborTrimmedSortedIDs,...  %(cell array of uint32) contains a sorted list of outgoing neighbors
	mpinfo.inNeighborTrimmedSortedIDs,...   %(cell array of uint32) contains a sorted list of incoming neighbors
	mpinfo.sampling.nGoalSamples...			%(int) number of samples from goal region
	);

mpinfo.fmtFailure = failure;
mpinfo.optPath = optPath;
mpinfo.optCost = optCost;
mpinfo.treeParents = parents;




