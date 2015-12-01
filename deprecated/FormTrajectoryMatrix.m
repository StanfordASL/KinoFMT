function TrajMat = FormTrajectoryMatrix( TrajStruct )
%FormTrajectoryMatrix condenses into a 4D matrix the information for the 
% optimal trajectory from each sampled state to every other sampled state
%
%   AUTHOR: Ross Allen
%   DATE:   Jan 20, 2014
%
%   INPUTS:
%       TrajStruct =        structure containing all the trajectory data
%
%   OUTPUTS:
%       TrajMat =           4D matrix with all trajectory data. Dimensions
%                           of matrix are (#samples x (#states+#controls) x
%                           #interp points x #samples)
%
%   NOTES:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

