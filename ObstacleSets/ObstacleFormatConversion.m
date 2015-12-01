% ObstacleFormatConversion converts obstacles from the length, orientation, 
%   center format to the diagonal corner format
%
% AUTHOR:   Ross Allen
% DATE:     Dec 9, 2014
%
% INPUTS:
%   LWH     -   col i gives [ length width height ] of the ith obstacle
%   YPR     -   col i gives [yaw pitch roll] of ith obstacle
%   CM      -   col i gives [x y z] position of center of ith obstacle
%
% OUTPUTS:
%   ulVerts -   Matrix containing the upper and lower corner vertices
%                   where row i (odd) is bottom corner and i+1 (even) is 
%                   top corner such that ulVertMat(i,:) < ulVertMat(i+1,:)
%
% NOTES:
%   -The input obstacles can only be 3 dimensional even though the output
%       format can handle arbitrary dimensionality
%   -The output format can only handle axis-aligned cuboids. If input has
%       nonzero yaw, pitch, roll. Conversion cannot be made
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function ulVerts = ObstacleFormatConversion( LWH, YPR, CM )

% Number of obstacles
m = size(LWH,2);

% Check that input matrices are same size
if ~all(size(LWH)==size(YPR)) || ~all(size(YPR)==size(CM))
    disp('Error in ObstacleFormatConversion: inputs sizes must match')
    ulVerts = NaN;
    return;
end

% Check that obstacles are axis-aligned
if any(YPR~=0)
    disp('Error in ObstacleFormatConversion: obstacles must be axis-aligned')
    ulVerts = NaN;
    return;
end

% Calculate lower and upper corners
ulVerts = zeros(2*m, 3);
for i = 1:m
    % Lower corner
    ulVerts(2*i-1, :) = CM(:,i)' - LWH(:,i)'/2;

    % Upper corner
    ulVerts(2*i, :) = CM(:,i)' + LWH(:,i)'/2;
end
