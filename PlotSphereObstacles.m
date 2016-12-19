%PlotSphereObstacles generates a plot of obstacles 
%
%   Ross Allen, ASL, Stanford University
%
%   Started:    Feb 2, 2016
%
%   Inputs:
%
%   Outputs:
%
%   Notes:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function err = PlotSphereObstacles(sphObs)

err = 1;
facealpha = 1.0;
nFace = 10;


for i = 1:size(sphObs,1)
   [x, y, z] = sphere(nFace);
   surf(    x*sphObs(i,4)+sphObs(i,1),...
            y*sphObs(i,4)+sphObs(i,2),...
            z*sphObs(i,4)+sphObs(i,3));
    colormap hsv
end

end