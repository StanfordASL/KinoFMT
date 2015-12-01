clear
clc
close all

load('~/Segovia/planningdata/GenKinoFMT/DubinsPrecomputeV2_500x10_Jan20-2014.mat');
controlMat = Text2Matrix('~/Segovia/planningdata/GenKinoFMT/DubinsPrecomputeV2_500x10_Jan20-2014_Control.txt');

[nSamples, nStateDims, nTrajNodes, ~] = size(trajMat);
[~, nControlDims, ~, ~] = size(controlMat);

trajMatFix = NaN*ones(nSamples, nStateDims, nTrajNodes, nSamples);
controlMatFix = NaN*ones(nSamples, nControlDims, nTrajNodes, nSamples);
for i = 1:nSamples
    for j = 1:nSamples
        trajMatFix(j,:,:,i) = trajMat(i,:,:,j);
        controlMatFix(j,:,:,i) = controlMat(i,:,:,j);
    end
end

disp('done reformatting')
err = Matrix2Text(trajMatFix, '~/Segovia/planningdata/GenKinoFMT/DubinsPrecomputeV2_500x10_Jan20-2014_Trajectory__2.txt');
err = Matrix2Text(controlMatFix, '~/Segovia/planningdata/GenKinoFMT/DubinsPrecomputeV2_500x10_Jan20-2014_Control__2.txt');
