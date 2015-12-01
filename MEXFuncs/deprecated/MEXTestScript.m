% Test the CheckCollisionMEX.cpp func for accuracy and speed

clear
clc

addpath([pwd, '/../']);
addpath([pwd, '/../ObstacleSets/']);

% Define Obstacles and bounds
[LWH, YPR, CM] = IndoorObstacleSet4(0);
ulVerts = ObstacleFormatConversion( LWH, YPR, CM );
bounds = [  -0.5    2.8;...
            0.14    4.25;...
            -2.75   -0.2];

% Generate random samples
nSamples = 100;
nSkip = randi(1e6,1);
sampleRange = [ -1.0    3.0;...
                0.1     4.5;...
                -3.0    0.0];
samples = HaltonSampling(3, nSamples, sampleRange, nSkip, 0, true);

% Run MEX collision checking
colCheck_mex = ones(nSamples, nSamples);
tic
for i = 1:nSamples-1
    for j = i+1:nSamples
        colCheck_mex(i, j) = CheckCollisionMEX(samples(i,:), samples(j,:), size(CM,2), ulVerts', bounds); 
    end
end
delt_mex = toc;

% Run mfile collision checking
colCheck_mfile = ones(nSamples, nSamples);
tic
for i = 1:nSamples-1
    for j = i+1:nSamples
        colCheck_mfile(i, j) = checkCollision(samples(i,:), samples(j,:), ulVerts, bounds); 
    end
end
delt_mfile = toc;

% Ensure identical results
%calcErr = any( colCheck_mex-colCheck_mfile ~= 0)
calcErr = any(any(colCheck_mex-colCheck_mfile ~=0))

% Compare timing
mex_2_mfile = delt_mex/delt_mfile
