% DSSCTrainCostEstimator trains weighted and unweighted batched linear
% regression estimator that is used to rapidly estimate the cost of a new
% optimal boundary value problem
%
%   Ross Allen, ASL, Stanford University
%   Sep 24, 2014
%
%   Functionality:
%       
%
%   Notes:
%       - Significantly different than TrainCostEstimator in previous
%       systems (e.g. DeepSpace). Old versions are commented out because
%       they are not used in old systems
%       - Cost estimator training is drawn from the same set of 2PBVPs that
%       neighbor classifier is trained from. Improvements could be made by
%       sampling from the remainder 2PBVPs that have been solved but it
%       requires a significant reworking of variable names (e.g.
%       nTot2PBVPS, validTotCaseNums, etc)
%       - COULD BE GENERALIZED TO SYSTEM-AGNOSTIC FUNCTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ mpinfo ] = DSSCTrainCostEstimator( mpinfo )

% Unpack variables to be accessed and modified
learning = mpinfo.learning;


% Unpack variables to accessed (not modified)
sampling = mpinfo.sampling;
stateMat = mpinfo.stateMat;
costMat = mpinfo.costMat;
evalMat = mpinfo.evalMat;
initStateIDs = sampling.initStateIDs; % state IDs for set of 2PBVPs sampled for ML training
finalStateIDs = sampling.finalStateIDs; % state IDs for set of 2PBVPs sampled for ML training
blr_taus = learning.cost.blr_taus;   % set of tau values used for weighted batched linear regression
costFeatureSet = learning.cost.featureSet;


% Loop through number of cost training cases
for k = 1:learning.cost.nTrainingsRuns
    trainingSampleTotCaseNums = randsample(sampling.validTotCaseNums,...
        learning.nMLTrainingSamples); % sample without replacement
    trainingInitStateIDs = initStateIDs(trainingSampleTotCaseNums,1);
    trainingFinalStateIDs = finalStateIDs(trainingSampleTotCaseNums,1);
    trainingExamplesMat1 = stateMat(trainingInitStateIDs,:); % Stores the set of all initial points for 2PBVP's
    trainingExamplesMat2 = stateMat(trainingFinalStateIDs,:); % Stores the set of all final points for 2PBVP's
    trainingXData{k}   = costFeatureSet( trainingExamplesMat1, trainingExamplesMat2);
    trainingYData{k}   = Inf*ones(size(trainingXData{k},1),1);
    for i = 1:size(trainingXData{k},1)
        trainingYData{k}(i,1) = costMat(evalMat(trainingInitStateIDs(i,1),trainingFinalStateIDs(i,1)));
    end
    xExtent{k} = max(trainingXData{k}) - min(trainingXData{k});

    
    testingSampleTotCaseNums = [1:sampling.nTot2PBVPs]';
    testingSampleTotCaseNums(trainingSampleTotCaseNums) = []; % remove 2PBVPs used in training
    testingSampleTotCaseNums(find(sampling.validTotCaseNumsBool(...
        testingSampleTotCaseNums,1)==0)) = []; % remove invalid 2PBVPs
    testingSampleTotCaseNums = randsample(testingSampleTotCaseNums,...
        learning.nMLTestingSamples ); % sample without replacement
    testingInitStateIDs = initStateIDs(testingSampleTotCaseNums,1);
    testingFinalStateIDs = finalStateIDs(testingSampleTotCaseNums,1);
    testingExamplesMat1 = stateMat(testingInitStateIDs,:); % Stores the set of all initial points for 2PBVP's
    testingExamplesMat2 = stateMat(testingFinalStateIDs,:); % Stores the set of all final points for 2PBVP's
    testingXData   = costFeatureSet( testingExamplesMat1, testingExamplesMat2);
    testingYData   = Inf*ones(size(testingXData,1),1);
    for i = 1:size(testingXData,1)
        testingYData(i,1) = costMat(evalMat(testingInitStateIDs(i,1),testingFinalStateIDs(i,1)));
    end
    
    % Unweighted
    theta{k} = batchUnweightedLinearRegression(...
        trainingXData{k}, trainingYData{k});
    curTestingEstOutput = [ones(length(testingXData(:,1)),1),...
        testingXData]*theta{k};
    curCostError = curTestingEstOutput - testingYData;
    testingErrorMax(1,k) = max(abs(curCostError));
    testingErrorRMS(1,k) = sqrt(sum(curCostError.^2)/length(curCostError));
    
    % Weighted
    for i = 1:length(blr_taus)
        [~, curTestingEstOutput] = batchWeightedLinearRegression(...
            trainingXData{k}, trainingYData{k}, blr_taus(i),...
            testingXData, xExtent{k});
        curCostError = curTestingEstOutput - testingYData;
        testingErrorMax(i+1,k) = max(abs(curCostError));
        testingErrorRMS(i+1,k) = sqrt(sum(curCostError.^2)/length(curCostError));
    end
    
end

% Determine best estimator
avgCostTestingErrorRms = sum(testingErrorRMS,2)/size(testingErrorRMS,2);
maxCostTestingErrorMax = max(testingErrorMax,[],2);
[~, indexBestTau] = min(avgCostTestingErrorRms);

% Choose weighted or unweighted and store data from best run
if indexBestTau == 1
    learning.cost.useUnweightedBLR = true;
    [~, bestRun] = min(testingErrorRMS(1,:));
    learning.cost.unweightedBLR.theta = theta{bestRun};
else
    learning.cost.useUnweightedBLR = false;
    [~, bestRun] = min(testingErrorRMS(indexBestTau,:));
    learning.cost.weightedBLR.tau = blr_taus(indexBestTau - 1);
    learning.cost.weightedBLR.xData = trainingXData{bestRun};
    learning.cost.weightedBLR.yData = trainingYData{bestRun};
    learning.cost.weightedBLR.xExtent = xExtent{bestRun};
end

% Consolidate results
learning.cost.testingErrorRMS = testingErrorRMS;
learning.cost.testingErrorMax = testingErrorMax;
mpinfo.learning = learning;

end
