%   SimpleUAVTrainSVM.m: Compute final time for objective function 
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        5/2/2014
%
%   Inputs:          
%
%   Outputs:
%
%   Functionality:
%       - Calculate neighborhood cost threshold
%       - Samples solved 2PBVPs for training (other solved 2PBVPs used for
%       testing)
%       - Trains SVM
%       - Tests SVM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [mpinfo] = SimpleUAVTrainNeighborClassifier(mpinfo)

% Unpack variables to be accessed and modified
learning = mpinfo.learning;
nMLTrainingSamples = learning.nMLTrainingSamples;
nMLTestingSamples = learning.nMLTestingSamples;

% Unpack variables to accessed (not modified)
n2PBVPs = mpinfo.sampling.n2PBVPs;
nTot2PBVPs = mpinfo.sampling.nTot2PBVPs;
costMat = mpinfo.costMat;
stateMat = mpinfo.stateMat;
evalMat = mpinfo.evalMat;
train_options = learning.neighbor.train_options;
neighborFeatureSet = learning.neighbor.featureSet;
validTotCaseNumsBool = mpinfo.sampling.validTotCaseNumsBool;
validTotCaseNums = mpinfo.sampling.validTotCaseNums;
initStateIDs = mpinfo.sampling.initStateIDs;
finalStateIDs = mpinfo.sampling.finalStateIDs;


% Check Options
if isnan(nMLTrainingSamples)
    nMLTrainingSamples = n2PBVPs;
    nMLTestingSamples = 0;
elseif nMLTrainingSamples <= 0 || nMLTrainingSamples > n2PBVPs
    disp('Error: Invalid number of training and testing examples.')
    disp('Prematurely exiting SimpleUAVTrainNeigborClassifier...')
    return
elseif isnan(nMLTestingSamples)
    nMLTestingSamples = n2PBVPs - nMLTrainingSamples;
elseif nMLTestingSamples <= 0 || nMLTestingSamples > n2PBVPs || ...
        nMLTrainingSamples+nMLTestingSamples > n2PBVPs
    disp('Error: Invalid number of training and testing examples.')
    disp('Prematurely exiting SimpleUAVTrainNeigborClassifier...')
    return
end

% Calculate neighborhood cost threshold (NOTE: based on quantile of cost)
if isfield(learning, 'neighborCostThreshold') &&...
        ~isnan(learning.neighborCostThreshold) &&...
        isfield(learning, 'neighborCostQuantile') &&...
        ~isnan(learning.neighborCostQuantile)
    disp('Invalid neighbor cost threshold. Choose a specific threshold or')
    disp('quantile of costs, but not both.')
    disp('Prematurely exiting SimpleUAVTrainNeigborClassifier...')
    return
elseif ~isfield(learning,'neighborCostThreshold') ||...
        isnan(learning.neighborCostThreshold)
    neighborCostThreshold = quantile(costMat, learning.neighborCostQuantile);
else
    neighborCostThreshold = learning.neighborCostThreshold;
end
disp(['Neighborhood Cost Threshold = ', num2str(neighborCostThreshold)])

% Sample set of solved 2PBVPs for training subset
trainingSampleTotCaseNums = randsample(validTotCaseNums, nMLTrainingSamples); % sample without replacement(
trainingInitStateIDs = initStateIDs(trainingSampleTotCaseNums,1);
trainingFinalStateIDs = finalStateIDs(trainingSampleTotCaseNums,1);

% Train SVM
[svm_output, n_training_nonclassifications, n_training_errors, percent_training_errors, ...
    n_training_true_positives, n_training_true_negatives, ...
    n_training_false_positives, n_training_false_negatives] = ...
    SimpleUAVTrainSVM(stateMat, evalMat, costMat,...
    trainingInitStateIDs, trainingFinalStateIDs, ...
    neighborCostThreshold, neighborFeatureSet, train_options);

% Output Results
disp('Training Results:');
disp(['Number of training non-classifications = ', num2str(n_training_nonclassifications)]);
disp(['Number of training errors = ', num2str(n_training_errors)]);
disp(['Training error percentage = ', num2str(percent_training_errors)]);
disp(['Number of training true positives = ', num2str(n_training_true_positives)]);
disp(['Number of training true negatives = ', num2str(n_training_true_negatives)]);
disp(['Number of training false positives = ', num2str(n_training_false_positives)]);
disp(['Number of training false negatives = ', num2str(n_training_false_negatives)]);

% Sample set of solved 2PBVPs that weren't used for training
testingSampleTotCaseNums = [1:nTot2PBVPs]';
testingSampleTotCaseNums(trainingSampleTotCaseNums) = []; % remove 2PBVPs used in training
testingSampleTotCaseNums(find(validTotCaseNumsBool(...
    testingSampleTotCaseNums,1)==0)) = []; % remove invalid 2PBVPs
testingSampleTotCaseNums = randsample(testingSampleTotCaseNums,...
    nMLTestingSamples); % sample without replacement
testingExamplesMat1 = stateMat(initStateIDs(testingSampleTotCaseNums,1),:); % Stores the set of all initial points for 2PBVP's
testingExamplesMat2 = stateMat(finalStateIDs(testingSampleTotCaseNums,1),:); % Stores the set of all final points for 2PBVP's

% Test SVM
testing_reachability = zeros( nMLTestingSamples, 1 ); % Stores the reachability indicators (1 = reachable, -1 = unreachable)
for k = 1:nMLTestingSamples
    curTotCaseNum = testingSampleTotCaseNums(k,1);
    if costMat(evalMat(initStateIDs(curTotCaseNum,1),...
            finalStateIDs(curTotCaseNum,1))) <= neighborCostThreshold
        testing_reachability(k) = 1;    % reachable
    else 
        testing_reachability(k) = -1;   % unreachable
    end
end
testing_data   = neighborFeatureSet( testingExamplesMat1, testingExamplesMat2);
testing_predicted_reachability = svmclassify(svm_output, testing_data); % use svmclassify instead of is_reachable to output -1 or 1 to be consisten with training
n_testing_nonclassifications = sum(isnan(testing_predicted_reachability));
n_testing_errors = nnz(testing_predicted_reachability - testing_reachability) - ...
    n_testing_nonclassifications;
percent_testing_errors = 100*n_testing_errors/k;
n_testing_true_positives = length(find(...
    testing_predicted_reachability + testing_reachability == 2));
n_testing_true_negatives = length(find(...
    testing_predicted_reachability + testing_reachability == -2));
n_testing_false_positives = length(find(...
    testing_predicted_reachability - testing_reachability == 2));
n_testing_false_negatives = length(find(...
    testing_predicted_reachability - testing_reachability == -2));

% Display results of testing
disp('Testing Results:');
disp(['Number of testing non-classifications = ', num2str(n_testing_nonclassifications)]);
disp(['Number of testing errors = ', num2str(n_testing_errors)]);
disp(['Testing error percentage = ', num2str(percent_testing_errors)]);
disp(['Number of testing true positives = ', num2str(n_testing_true_positives)]);
disp(['Number of testing true negatives = ', num2str(n_testing_true_negatives)]);
disp(['Number of testing false positives = ', num2str(n_testing_false_positives)]);
disp(['Number of testing false negatives = ', num2str(n_testing_false_negatives)]);


% Consolidate Results
learning.nMLTrainingSamples = nMLTrainingSamples;
learning.nMLTestingSamples = nMLTestingSamples;
learning.neighborCostThreshold = neighborCostThreshold;
learning.neighbor.trainingSampleTotCaseNums = trainingSampleTotCaseNums;
learning.neighbor.trainingInitStateIDs = trainingInitStateIDs;
learning.neighbor.trainingFinalStateIDs = trainingFinalStateIDs;
learning.neighbor.svm_output = svm_output;
learning.neighbor.training.nNonclassifications = n_training_nonclassifications;
learning.neighbor.training.nErrors = n_training_errors;
learning.neighbor.training.percentErrors = percent_training_errors;
learning.neighbor.training.nTruePositives = n_training_true_positives;
learning.neighbor.training.nTrueNegatives = n_training_true_negatives;
learning.neighbor.training.nFalsePositives = n_training_false_positives;
learning.neighbor.training.nFalseNegatives = n_training_false_negatives;
learning.neighbor.testingSampleTotCaseNums = testingSampleTotCaseNums;
learning.neighbor.testing.nErrors = n_testing_errors;
learning.neighbor.testing.nNonclassification = n_testing_nonclassifications;
learning.neighbor.testing.percentErrors = percent_testing_errors;
learning.neighbor.testing.nTruePositives = n_testing_true_positives;
learning.neighbor.testing.nTrueNegatives = n_testing_true_negatives;
learning.neighbor.testing.nFalsePositives = n_testing_false_positives;
learning.neighbor.testing.nFalseNegatives = n_testing_false_negatives;
mpinfo.learning = learning;

end
