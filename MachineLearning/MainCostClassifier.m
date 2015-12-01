
s=dbstatus;
save('myBreakpoints.mat', 's');
clear all
load('myBreakpoints.mat');
dbstop(s);
clc
close all

datafile = '.\Reachability Classifier\Data\DubinsTrainingData-IROS_InitialSubmission.txt';
testdatafile = '.\Reachability Classifier\Data\DubinsTestingData-IROS_InitialSubmission.txt';
resultsfile = 'DeepSpaceResultsPCT-2014-02-06-1930.mat';
cost_classifier(datafile, testdatafile, resultsfile);

load(resultsfile);
costAvg = mean(costTest2014);
costStdev = std(costTest2014);

% Classify reachability for avg cost
actualReachability = (costTest2014 <= costAvg);
for i=1:length(featureResults)
    predictedReachability{i} = (featureResults{i}.estimatedCosts <= costAvg);
    misclassifiedCount(i) = sum(predictedReachability{i} ~= actualReachability);
end
disp('Misclassification count for average cost threshold:')
misclassifiedCount

% Classify reachability for high cost
actualReachability = (costTest2014 <= costAvg+costStdev);
for i=1:length(featureResults)
    predictedReachability{i} = (featureResults{i}.estimatedCosts <= costAvg+costStdev);
    misclassifiedCount(i) = sum(predictedReachability{i} ~= actualReachability);
end
disp('Misclassification count for high cost threshold:')
misclassifiedCount

% Classify reachability for low cost
actualReachability = (costTest2014 <= costAvg-costStdev);
for i=1:length(featureResults)
    predictedReachability{i} = (featureResults{i}.estimatedCosts <= costAvg-costStdev);
    misclassifiedCount(i) = sum(predictedReachability{i} ~= actualReachability);
end
disp('Misclassification count for low cost threshold:')
misclassifiedCount