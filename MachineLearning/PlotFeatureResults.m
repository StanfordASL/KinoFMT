
clear all
close all
clc

set(0,'DefaultAxesFontSize', 20)
textSize = 20;
legendSize = 16;
markersize = 14;
linewidth = 2;

load('DubinsResults_CostClassifier_IROS_InitialSubmission.mat');

avgFeatureCosts = zeros(size(featureResults));

for i=1:length(featureResults)
%     bestCosts = min(featureResults{i}.trainErrorRMS,[],2);
    avgFeatureCosts(i) = 100*featureResults{i}.testErrorRMS; %mean(bestCosts);
end
[bestCost, bestIndex] = min(avgFeatureCosts);

figure()
plot(length(featureResults):-1:1, avgFeatureCosts, '*','linewidth',linewidth)
% title('Average Cost Test Error vs Feature Count')
xlabel('Number of Features')
ylabel('Average RMS Test Error')
% sortedFeatureList = featureList(setRemovalOrder)

avgFeatureCosts = zeros(size(featureResults));
for i=1:length(featureResults)
%     bestCosts = min(featureResults{i}.trainErrorPCT,[],2);
    avgFeatureCosts(i) = 100*featureResults{i}.testErrorPCT; %100*mean(bestCosts);
end
[bestCost, bestIndex] = min(avgFeatureCosts);

figure()
plot(length(featureResults):-1:1, avgFeatureCosts, '*','linewidth',linewidth)
% title('Average Percent Test Error vs Feature Count')
xlabel('Number of Features')
ylabel('Avg % Test Error')

set_plot_properties('mode','publication')

% tauIndices = 1:5;
% featureIndex = 1;
% costError = 100*featureResults{featureIndex}.costErrorPCT;
% legendLabels = {'10-Fold, Set 1', '10-Fold, Set 2', '10-Fold, Set 3', '10-Fold, Set 4', '10-Fold, Set 5', '10-Fold, Set 6', '10-Fold, Set 7', '10-Fold, Set 8', '10-Fold, Set 9', '10-Fold, Set 10'};
% legendIndices = [];
% 
% figure()
% hold all
% for i=1:length(costError(:,1))
%    validResultsIndex = find([10000 costError(i,2:end)] < 100);
%    plot(taus(validResultsIndex-1), costError(i,validResultsIndex), '-*')
% end
% legend(legendLabels{legendIndices}, 'Location', 'EastOutside')
% title('Cost Test Error vs Weighting Factor, All Features')
% xlabel('Weighting Factor')
% ylabel('RMS Test Error')
% 
% for i=2:length(costError(1,:))
%     avgResult(i-1) = mean(costError(i,costError(i,:) < 100));
% end
% figure()
% plot(taus, avgResult, '-*')
% 
% 
% 
% tauIndices = 1:4;
% featureIndex = 15;
% costError = featureResults{featureIndex}{1};
% legendLabels = {'30% Hold Out', '10-Fold, Set 1', '10-Fold, Set 2', '10-Fold, Set 3', '10-Fold, Set 4', '10-Fold, Set 5', '10-Fold, Set 6', '10-Fold, Set 7', '10-Fold, Set 8', '10-Fold, Set 9', '10-Fold, Set 10'};
% legendIndices = [];
% 
% figure()
% hold all
% for i=1:length(costError(:,1))
%    if i ~= 9
%        plot(taus(tauIndices), costError(i,tauIndices+1), '-*')
%        legendIndices(end+1) = i;
%    end
% end
% legend(legendLabels{legendIndices}, 'Location', 'EastOutside')
% title('Cost Test Error vs Weighting Factor, All Features')
% xlabel('Weighting Factor')
% ylabel('RMS Test Error')
% 
% 
% 
% tauIndices = 1:4;
% featureIndex = 22;
% costError = featureResults{featureIndex}{1};
% legendLabels = {'30% Hold Out', '10-Fold, Set 1', '10-Fold, Set 2', '10-Fold, Set 3', '10-Fold, Set 4', '10-Fold, Set 5', '10-Fold, Set 6', '10-Fold, Set 7', '10-Fold, Set 8', '10-Fold, Set 9', '10-Fold, Set 10'};
% legendIndices = [];
% 
% figure()
% hold all
% for i=1:length(costError(:,1))
% %    if isfinite(sum(costError(i,:)))
%        plot(taus(tauIndices), costError(i,tauIndices+1), '-*')
%        legendIndices(end+1) = i;
% %    end
% end
% legend(legendLabels{legendIndices}, 'Location', 'EastOutside')
% title('Cost Test Error vs Weighting Factor, All Features')
% xlabel('Weighting Factor')
% ylabel('RMS Test Error')
