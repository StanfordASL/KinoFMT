% function [blg_output, error_rms, error_max]  =...
%     DeepSpaceTrainCostEstimator( stateMat, evalMat, costMat,...
%     initStateIDs, finalStateIDs, extract_2PBVP_features) 
% DeepSpaceTrainCostEstimator Trains a optimal 2-point boundary value problem
% cost estimator based on batched linear regression
% % 
% % 
% % AUTHOR:  Ross Allen, ASL, Stanford University
% % DATE:    March 14, 2014
% % 
% % BASED ON:  Ashley Clark, ASL, Stanford University
% % DATE:      January 2014
% % 
% % INPUTS:
% % stateMat = matrix of states used for training
% % 
% % initStateIDs = list of stateMat indices that represent the initial state of the 2PBVP
% % 
% % finalStateIDs = list of stateMat indices that represent the final state of the 2PBVP
% % 
% % costMat = c(i,1) is the cost of the 2PBVP from stateMat(initStateIDs(i),:) to stateMat(finalStateIDs(i),:)
% % 
% % 
% % extract_2PBVP_features = a function pointer of form:
% %       'feature_matrix = extract_2PBVP_features( stateMat1, stateMat2 )'
% % that takes a matrix of initial states (in rows) and a
% % matrix of final states (in rows) and computes a feature vector for each
% % initial-final state pair (in rows)
% % 
% % 
% % OUTPUTS:
% % blg_output = theta values for a batched linear regression
% % 
% % error_rms = root mean square of the training error
% % 
% % error_percent = average training error as percent of true cost 
% % 
% % 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% %Extract the matrix of feature vectors from the matrix of states
% stateMat1 = stateMat(initStateIDs,:);
% stateMat2 = stateMat(finalStateIDs,:);
% training_data   = extract_2PBVP_features( stateMat1, stateMat2 );
% m               = size(training_data,1);    % true number of examples for which to train SVM
% 
% % Extract training costs
% % Label training data: Determine reachability within cost threshold
% training_cost   = Inf*ones(m,1);
% for i = 1:m
%     training_cost(i,1) = costMat(evalMat(initStateIDs(i,1),finalStateIDs(i,1)));
% end
% 
% % Determine best model
% blg_output = batchLinearRegression(training_data, training_cost);
% training_estimated_cost = [ones(length(training_data(:,1)),1),...
%     training_data]*blg_output;
% cost_error = training_estimated_cost - training_cost;
% error_max = max(abs(cost_error));
% error_rms = sqrt(sum(cost_error.^2)/length(cost_error));
% 
% % xExtent = max(training_data) - min(training_data);
% % for i=1:length(taus)
% %     
% % end
% 
% end
% 
% function [theta] = batchLinearRegression(xdat, ydat)
% 
% % xfit = [ ---xfit1--- ]
% %        [ ---xfit2--- ] etc
% 
% % Unweighted Least Squares
% X = [ones(length(xdat(:,1)),1) xdat];
% theta = (X'*X)\(X'*ydat);
% yfit = [ones(length(xfit(:,1)),1), xfit]*theta;
% 
% end