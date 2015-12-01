function [svm_output, n_errors, error_percent]  =...
    DeepSpaceTrainSVM( stateMat, evalMat, costMat,...
    initStateIDs, finalStateIDs, neighborCostThreshold,...
    extract_2PBVP_features, train_options, varargin ) 
% DeepSpaceTrainSVM   Trains a reachability classifier for the deep-space 
%                     spacecraft 2pt boundary value problem
%
%
%AUTHOR:  Ross Allen, ASL, Stanford University
%DATE:    March 11, 2014
%
%BASED ON:  Joseph Starek, ASL, Stanford University
%DATE:      January 2014
%
%INPUTS:
% stateMat = matrix of states used for training
%
% initStateIDs = list of stateMat indices that represent the initial state of the 2PBVP
%
% finalStateIDs = list of stateMat indices that represent the final state of the 2PBVP
%
% costMat = c(i,1) is the cost of the 2PBVP from stateMat(initStateIDs(i),:) to stateMat(finalStateIDs(i),:)
% 
% neighborCostThreshold = all 2PBVP's whose cost-to-go is less than this value are considered "reachable"
%
% extract_2PBVP_features = a function pointer of form:
%       'feature_matrix = extract_2PBVP_features( stateMat1, stateMat2 )'
% that takes a matrix of initial states (in rows) and a
% matrix of final states (in rows) and computes a feature vector for each
% initial-final state pair (in rows)
%
% optional arguments:
% 'Export', true/false = boolean indicating whether to export parameters to file
% 'ExportFilename', string = name of filename for exported training parameters
%
%OUTPUTS:
% svm_output = MATLAB object that contains the trained SVM. Used to predict/classify new data
%
% n_errors = total number of training examples that were misclassified with svm_output
%
% error_percent = percent of training examples that were misclassified
%
% Call as:
% train_options = svmset('Param1', Value1, 'Param2', Value2, ...)
% svm_output    = reachability_classifier( stateMat, costMat, neighborCostThreshold, @dubins_extract_2PBVP_features, train_options )
%

% Default values
SVM_filename            = 'svm_train_parameters.txt';
export_train_parameters = false;

% Read in user input
for k = 1:2:length(varargin)
    if strcmpi( varargin{k}, 'Export' )
        export_train_parameters = varargin{k+1};
    elseif strcmpi( varargin{k}, 'ExportFilename' )
        SVM_filename = varargin{k+1};
    end
end

% Training labels
TRUE        = 1;
FALSE       = -1;


% Extract the matrix of feature vectors from the matrix of states
stateMat1 = stateMat(initStateIDs,:);
stateMat2 = stateMat(finalStateIDs,:);
training_data   = extract_2PBVP_features( stateMat1, stateMat2 );
m               = size(training_data,1);    % true number of examples for which to train SVM
n               = size(training_data,2);

% Label training data: Determine reachability within cost threshold
training_reachability   = FALSE.*ones(m,1);
for i = 1:m
    if costMat(evalMat(initStateIDs(i,1),finalStateIDs(i,1))) <= neighborCostThreshold
        training_reachability(i,1) = TRUE;
    end
end

% Run the SVM Algorithm
svm_output  = svm_train( training_data, training_reachability, train_options );

% Determine Training Error
training_predicted_reachability = svmclassify(svm_output, training_data);
n_errors = nnz(training_predicted_reachability - training_reachability);
error_percent = 100*n_errors/m;

% Export training parameters
if export_train_parameters
    export_training_parameters( SVM_filename, svm_output, train_options );
end

end