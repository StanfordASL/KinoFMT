
function [svm_output, n_errors, error_percent]  =...
    reachability_classifier( stateMat, costMat, neighborCostThreshold,...
    extract_2PBVP_features, train_options, varargin ) 
% REACHABILITY_CLASSIFIER   Trains a reachability classifier for 2PBVP's 
%
%AUTHOR:    Joseph Starek, ASL, Stanford University
%DATE:      January 2014
%
%MODIFIED:  Ross Allen, ASL, Stanford University
%DATE:      February 2014
%
%INPUTS:
% stateMat = [ x1^T; x2^T; ...; xN^T ]
%
% costMat = [ c_{ij} ], where c_{ij} is the cost to go from node i (state xi) to node j (state xj)
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

% Filter out infeasible 2PBVP's from consideration
k           = 0;
Nsamples    = size( stateMat, 1 );                          % Number of samples for which each pair will have an associated 2PBVP, some of which may not be valid (infinite cost)
Nstates     = size( stateMat, 2 );                          % Number of states in each state vector

stateMat1  	= zeros( Nsamples^2 - Nsamples, Nstates );      % Stores the set of all initial points for (feasible) 2PBVP's
stateMat2  	= zeros( Nsamples^2 - Nsamples, Nstates );      % Stores the set of all final points for (feasible) 2PBVP's
cost        = zeros( Nsamples^2 - Nsamples, 1 );               % Stores the costs of each (feasible) 2PBVP, used for determining reachability   
for k2 = 1:Nsamples
    for k1 = 1:Nsamples
        if ( k1 ~= k2 ) && ( costMat( k1, k2 ) < Inf )
            k = k + 1;
            stateMat1(k,:)  = stateMat( k1, : );
            stateMat2(k,:)  = stateMat( k2, : );
            cost(k)         = costMat( k1, k2 );
        end
    end
end

% Truncate excess storage in stateMat1, stateMat2, and cost
if k == 0
    error('No feasible cases found for classifying reachability.');
else
    stateMat1( (k+1):end, : )  = [];
    stateMat2( (k+1):end, : )  = [];
    cost( (k+1):end )       = [];
end


% Extract the matrix of feature vectors from the matrix of states
training_data   = extract_2PBVP_features( stateMat1, stateMat2 );
m               = size(training_data,1);    % true number of examples for which to train SVM
n               = size(training_data,2);

% Label training data: Determine reachability within cost threshold
training_reachability   = FALSE.*ones(m,1);
training_reachability( cost <= neighborCostThreshold ) = TRUE;

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
