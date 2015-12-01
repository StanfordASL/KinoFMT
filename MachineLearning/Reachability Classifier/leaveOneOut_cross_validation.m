
function [svm_output, avg_n_errors, avg_error_percent] = leaveOneOut_cross_validation( data, labels, train_options, test_options )
    % Leaves a single training example out of the training set and trains SVM on the rest, once for each example
    m       = size(data,1);
    [svm_output, avg_n_errors, avg_error_percent] = kFold_cross_validation( m, data, labels, train_options, test_options );
end