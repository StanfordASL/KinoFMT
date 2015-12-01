
function [svm_output, n_errors, error_percent] = holdOut_cross_validation( holdout_percent, data, labels, train_options, test_options )
    m    	= size(data,1);                                                 % Total number of data points
    m_test	= max( floor( (min(max(holdout_percent,0),100)/100)*m ), 1);   	% Number of data points used for testing (m_train = m - m_test);
    
    % Hold out a percentage of the training data for testing
    test_data           = data( 1:m_test, : );
    test_labels         = labels( 1:m_test );
    training_data       = data( (m_test+1):end, : );
    training_labels     = labels( (m_test+1):end );
    
    % Run SVM and evaluate the prediction performance
    svm_output{1}       = svm_train( training_data, training_labels, train_options );
    svm_labels          = svm_predict( svm_output{1}, test_data, test_options );
    [n_errors, error_percent]	= count_errors( svm_labels, test_labels );
end