
function [svm_output, avg_n_errors, avg_error_percent] = kFold_cross_validation( k, data, labels, train_options, test_options )
    m       = size(data,1);                 % Total number of data points
    m_test  = floor(m/k);                   % Number of data points used for testing per iteration (m_train = m - m_test);
    
    n_errors        = zeros(1,k);
    error_percent   = zeros(1,k);
    svm_output      = cell(1,k);
    for j = 1:k
        % Remove the j-th piece of data from the training set for training
        test_data      	= data( ((j-1)*m_test + 1):j*m_test, : );
        test_labels    	= labels( ((j-1)*m_test + 1):j*m_test );
        training_data  	= [ data( 1:((j-1)*m_test), : ); data( (j*m_test+1):end, : ) ];
        training_labels	= [ labels( 1:((j-1)*m_test) ); labels( (j*m_test+1):end ) ];
        
        % Run SVM and evaluate the prediction performance
        svm_output{j}  	= svm_train( training_data, training_labels, train_options );
        svm_labels    	= svm_predict( svm_output{j}, test_data, test_options );
        [n_errors(j), error_percent(j)]   = count_errors( svm_labels, test_labels );
    end
    
    % Average the test errors
    avg_n_errors        = mean(n_errors);
    avg_error_percent   = mean(error_percent);
end