
function svm_labels = svm_predict( svm_output, test_data, test_options )
    if size(test_data,2) > 2
        % Cannot display data of dimension greater than 2
        test_options.showplot = false;
    end
    
    svm_labels      = svmclassify( svm_output, test_data, 'Showplot', test_options.showplot );
end
