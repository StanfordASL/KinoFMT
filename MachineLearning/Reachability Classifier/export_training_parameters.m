
function export_training_parameters( SVM_filename, svm_output, train_options )
    Nhypotheses = length(svm_output);
    fid = fopen(SVM_filename);

        'mlp_params', mlp_params, ...
        'polyorder', polyorder, ...
        'rbf_sigma'
%     .Alpha
%     .Bias
%     .KernelFunction
%     .KernelFunctionArgs
    for j = 1:Nhypotheses
        fprintf( fid, 'SVM Output\n' ); %svm_output{j}.SupportVectors
        if train_options.autoscale
            % Print scaling data
        end
    end
end