
function [n_errors, error_percent] = count_errors( predicted_labels, true_labels )
    n_errors        = sum( predicted_labels ~= true_labels );
    error_percent   = 100 * (n_errors / length(true_labels));
end
