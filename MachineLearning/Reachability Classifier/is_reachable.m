
function bool_vector = is_reachable( svm_output, query_points )
    Npoints     = size(query_points,1);
    
    if iscell(svm_output)
        Nhypotheses = length(svm_output);

        y = zeros( Npoints, Nhypotheses );
        for j = 1:Nhypotheses
            y(:,j) = svmclassify( svm_output{j}, query_points );
        end

        count_of_reachable_votes    = sum( y == 1, 2 );
        bool_vector                 = (count_of_reachable_votes >= (Nhypotheses/2));
        if mod(Nhypotheses,2) == 0
            tie_votes                   = ( count_of_reachable_votes == (Nhypotheses/2) );
            bool_vector( tie_votes )    = ( rand( sum(tie_votes), 1 ) >= 0.5 );
        end
    else
        bool_vector = ( svmclassify( svm_output, query_points ) == 1 );
    end
end