function matSum = sum3DMat_m(A)

matSum = 0;

for k = 1:size(A,3)
    for j = 1:size(A,2)
        for i = 1:size(A,1)
            matSum = matSum + A(i,j,k);
        end
    end
end

end
