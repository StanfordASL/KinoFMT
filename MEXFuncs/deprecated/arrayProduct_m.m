function outArr = arrayProduct_m(sclr, arr)

outArr = zeros(1,length(arr));

for i = 1:length(arr)
    outArr(i) = sclr*arr(i);
end

end