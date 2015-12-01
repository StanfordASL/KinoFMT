function err = Matrix2Text( A, FileName )
%Matrix2Text transforms a multidimensional matrix into a text file that
%prints it as a 1D array
%
%   AUTHOR: Ross Allen
%   DATE:   Jan 16, 2014
%
%   INPUTS:
%       A =         matrix to be converted (arbitrary size)
%       FileName =  file name to print to
%
%   NOTES:
%       -"flattens" multidimensional array with indexing scheme:
%           original(i1, i2,..., in) = 
%           flat(j1 + d1*(j2 + d2*(j3 +...+ dn_2*(jn_1 + dn_1*jn)...)) + 1)
%           where j1=i1-1, j2=i2-1, ...
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
err = 0;
n = ndims(A);               % number of dimensions of matrix
Aflat = reshape(A,[],1);    % reshape matrix into a single vector 
fid = fopen(FileName, 'W');
fprintf(fid, 'Number of dimensions in matrix: %u \r\n', n);

for i = 1:n
    fprintf(fid, 'Length of dimension %u: %u \r\n', i, size(A,i));
end

fprintf(fid, 'values: \r\n');

for i = 1:length(Aflat)
    fprintf(fid, '%12.6e \r\n', Aflat(i));
end

fclose(fid);
end

