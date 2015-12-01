function A = Text2Matrix( FileName )
%Text2Matrix takes a formatted text file and outputs a matrix that is
%potentially multidimensional
%
%   AUTHOR: Ross Allen
%   DATE:   Jan 21, 2014
%
%   INPUTS:
%       FileName =  file name to print to
%
%   OUTPUTS:
%       A =         matrix with data
%
%   NOTES:
%       -"flattens" multidimensional array with indexing scheme:
%           original(i1, i2,..., in) = 
%           flat(j1 + d1*(j2 + d2*(j3 +...+ dn_2*(jn_1 + dn_1*jn)...)) + 1)
%           where j1=i1-1, j2=i2-1, ...
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% open file
fid = fopen(FileName);

% Extract number of dimensions
line = fgetl(fid);
[~, rem] = strtok(line, ':');
[~, rem] = strtok(rem);
numDims = int8(str2num(rem));

% Get size of each dimension
dims = zeros(1,numDims);
for i = 1:numDims
    line = fgetl(fid);
    [~, rem] = strtok(line, ':');
    [~, rem] = strtok(rem);
    dims(i) = floor(str2num(strtrim(rem)));
end

% Skip next line
line = fgetl(fid);

% Populate matrix
A = zeros(dims);
for flatind = 1:prod(dims)
    expind = zeros(1,numDims);
    k = flatind-1;
    for j = 1:numDims
        Q = floor(k/dims(j));
        R = k - Q*dims(j);
        expind(j) = R+1;
        k = Q;
    end
    expind = num2cell(expind);
    line = strtrim(fgetl(fid));
    if strcmp(line, 'Inf') || strcmp(line, 'NaN')
        A(expind{:}) = str2num(line);
    else
        [base, exp] = strtok(line, 'e');
        base = str2num(base);
        exp(1) = [];
        exp = str2num(exp);
        A(expind{:}) = base*10^exp;
    end
end


fclose(fid);
end
