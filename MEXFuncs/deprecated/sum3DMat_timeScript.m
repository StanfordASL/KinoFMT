clear
clc

A = rand(100,100,100);

tic
sum_mex = sum3DMat(A);
t_mex = toc;

tic
sum_mfile = sum3DMat_m(A);
t_mfile = toc;

t_mex/t_mfile
