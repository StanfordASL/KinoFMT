clear
clc

s = 3.14159;
A = 2000*rand(1,100)-1000;

tic
ranB_mex = arrayProduct(s, A);
t_mex = toc;

tic
B_mfile = arrayProduct_m(s,A);
t_mfile = toc;

t_mex/t_mfile