/*
 * sum3DMat.cpp - example in MATLAB External Interfaces
 *
 * sum over all entries stored in a 3D matrix which
 * is a field in a struct and stores in another
 * field in the struct
 *
 * This is a MEX file for MATLAB.
*/

#include "mex.h"
#include "matrix.h"

/* the gateway function */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

    /* get pointer to data in multidimensional array */
    double *inMatrix = mxGetPr(prhs[0]);   // get pointer to input matrix
    int nDims = mxGetNumberOfDimensions(prhs[0]);    // get number of dimensions in matrix
    const int* dims = mxGetDimensions(prhs[0]);     // get dimensions of input matrix
    
    /* calculate sum */
    double sum = 0.0;
    for(int k=0; k<dims[2]; k++){
        for(int j=0; j<dims[1]; j++){
            for(int i=0; i<dims[0]; i++){
                sum = sum + inMatrix[i + j*dims[0] + k*dims[0]*dims[1]];
            }
        }
    }

    /* output */
    plhs[0] = mxCreateDoubleScalar(sum);
}
