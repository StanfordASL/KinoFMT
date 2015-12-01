/*
 * FactorialRatioMEX returns factorial(i)/factorial(i-j) where i>=j
 *
 * AUTHOR:  Ross Allen
 * DATE:    Nov 12, 2015
 *
 * INPUTS:
 *      i:  [int]   i >= j
 *      j:  [int] 
 *
 * OUTPUTS:
 *      res:    [uint64_t] factorial(i)/factorial(i-j)
 *
 * NOTES:
 *      - does not perform error checks on inputs
 *
*/

#include "mex.h"
#include <stdint.h>
#include <iostream>




/* the gateway function */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

    /* No input checks performed for performance. User is responsible for proper input*/

    /* extract information nodes */
    const int I = (int)mxGetScalar(prhs[0]);
    const int J = (int)mxGetScalar(prhs[1]);

    /* calculate ratio of integerts */
    uint64_t factrat = 1;
    for (int iter = I; iter > I-J; --iter){
        factrat *= (I-(I-iter));
    }
    

    /* output */
    plhs[0] = mxCreateDoubleScalar((double)factrat);
    return;
}
