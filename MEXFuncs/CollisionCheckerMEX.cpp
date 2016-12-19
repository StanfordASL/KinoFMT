/*
 * CollisionCheckerMEX.cpp - is the cpp implementation of RunKinoFMT
 *
 * AUTHOR:  Ross Allen
 * DATE:    Feb 02, 2015
 *
 * INPUTS:
 *      vArr:       (double) [nWorkDims] array containing the first node
 *      wArr:       (double) [nWorkDims] array containing the second node
 *		bounds:		(double) [nWorkDims,2] matrix containing the environmental bounds
 *		obsMat:		(double) [nWorkDims,2*nObs] matrix storing upper and lower vertices of m obstacles
 *      sphObsMat:  (double) [nWorkDims+1,nSphObs] matrix storing position and radius of sphereical obstacles
 *
 * OUTPUTS:
 *
 * NOTES:
*/

#include "mex.h"
#include "RunCollisionCheck.h"


/* the gateway function */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    /* Input error checks (minimal, user required to ensure proper input) */
    
    /***********************/
    /* Extract information */
    /***********************/
    const double * vArr 	= 	mxGetPr(prhs[0]);
    const double * wArr 	= 	mxGetPr(prhs[1]);
    const uint32_t nWorkDims	=	(int)mxGetScalar(prhs[2]);
    const double * bounds 	= 	mxGetPr(prhs[3]);
    const double * obsMat 	= 	mxGetPr(prhs[4]);
    const double * sphObsMat 	= 	mxGetPr(prhs[5]);
    const int * boundsDims      = mxGetDimensions(prhs[3]);
    const int * obsMatDims	= 	mxGetDimensions(prhs[4]);
    const int * sphObsMatDims   = mxGetDimensions(prhs[5]);
    const uint32_t nBounds      =   boundsDims[0];
    const uint32_t nObs			=	obsMatDims[1]/2;
    const uint32_t nSphObs      =   sphObsMatDims[1];
    
    
    /***********************/
    /* Run Collision Check */
    /***********************/
    bool valid = runCollisionCheck(nWorkDims, vArr, wArr, nBounds, bounds,
					nObs, obsMat,
                    nSphObs, sphObsMat);

    /*****************/
    /* Format output */
    /*****************/
    plhs[0] = mxCreateLogicalScalar(valid);
    
    
    return;
}