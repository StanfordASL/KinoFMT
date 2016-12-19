/*
 * CheckCollisionMEX.cpp - is the cpp implementation of checkCollision
 *
 * AUTHOR:  Ross Allen
 * DATE:    Oct 23, 2015
 *
 * INPUTS:
 *      nodes:      [3,nNodes] array containing 3D location of first nodes
 *      obs:        [3,2*nObs] matrix storing upper and lower vertices of m obstacles
 *      bounds:     [3,2] matrix containing the environmental bounds
 *
 * OUTPUTS:
 *      valid:      boolean operator. True if no collision
 *
 * NOTES:
 *      - assumes 3D, rectangular, axis aligned obstacles
 *      - assumes at least 2 nodes
 *      - nodes and obs are transpose of nodes2check and ulVerts used in the checkCollision Mfile, be careful
 * 		- based on work by Brian Ichter
 *
*/

#include "mex.h"
#include "matrix.h"
#include <iostream>

const int DIM = 3;

bool faceContainsProjection(const double *v, const double *v_to_w, const double lambda, int j, const double *obs)
{
	for (int d = 0; d < DIM; ++d) {
		double projection = v[d] + v_to_w[d]*lambda;
		if (d != j && !(obs[d] <= projection && 
			projection <= obs[DIM+d]))
			return false;
	}
	return true;
}

bool motionValidQ(const double *v, const double *w, const double *obs) {
	double v_to_w[DIM];
	for (int d = 0; d < DIM; ++d) {
		v_to_w[d] = w[d] - v[d];
	}

	for (int d = 0; d < DIM; ++d) {
		double lambda;
		if (v[d] < obs[d]) {
			lambda = (obs[d] - v[d])/v_to_w[d];
		} else {
			lambda = (obs[DIM + d] - v[d])/v_to_w[d];
		}
		if (faceContainsProjection(v, v_to_w, lambda, d, obs))
			return false;
	}
	return true;
}

bool broadphaseValidQ(const double *bb_min, const double *bb_max, const double *obs) {
	for (int d = 0; d < DIM; ++d) {
		if (bb_max[d] <= obs[d] || obs[DIM+d] <= bb_min[d]) 
			return true;
	}
	return false;
}

bool isMotionValid(const double *v, const double *w, const double *bb_min, const double *bb_max, int obstaclesCount, const double *obstacles)
{
	// go through each obstacle and do broad then narrow phase collision checking
	for (int obs_idx = 0; obs_idx < obstaclesCount; ++obs_idx) {
		double obs[DIM*2];
		for (int d = 0; d < DIM; ++d) {
			obs[d] = obstacles[obs_idx*2*DIM + d];
			obs[DIM+d] = obstacles[obs_idx*2*DIM + DIM + d];
		}
	
		if (!broadphaseValidQ(bb_min, bb_max, obs)) {
			if (!motionValidQ(v, w, obs)) {
				return false;
			}
		} 
	}
	return true;
}


/* the gateway function */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

    /* No input checks performed for performance. User is responsible for proper input*/

    /* extract information nodes */
    double *nodes = mxGetPr(prhs[0]);  // nodes
    const int* nodeDims = mxGetDimensions(prhs[0]); 
    double *obs = mxGetPr(prhs[1]); // upper and lower vectices of obstacles
    const int* obsDims = mxGetDimensions(prhs[1]);
    const int nObs = obsDims[1]/2;  // number of obstacles

    /* initialize output */
    bool valid = true;

    /* iterate through each pair of nodes */
    for (int nodeInd = 0; nodeInd < nodeDims[1]-1; nodeInd++){

        double *v = &nodes[nodeInd*DIM];        // be careful, no end-of-array check
        double *w = &nodes[(nodeInd+1)*DIM];    // be careful, no end-of-array check

        /* calculate bounds of the bounding box */
        double bb_min[DIM], bb_max[DIM];
        for (int d = 0; d < DIM; ++d) {
        	if (v[d] > w[d]) {
        		bb_min[d] = w[d];
        		bb_max[d] = v[d];
        	} else {
        		bb_min[d] = v[d];
        		bb_max[d] = w[d];
        	}
        }

        /* debugging */
//         std::cout << "nodeInd = " << nodeInd << std::endl;
//         mexPrintf("nodeInd = %d\n", nodeInd);
//         for (int deb_it = 0; deb_it < DIM; deb_it++){
//             std::cout << "v[" << deb_it << "] = " << (int)(1000.0*v[deb_it]) << std::endl;
//             mexPrintf("v[%d] = %d\n", deb_it, (int)(1000.0*v[deb_it]));
//         }

        /* check spacial boundaries */
        if (nrhs == 3){
            double *bounds = mxGetPr(prhs[2]);  // boundary of environment
            for (int i = 0; i < DIM; i++) {
                if (bb_min[i] < bounds[i] || bb_max[i] > bounds[i+DIM]) {
        			valid = false;
        			plhs[0] = mxCreateLogicalScalar(valid);
//                     mexPrintf("BOUNDARY COLLISION\n");
        			return;
        		}
            }
        }
        
        /* check obstacles */
        valid = isMotionValid(v, w, bb_min, bb_max, nObs, obs);
        if (!valid) {
            plhs[0] = mxCreateLogicalScalar(valid);
//             mexPrintf("OBSTACLE COLLISION\n");
            return;
        }
    }

    /* output */
    plhs[0] = mxCreateLogicalScalar(valid);
//     mexPrintf("collision free\n");
    return;
}
