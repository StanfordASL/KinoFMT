/*
 * CheckCollisionCuboidSphere.cpp - is the cpp implementation of checkCollision
 *
 * AUTHOR:  Ross Allen
 * DATE:    Oct 23, 2015
 *
 * INPUTS:
 *
 * OUTPUTS:
 *      valid:      boolean operator. True if no collision
 *
 * NOTES:
 *      - assumes 2D or 3D axis aligned cuboids and spheres
 *      - assumes at least 2 nodes
 *      - nodes and obs are transpose of nodes2check and ulVerts used in the checkCollision Mfile, be careful
*/

#include "mex.h"
#include "RunCollisionCheck.h"

#define _EPS_ 1.0e-15

/* test for line segment collision with obstacle faces */
bool faceContainsProjection(const uint32_t nWorkDims, 
							const std::vector<double>& v, const std::vector<double>& v_to_w, 
							const double lambda, int j, const std::vector<double>& obs)
{
	for (int d = 0; d < nWorkDims; ++d) {
		double projection = v[d] + v_to_w[d]*lambda;
		if (d != j && !(obs[d] <= projection && 
			projection <= obs[nWorkDims+d]))
			return false;
	}
	return true;
}

/* test for collision with obstacle */
bool motionValidQ(	const uint32_t nWorkDims, 
					const std::vector<double>& v, const std::vector<double>& w,
                    const std::vector<double>& obs)
{
	std::vector<double> v_to_w(nWorkDims);
	for (int d = 0; d < nWorkDims; ++d) {
		v_to_w[d] = w[d] - v[d];
	}

	for (int d = 0; d < nWorkDims; ++d) {
		double lambda;
		if (v[d] < obs[d]) {
			lambda = (obs[d] - v[d])/v_to_w[d];
		} else {
			lambda = (obs[nWorkDims + d] - v[d])/v_to_w[d];
		}
		if (faceContainsProjection(nWorkDims, v, v_to_w, lambda, d, obs))
			return false;
	}
	return true;
}

/* test for min distance to sphere */
// see mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
bool minDistValidQ( const uint32_t nWorkDims, 
					const std::vector<double>& x1, const std::vector<double>& x2, 
					const std::vector<double>& sph)
{
    bool valid = true;
    
    // Calc sqr distances between nodes and dot products
    double x10_2 = 0.0;
    double x20_2 = 0.0;
    double x21_2 = 0.0;
    double x21_d_x10 = 0.0;
    for (int d = 0; d < nWorkDims; ++d) {
        double x10_cur = x1[d]-sph[d];
        double x20_cur = x2[d]-sph[d];
        double x21_cur = x2[d]-x1[d];
        x10_2 += x10_cur * x10_cur;
        x20_2 += x20_cur * x20_cur;
        x21_2 += x21_cur * x21_cur;
        x21_d_x10 += x21_cur * x10_cur;
    }
    
    // Check individual node distance
    double radSqr = sph[nWorkDims]*sph[nWorkDims];
    if ( x10_2 <= radSqr || x20_2 <= radSqr) {
        // collision occurred
        return false;
    } else if (x21_2 < _EPS_) {
        // this conditions avoids division by zero in case x1==x2
        // if x1 and x2 are outside of the radius and the distance between
        // x1 and x2 is zero, then the intersecting line cannot be in collision
        return true;
    }
    
    // Check minimum distance with obstacle radius
    double t_param = - x21_d_x10 / x21_2;
    double distSqr = x10_2 + t_param*x21_d_x10;
    if (distSqr <= radSqr) {
        // Check parameter to determine if min point is between original points
        if (0.0 <= t_param && t_param <= 1.0){
            // collision occurred
            return false;
        }
    }
    return valid;
}

/* test for bounding box proximity to obstacles */
bool broadphaseValidQ(	const uint32_t nWorkDims, 
						const std::vector<double>& bb_min, const std::vector<double>& bb_max, 
						const std::vector<double>& obs)
{
	for (int d = 0; d < nWorkDims; ++d) {
		if (bb_max[d] <= obs[d] || obs[nWorkDims+d] <= bb_min[d]) 
			return true;
	}
	return false;
}

bool isMotionValid(	const uint32_t nWorkDims, 
                    const std::vector<double>& v, const std::vector<double>& w,
                    const std::vector<double>& bb_min, const std::vector<double>& bb_max,
					int obstaclesCount, const double *obstacles,
                    int sphObstacleCount, const double *sphObstacles)
{
	// go through each cuboid obstacle and do broad then narrow phase collision checking
	for (int obs_idx = 0; obs_idx < obstaclesCount; ++obs_idx) {
		std::vector<double> obs(nWorkDims*2);
		for (int d = 0; d < nWorkDims; ++d) {
			obs[d] = obstacles[obs_idx*2*nWorkDims + d];
			obs[nWorkDims+d] = obstacles[obs_idx*2*nWorkDims + nWorkDims + d];
		}
	
		if (!broadphaseValidQ(nWorkDims, bb_min, bb_max, obs)) {
			if (!motionValidQ(nWorkDims, v, w, obs)) {
				return false;
			}
		} 
	}
    
    // go through each sphere obstacle doing broad and narrow collisioin checking
    for (int sph_idx = 0; sph_idx < sphObstacleCount; ++sph_idx){
        std::vector<double> sphObs(nWorkDims+1);    // sphere position and radius
        std::vector<double> sphBB(nWorkDims*2);     // sphere bounding box ul verts
        double rad = sphObstacles[sph_idx*(nWorkDims+1)+nWorkDims]; // radius of sphere
        for (int d = 0; d < nWorkDims; ++d) {
            sphObs[d] = sphObstacles[sph_idx*(nWorkDims+1)+d];
            sphBB[d] = sphObstacles[sph_idx*(nWorkDims+1)+d] - rad; // lower bound
            sphBB[nWorkDims+d] = sphObstacles[sph_idx*(nWorkDims+1)+d] + rad; // upper bound
        }
        
        sphObs[nWorkDims] = rad;
        
        if (!broadphaseValidQ(nWorkDims, bb_min, bb_max, sphBB)) {
            if (!minDistValidQ(nWorkDims, v, w, sphObs)) {
                return false;
            }
        }
    }
    
	return true;
}

bool checkBoundsCuboidsSpheres(const uint32_t nWorkDims,
                    const std::vector<double>& v, const std::vector<double>& w,
                    const std::vector<double>& bb_min, const std::vector<double>& bb_max,
					int nBounds, const double * bounds, int nObs, const double * obs,
                    int nSphObs, const double * sphObs){
    
    bool valid = true;
    
    /* check spacial boundaries */
	for (int i = 0; i < nBounds; i++) {
		if (bb_min[i] < bounds[i] || bb_max[i] > bounds[i+nWorkDims]) {
			valid = false;
			return valid;
		}
	}
    
    /* check obstacles */
    valid = isMotionValid(nWorkDims, v, w, bb_min, bb_max, nObs, obs, nSphObs, sphObs);
    return valid;
}
    

/* collision checker */
bool runCollisionCheck(const uint32_t nWorkDims, const uint64_t nBVPs,
                    const double * trajMat,
					const uint64_t vInd, const uint64_t wInd, 
                    const uint32_t nBounds, const double * bounds,
					const uint32_t nObs, const double * obs,
                    const uint32_t nSphObs, const double * sphObs)
{
    /* initialize output */
    bool valid = true;

    /* extract nodes and calculate bounds of the bounding box */
    std::vector<double> v(nWorkDims);
    std::vector<double> w(nWorkDims);
    std::vector<double> bb_min(nWorkDims);
    std::vector<double> bb_max(nWorkDims);
    for (uint32_t d = 0; d < nWorkDims; ++d) {
        v[d] = trajMat[vInd + d*nBVPs];
        w[d] = trajMat[wInd + d*nBVPs];
       if (v[d] > w[d]) {
    		bb_min[d] = w[d];
    		bb_max[d] = v[d];
    	} else {
    		bb_min[d] = v[d];
    		bb_max[d] = w[d];
    	}
    }
    
    valid =  checkBoundsCuboidsSpheres(nWorkDims, v, w,
                    bb_min, bb_max, nBounds, bounds, nObs, obs, nSphObs, sphObs);
                    
	return valid;
}

/* collision checker */
bool runCollisionCheck(const uint32_t nWorkDims,
					const double * vArr, const double * wArr, 
                    const uint32_t nBounds, const double * bounds,
					const uint32_t nObs, const double * obs,
                    const uint32_t nSphObs, const double * sphObs)
{
    /* initialize output */
    bool valid = true;

    /* extract nodes and calculate bounds of the bounding box */
    std::vector<double> v(nWorkDims);
    std::vector<double> w(nWorkDims);
    std::vector<double> bb_min(nWorkDims);
    std::vector<double> bb_max(nWorkDims);
    for (uint32_t d = 0; d < nWorkDims; ++d) {
        v[d] = vArr[d];
        w[d] = wArr[d];
       if (v[d] > w[d]) {
    		bb_min[d] = w[d];
    		bb_max[d] = v[d];
    	} else {
    		bb_min[d] = v[d];
    		bb_max[d] = w[d];
    	}
    }
    
    valid =  checkBoundsCuboidsSpheres(nWorkDims, v, w,
                    bb_min, bb_max, nBounds, bounds, nObs, obs, nSphObs, sphObs);
                    
	return valid;
}