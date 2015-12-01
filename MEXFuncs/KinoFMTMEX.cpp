/*
 * RunKinoFMTMEX.cpp - is the cpp implementation of RunKinoFMT
 *
 * AUTHOR:  Ross Allen
 * DATE:    Oct 30, 2015
 *
 * INPUTS:
 *      evalMat:	(uint64) [nSample, nSample] matrix holding case numbers for 2PBVP solutions
 *		costMat:	(double) [nBVPs, 1] vector holding cost of each 2PBVP
 *		trajMat:	(double) [nBVPs, nStateDims, nTrajNodes] 3D matrix holding state trajectory for each 2PBVP
 *		obsMat:		(double) [nConfDims,2*nObs] matrix storing upper and lower vertices of m obstacles
 *		bounds:		(double) [nConfDims,2] matrix containing the environmental bounds
 * 		outNeighbor:	(cell array of uint32) contains a sorted list of outgoing neighbors
 * 		inNeighbor:		(cell array of uint32) contains a sorted list of incoming neighbors
 * 		nGoalSamples:	(int) number of samples from goal region
 *
 * OUTPUTS:
 *
 * NOTES:
 *      - assumes rectangular, axis aligned obstacles
 *      - assumes at least 2 nodes
 *      - nodes and obs are transpose of nodes2check and ulVerts used in the checkCollision Mfile, be careful
 * 		- assumes Xstart is always node 1, and goal states are 2 to 1+nGoalSamples
 *      - "ID" is the label of a sampled state that is independent of matlab or cpp implementation. "ind" or some variation thereof is used to index in to vectors in C++
 *      - Neighborhoods must be passed as sorted ID list of uint32. This differs to prior 
 *          implementation that was sorted based on cost. error checking on this is not in place
 *      - evalMat must be passed as type uint64
*/

#include "mex.h"
#include "matrix.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>
#include <stdint.h>

typedef std::vector<uint32_t>::size_type uint_vec_size;
typedef std::vector<uint32_t>::iterator uint_vec_iter;

class Neighborhood {
    uint32_t nNeighbors;
    std::vector<uint32_t> neighborIDs;
    std::vector<double> neighborCosts;

    public:
        Neighborhood();
        Neighborhood(uint32_t);
        Neighborhood(uint32_t, const std::vector<uint32_t>&, const std::vector<double>&);
        void setNeighborData(uint32_t, uint32_t, double);
        const std::vector<uint32_t>& getIDs();
        uint32_t getID(uint32_t);
        double getCost(uint32_t);
};
Neighborhood::Neighborhood():
    // use initializer list to avoid reconstruction of ID and cost vectors
    nNeighbors(),
    neighborIDs(),
    neighborCosts(){
}
Neighborhood::Neighborhood(uint32_t N):
    // use initializer list to avoid reconstruction of ID and cost vectors
    nNeighbors(N),
    neighborIDs(N, 0),
    neighborCosts(N, 0.0){
}
Neighborhood::Neighborhood( uint32_t N, 
                            const std::vector<uint32_t>& IDs, 
                            const std::vector<double>& costs):
    // use initializer list to avoid reconstruction of ID and cost vectors
    nNeighbors(N),
    neighborIDs(IDs),
    neighborCosts(costs){
}
const std::vector<uint32_t>& Neighborhood::getIDs(){
    return neighborIDs;
}
void Neighborhood::setNeighborData(uint32_t ind, uint32_t ID, double cost){
    neighborIDs.at(ind) = ID;
    neighborCosts.at(ind) = cost;
}
uint32_t Neighborhood::getID(uint32_t ind){
    return neighborIDs.at(ind);
}
double Neighborhood::getCost(uint32_t ind){
    return neighborCosts.at(ind);
}

/* return the neighborhood of the pivot */
Neighborhood getNeighborhoodFromCell(const mxArray *neighborCellArray, uint32_t pivot){
    // extract cell of neighborhood from cell array
    mwIndex pivInd = (mwIndex)pivot-1;  // account for zero-indexing
    const mxArray *neighborCellElem = mxGetCell(neighborCellArray, pivInd);
    const double *neighborData = mxGetPr(neighborCellElem);
    const int *neighborDim = mxGetDimensions(neighborCellElem);

    // construct Neighborhood object
    uint32_t nNeighbors = neighborDim[0];
    Neighborhood hood(nNeighbors);
    for (uint32_t i=0; i<nNeighbors; i++){
        hood.setNeighborData(i, (uint32_t)neighborData[i], neighborData[i+nNeighbors]);
    }
    return hood;
}

/* return neighborhood ID array for the  pivot state.*/
uint32_t * getNeighborIDsFromCell(const mxArray *neighborCellArray, uint32_t pivot, 
                            uint32_t& nNeighbors){
    
    const mwIndex pivInd = (mwIndex)pivot-1;  // account for zero-indexing
    const mxArray *neighborCellElem = mxGetCell(neighborCellArray, pivInd);
    const int *neighborDim = mxGetDimensions(neighborCellElem); 
    nNeighbors = neighborDim[0];
    return (uint32_t*)mxGetData(neighborCellElem);
}

/* convert std::vector<uint32_t> to an mxArray */
mxArray * getMexUnsignedIntArray(const std::vector<uint32_t>& v){
    // NOTE: this function is not stable! causes following error:
    //malloc.c:3695: _int_malloc:Assertion '(unsigned long)(size) >= (unsigned long)(nb)' failed
    mxArray * mx = mxCreateNumericMatrix(v.size(), 1, mxUINT32_CLASS, mxREAL);
    std::copy(v.begin(), v.end(), mxGetPr(mx));
    return mx;
}

/* determine if uint32_t is in sorted vector */
bool inSortedVec(uint32_t z, const std::vector<uint32_t>& v){
    bool isPresent = false;
    for (int i = 0; i < v.size(); i++){
        if (z == v.at(i)){
            isPresent = true;
            break;
        } else if (z < v.at(i)){
            break;
        }
    }
    return isPresent;
}

/* find intersection of a sorted array and a sorted vector and store in vector */
void intersectSortedArrayAndVector(uint32_t nArr, uint32_t * arr, 
            std::vector<uint32_t>& vec, std::vector<uint32_t>& out){

    uint32_t maxOutSize = (nArr < (uint32_t)vec.size()) ? nArr : (uint32_t)vec.size();
    out.resize(maxOutSize);
    std::vector<uint32_t>::iterator enditer = 
        set_intersection(arr, arr+nArr, vec.begin(), vec.end(), out.begin());
    out.resize(enditer-out.begin());

}

/* test for line segment collision with obstacle faces */
bool faceContainsProjection(const uint32_t nConfDims, 
							const std::vector<double>& v, const std::vector<double>& v_to_w, 
							const double lambda, int j, const std::vector<double>& obs)
{
	for (int d = 0; d < nConfDims; ++d) {
		double projection = v[d] + v_to_w[d]*lambda;
		if (d != j && !(obs[d] <= projection && 
			projection <= obs[nConfDims+d]))
			return false;
	}
	return true;
}

/* test for collision with obstacle */
// Could this be improved by recording which obstacles violate the bounding box
//	and only passing those to motionValid?
bool motionValidQ(	const uint32_t nConfDims, 
					const std::vector<double>& v, const std::vector<double>& w,
                    const std::vector<double>& obs)
{
	std::vector<double> v_to_w(nConfDims);
	for (int d = 0; d < nConfDims; ++d) {
		v_to_w[d] = w[d] - v[d];
	}

	for (int d = 0; d < nConfDims; ++d) {
		double lambda;
		if (v[d] < obs[d]) {
			lambda = (obs[d] - v[d])/v_to_w[d];
		} else {
			lambda = (obs[nConfDims + d] - v[d])/v_to_w[d];
		}
		if (faceContainsProjection(nConfDims, v, v_to_w, lambda, d, obs))
			return false;
	}
	return true;
}

/* test for bounding box proximity to obstacles */
// Could this be improved by recording which obstacles violate the bounding box
//	and only passing those to motionValid?
bool broadphaseValidQ(	const uint32_t nConfDims, 
						const std::vector<double>& bb_min, const std::vector<double>& bb_max, 
						const std::vector<double>& obs)
{
	for (int d = 0; d < nConfDims; ++d) {
		if (bb_max[d] <= obs[d] || obs[nConfDims+d] <= bb_min[d]) 
			return true;
	}
	return false;
}

bool isMotionValid(	const uint32_t nConfDims, 
                    const std::vector<double>& v, const std::vector<double>& w,
                    const std::vector<double>& bb_min, const std::vector<double>& bb_max,
					int obstaclesCount, const double *obstacles)
{
	// go through each obstacle and do broad then narrow phase collision checking
	for (int obs_idx = 0; obs_idx < obstaclesCount; ++obs_idx) {
		std::vector<double> obs(nConfDims*2);
		for (int d = 0; d < nConfDims; ++d) {
			obs[d] = obstacles[obs_idx*2*nConfDims + d];
			obs[nConfDims+d] = obstacles[obs_idx*2*nConfDims + nConfDims + d];
		}
	
		if (!broadphaseValidQ(nConfDims, bb_min, bb_max, obs)) {
			if (!motionValidQ(nConfDims, v, w, obs)) {
				return false;
			}
		} 
	}
	return true;
}

/* collision checker */
/*
 * INPUTS:
 *      obs:        [nConfDims,2*nObs] matrix storing upper and lower vertices of m obstacles
 *      bounds:     [nConfDims,2] matrix containing the environmental bounds
*/
bool checkCollision(const uint32_t nConfDims, const uint64_t nBVPs,
                    const double * trajMat,
					const uint64_t vInd, const uint64_t wInd, 
					const uint32_t nObs, const double * obs,
					const double * bounds)
{
    /* initialize output */
    bool valid = true;

    /* extract nodes and calculate bounds of the bounding box */
    std::vector<double> v(nConfDims);
    std::vector<double> w(nConfDims);
    std::vector<double> bb_min(nConfDims);
    std::vector<double> bb_max(nConfDims);
    for (uint32_t d = 0; d < nConfDims; ++d) {
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

    /* check spacial boundaries */
	for (int i = 0; i < nConfDims; i++) {
		if (bb_min[i] < bounds[i] || bb_max[i] > bounds[i+nConfDims]) {
			valid = false;
			return valid;
		}
	}
    
    /* check obstacles */
    valid = isMotionValid(nConfDims, v, w, bb_min, bb_max, nObs, obs);
    if (!valid) {
        return valid;
    }

	return valid;
}

/* insert sorted nufrontier in sorted frontier and remove pivot */
void processFrontier(   const std::vector<uint32_t>& nuVec, std::vector<uint32_t>& baseVec,
                        uint32_t& pivot, const std::vector<double>& cost2Come)
{
    std::vector<uint32_t> resVec;
    resVec.resize(baseVec.size() + nuVec.size());  // resize (account for pivot removal later)
    uint32_t nuInd = 0;
    uint32_t baseInd = 0;
    uint32_t resInd = 0;
    double minCostPivot = std::numeric_limits<double>::infinity();
    uint32_t nuPivot = 0;

    /* merge overlapping portions */
    while (resInd < resVec.size()) {
        if (nuInd < nuVec.size()){
            if(baseInd < baseVec.size()){

                // elements left in both nuVec ans baseVec, compare
                if (nuVec.at(nuInd) < baseVec.at(baseInd)) {
                    resVec.at(resInd) = nuVec.at(nuInd);
                    ++nuInd;
                } else {
                    resVec.at(resInd) = baseVec.at(baseInd);
                    ++baseInd;
                }

            } else {
                // still elements left in nuVec, copy in order
                resVec.at(resInd) = nuVec.at(nuInd);
                ++nuInd;
            }   
        } else {
            // still elements left in baseVec, copy in order
            resVec.at(resInd) = baseVec.at(baseInd);
            ++baseInd;  
        }

//         mexPrintf("DEBUG: resVec.at(%d) = %d\n", resInd, resVec.at(resInd));

        /* remove pivot from frontier or determine new pivot */
        // Since frontSet and nuFront set should be unique elements, this should only happen once
        if (resVec.at(resInd) == pivot){
            resVec.pop_back();

        } else {
            
            /* determine minimum cost for next pivot */
            double curCostPivot = cost2Come.at(resVec.at(resInd)-1);    // account for zero indexing
            if (curCostPivot < minCostPivot) {
                minCostPivot = curCostPivot;
                nuPivot = resVec.at(resInd);
            }
    
            /* increment iterator */
            ++resInd;
        }
        
    }
    
    /* swap variables */
    baseVec.swap(resVec);
    pivot = nuPivot;
}

/* return optimal path from tree */
void getOptimalPath(const std::vector<uint32_t>& parents, 
                    uint32_t Xstart, uint32_t Xgoal, 
                    std::vector<uint32_t>& optPath)
{ 
    uint32_t x2 = Xgoal;
    uint32_t x1 = parents.at(x2-1);
    optPath.push_back(x2);
    optPath.push_back(x1);
    
    /* walk back through tree */
    while( x1 != Xstart ){
        x2 = x1;
        x1 = parents.at(x2-1);
        optPath.push_back(x1);
    }

    /* reverse vector order - O(n/2) */
    std::reverse(optPath.begin(), optPath.end());

}

/* the gateway function */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    /* Input error checks (minimal, user required to ensure proper input) */
    
    /***********************/
    /* Extract information */
    /***********************/
    const uint64_t * evalMat 	= 	(uint64_t *)mxGetData(prhs[0]);
    const double * costMat 	= 	mxGetPr(prhs[1]);
    const double * trajMat 	= 	mxGetPr(prhs[2]);
    const double * obsMat 	= 	mxGetPr(prhs[3]);
    const double * bounds	= 	mxGetPr(prhs[4]);
    const mxArray * outNeighborIDs		=	prhs[5];
    const mxArray * inNeighborIDs		= 	prhs[6];
    const uint32_t nGoalSamples	=	(int)mxGetScalar(prhs[7]);
    const int * evalMatDims =	mxGetDimensions(prhs[0]);
    const int * costMatDims =   mxGetDimensions(prhs[1]); 
    const int * trajMatDims	=	mxGetDimensions(prhs[2]);
    const int * obsMatDims	= 	mxGetDimensions(prhs[3]);
    const uint32_t  costMatLen  =   costMatDims[0];
    const uint32_t nSamples 	=	evalMatDims[0];
    const uint64_t nBVPs   =	trajMatDims[0];
    const uint32_t nStateDims	=	trajMatDims[1];
    const uint32_t nTrajNodes	=	trajMatDims[2];
    const uint32_t nConfDims	=	obsMatDims[0];
    const uint32_t nObs			=	obsMatDims[1]/2;
    
//     for (uint32_t i = 0; i < nSamples*nSamples; ++i){
//         mexPrintf("DEBUG: evalMat(%d) = %d\n", i, evalMat[i]);
//     }
    
    /*********************/
    /* Prepare variables */
    /*********************/
    const uint32_t Xstart = 1;       // start state

    /* goal states */
    std::vector<uint32_t> Xgoal(nGoalSamples);   // goal states
    for (uint32_t iter = 0; iter < nGoalSamples; iter++){
		Xgoal.at(iter) = 2 + iter;
	}

    /* edge set: store edges in tree */
    // Note a change in format from mfile implementation
    // Note may be irrelevant with parents implementation
    std::vector< uint64_t > edgeSet;
    edgeSet.reserve(nBVPs);                 // allocate memory but don't change size

    /* unexplored set of nodes */
    std::vector<uint32_t> unexSet;          // states not in tree
    unexSet.resize(nSamples-1);                   // allocate memory and change size of vector
    for (uint32_t iter = 0; iter < nSamples-1; iter++){
        unexSet.at(iter) = 2 + iter;
    }

    /* frontier set of nodes */
    std::vector<uint32_t> frontSet;     // states in frontier of tree
    frontSet.push_back(Xstart);             // no need to reserve memory due to sortedInsertMerge

    uint32_t z = Xstart;                // initial pivot
    std::vector<double> cost2Come(nSamples, 0.0);   // min cost to reach each node
    std::vector<uint32_t> parents(nSamples, 0); // parent node in tree of each node in tree

    /***********************************/
    /* Run through kino-FMT while loop */
    /***********************************/
    while(!inSortedVec(z, Xgoal)){
        
        /* Outgoing neighborhood of z */
        uint32_t nNzOut;    // number of states in z outgoing neighborhood
        uint32_t * NzOut = getNeighborIDsFromCell(outNeighborIDs, z, nNzOut);
//         mexPrintf("DEBUG: z = %d -------------------------- \n", z);

        /* new states to be placed in frontier set */
        std::vector<uint32_t> nuFrontSet;
        nuFrontSet.reserve(nSamples);       // allocate memory but don't change size
        
        /* set of unexplored states in outgoing neighborhood of z */
        std::vector<uint32_t> Xnear;
        intersectSortedArrayAndVector(nNzOut, NzOut, unexSet, Xnear);
        
        /* iterate through Xnear */
        for (uint_vec_iter xIter = Xnear.begin(); xIter != Xnear.end(); ++xIter){
            uint32_t x = *xIter;
            uint32_t xInd = x-1;
            
            /* incoming neighborhood of x */
            uint32_t nNxIn;  // number of states in x incoming neighborhood
            uint32_t * NxIn = getNeighborIDsFromCell(inNeighborIDs, x, nNxIn);
            
            /* set of frontier states in incoming neighborhood of x */
            std::vector<uint32_t> Ynear;
            intersectSortedArrayAndVector(nNxIn, NxIn, frontSet, Ynear);
            
            uint32_t yMin;
            uint64_t yMinxBVP;
            double xMinCost2Come = std::numeric_limits<double>::infinity();
            
            /* iterate through Ynear */
            for (uint_vec_iter yIter = Ynear.begin(); yIter != Ynear.end(); ++yIter){
                uint32_t y = *yIter;
                uint32_t yInd = y-1;

                /* access BVP case number for yx edge */
//                 mexPrintf("DEBUG: xInd = %d, nSamples = %d\n", xInd, nSamples);
                uint64_t yxBVP = evalMat[yInd + nSamples*xInd];
                uint64_t yxBVPInd = yxBVP-1;
                
                /* determine cost to come to x via y */
//                 mexPrintf("DEBUG: yInd = %d, yxBVPInd = %d\n", yInd, yxBVPInd);
                double yxCost2Come = cost2Come.at(yInd) + costMat[yxBVPInd];
 
                /* perform dynamic programming */
                if (yxCost2Come <= xMinCost2Come){
                    xMinCost2Come = yxCost2Come;
                    yMin = y;
                    yMinxBVP = yxBVP;
                } 
            }

//             mexPrintf("DEBUG: min cost to %d is %d via %d\n", x, (int)(100.0*xMinCost2Come), yMin);

            /* check collisions */
            bool collisionFree;
            for (uint32_t nodeInd = 0; nodeInd < nTrajNodes-1; ++nodeInd){
                
                uint64_t vInd = yMinxBVP-1 + nodeInd*nBVPs*nStateDims;
                uint64_t wInd = vInd + nBVPs*nStateDims;
//                 mexPrintf("DEBUG: collision check: vInd=%d, wInd=%d \n", vInd, wInd);
                collisionFree = checkCollision(nConfDims, nBVPs, trajMat, vInd, wInd, 
					                                nObs, obsMat, bounds);
//                 mexPrintf("DEBUG: collision check: y=%d, x=%d, nodeInd=%d: %d\n", yMin, x, nodeInd, collisionFree);

                /* if any segment is in collision, break */
                if (!collisionFree) {
                    break;
                }
            }

            /* update costMat or tree */
            if (collisionFree) {
                /* update tree edge set */
                edgeSet.push_back(yMinxBVP); 

                /* update new frontier set */
                nuFrontSet.push_back(x);

                /* update cost to come vector */
                cost2Come.at(x-1) = xMinCost2Come;

                /* update parent set */
                parents.at(x-1) = yMin;

                /* remove explored state from unexplored set. Should remain sorted */
                std::vector<uint32_t>::iterator xUnexIt;
                xUnexIt = std::find(unexSet.begin(), unexSet.end(), x);
                unexSet.erase(xUnexIt);
            }

            //} else {

            //    /* update costMat with collision info */
            //    // NOTE: due to memory management, perhaps this update should not be done
            //    costMat[yMinxBVP-1] = std::numeric_limits<double>::infinity();
            //}
                     
            
        }

        /* Merge frontier set and new frontier set, extract z, and determine new pivot */
        // NOTE: both frontSet and nuFrontSet should be sorted by construction
        // keep sorted when merged
        processFrontier(nuFrontSet, frontSet, z, cost2Come);
//         mexPrintf("DEBUG: frontSet.size() = %d\n", frontSet.size());
//         for (int i = 0; i < frontSet.size(); ++i) {
//             mexPrintf("DEBUG frontSet[%d] = %d\n", i, frontSet[i]);
//         }

        /*  check for failure condition */
        if (frontSet.size() == 0){
            // Should move print statement outside of timed function to not affect computation time
            mexPrintf("DEBUG: FMT FAILURE: frontier set has emptied before Xgoal reached\n");
            
            /* fill failed outputs */
            // TO BE COMPLETED
            break;
        }
    }

//     mexPrintf("DEBUG: SOLUTION\n");
//     for (int i = 0; i < edgeSet.size(); ++i) {
//         mexPrintf("DEBUG: edgeSet[%d] = %d\n", i, edgeSet[i]);
//     }
    
    /* return optimal path from tree */
    std::vector<uint32_t> optPath;
    optPath.reserve(nSamples);      // at most, the tree can have nSamples
    getOptimalPath(parents, Xstart, z, optPath);
    
//     for (int i = 0; i < optPath.size(); ++i) {
//         mexPrintf("DEBUG: optPath[%d] = %d\n", i, optPath[i]);
//     }
//     
//     for (int i = 0; i < cost2Come.size(); ++i) {
//         mexPrintf("DEBUG: cost2Come[%d] = %d\n", i, (int)(1000.0*cost2Come[i]));
//     }

    /*****************/
    /* Format output */
    /*****************/
    //plhs[0] = getMexUnsignedIntArray(unexSet);
    //plhs[0] = mxCreateLogicalScalar(true);
    /* optimal path */
    plhs[0] = mxCreateNumericMatrix(optPath.size(), 1, mxUINT32_CLASS, mxREAL);
    uint32_t * optPathOutput = (uint32_t *)mxGetData(plhs[0]);
    for (uint32_t i = 0; i < optPath.size(); ++i){
        optPathOutput[i] = optPath.at(i);
    }
    
    /* optimal cost */
    plhs[1] = mxCreateDoubleScalar(cost2Come.at(z-1));
    
    /* exploration tree */
    plhs[2] = mxCreateNumericMatrix(parents.size(), 1, mxUINT32_CLASS, mxREAL);
    uint32_t * parentsOutput = (uint32_t *)mxGetData(plhs[2]);
    for (uint32_t i = 0; i < parents.size(); ++i){
        parentsOutput[i] = parents.at(i);
    }
    
    return;
}
