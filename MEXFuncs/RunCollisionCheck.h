#ifndef _RUN_COLLISION_CHECK_H_
#define _RUN_COLLISION_CHECK_H_

#include "matrix.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>
#include <stdint.h>

// Other functions don't need be accessible by other files

/* pass information to collision checker */
bool runCollisionCheck(const uint32_t nConfDims, const uint64_t nBVPs,
                    const double * trajMat,
					const uint64_t vInd, const uint64_t wInd, 
                    const uint32_t nBounds, const double * bounds,
					const uint32_t nObs, const double * obs,
                    const uint32_t nSphObs, const double * sphObs);

bool runCollisionCheck(const uint32_t nConfDims,
					const double * vArr, const double * wArr, 
                    const uint32_t nBounds, const double * bounds,
					const uint32_t nObs, const double * obs,
                    const uint32_t nSphObs, const double * sphObs);

#endif