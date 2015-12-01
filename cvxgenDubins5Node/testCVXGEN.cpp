/* 
 * File:   testCVXGEN.cpp
 * Author: Ross Allen
 *
 * Created on January 2, 2014, 5:34 PM
 */

#include "solver.h"

#define _USE_MATH_DEFINES

#include <cstdlib>
#include <limits>


// Instantiate global namespace
Vars vars;
Params params;
Workspace work;
Settings settings;

using namespace std;


/*
 * 
 */
int main(int argc, char** argv) {

    // Extract dimensions
    int n_vars = sizeof(params.oldX)/sizeof(params.oldX[0]);
    int n_naec = sizeof(params.NAEC)/sizeof(params.NAEC[0]);
    int n_trust = sizeof(params.s_th)/sizeof(params.s_th[0]);
    
    // Set params that are constant for a given robot/dynamics
    params.turnrate[0] = 45;            // (deg/s) maximum turnrate
    params.t0[0] = 0;                   // (s) initial time
    
    // Set params that are constant for a given optimal control problem
    params.x0[0] = 0;                   // (m) x-initial pos
    params.y0[0] = 0;                   // (m) y-initial pos
    params.th0[0] = 0;                  // (deg) initial heading
    params.xf[0] = 0;                   // (m) x-final pos
    params.yf[0] = 8/M_PI;              // (m) y-final pos
    params.thf[0] = 180;                // (deg) final heading
    params.ylower[0] = -std::numeric_limits<double>::max();
    params.yupper[0] = std::numeric_limits<double>::max();
    params.xlower[0] = -std::numeric_limits<double>::max();
    params.xupper[0] = std::numeric_limits<double>::max();
    
    // Set params that are constant for one penalty iteration (ref: J. Schulman)
    params.mu[0] = 0.5;
    
    // Set params that are constant for one convexify iteration (ref: J. Schulman)
    for (int iter1 = 0; iter1 <= n_naec-1; iter1++){
        params.NAEC[iter1] = 1.0;
        for (int iter2 = 0; iter2 <= n_vars-1; iter2++){
            params.gradNAEC[iter2 + iter1*n_naec] = 1.0;
        }
    }
    
    // Set params that are constant for one trust region iteration (ref: J. Schulman)
    for (int iter1 = 0; iter1 <= n_vars-1; iter1++){
        params.oldX[iter1] = 1.0;
    }
    
    // Set params that vary in one trust region iteration (ref: J. Schulman)
    params.s_tf[0] = 2.0;
    for (int iter1 = 0; iter1 <= n_trust-1; iter1++){
        params.s_th[iter1] = M_PI/4;
    }
    
    // Call Solver
    set_defaults();
    setup_indexing();
    settings.verbose = 1;
    solve();

    return 0;
}