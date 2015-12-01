/* 
 * File:   superState.h
 * Author: ross
 *
 * Created on January 23, 2014, 9:30 PM
 */

#ifndef SUPERSTATE_H
#define	SUPERSTATE_H

#include <vector>
#include "BVP_info.h"

struct superState{
    std::vector <double>* aSampledState;
    std::vector <BVP_info*> allBVPs;
};



#endif	/* SUPERSTATE_H */

