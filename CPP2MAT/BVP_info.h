/* 
 * File:   BVP_info.h
 * Author: Ross Allen
 *
 * Created on January 23, 2014, 9:14 PM
 * 
 * BVP_info is a structure for holding the information about a single optimal 
 * control, 2-point boundary value problem. This includes the trajectory in the
 * form of a set of intermediate states and the cost for the trajectory
 */


#ifndef BVP_INFO_H
#define	BVP_INFO_H

#include <vector>

struct BVP_info {
    std::vector <std::vector <double>* > trajectory;
    double cost;
};
    
#endif	/* BVP_INFO_H */

