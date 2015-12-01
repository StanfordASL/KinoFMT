/* 
 * File:   compilePrecomputedData.h
 * Author: Ross Allen
 *
 * Created on January 23, 2014, 10:31 PM
 * 
 * Reads data in from set of text files and stores it in a set of vectors and 
 * structures
 */

#ifndef COMPILEPRECOMPUTEDDATA_H
#define	COMPILEPRECOMPUTEDDATA_H

#include <vector>
#include <string>

#include "BVP_info.h"
#include "superState.h"
#include "FlatMatrix.h"
#include "text2FlatMatrix.h"


std::vector <std::vector <double>*> getSampledStateID(std::string);

std::vector <std::vector <BVP_info> > getPrecomputedData(std::string, std::string);

#endif	/* COMPILEPRECOMPUTEDDATA_H */

