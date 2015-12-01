/* 
 * File:   FlatMatrix.h
 * Author: Ross Allen
 *
 * Created on January 17, 2014, 12:31 PM
 * 
 * FlatMatrix holds a 1D vector of values that can be transformed into a
 * multidimensional matrix using the dimensions variable. the indexing scheme
 * is assumed to take the form:
 * expanded( i1, i2, ..., in) = Flat(i1 + d1*(i2 +...+dn_2*(in_1+dn_1*in)...))
 */

#ifndef FLATMATRIX_H
#define	FLATMATRIX_H

#include <vector>


class FlatMatrix{
public:
    FlatMatrix();
    FlatMatrix(std::vector <int>, std::vector <double>);
    std::vector <int> getDimensions();
    std::vector <double> getValues();
    void setDimensions(std::vector <int>);
    void setValues(std::vector <double>);
    double value_at(std::vector <int>);
private:
    std::vector <int> dimensions;
    std::vector <double> values;
};

#endif	/* FLATMATRIX_H */

