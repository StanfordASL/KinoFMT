#include "FlatMatrix.h"

using namespace std;

FlatMatrix::FlatMatrix(){
}

FlatMatrix::FlatMatrix(vector<int> dims, vector<double> vals){
    dimensions = dims;
    values = vals;
}

vector <int> FlatMatrix::getDimensions(){
    return dimensions;
}

vector <double> FlatMatrix::getValues(){
    return values;
}

void FlatMatrix::setDimensions(vector<int> nuDims){
    dimensions = nuDims;
}

void FlatMatrix::setValues(vector<double> nuVals){
    values = nuVals;
}

double FlatMatrix::value_at(vector<int> indices){
    int flatind = indices.at(indices.size()-1);  // index in flattened, 1D array
    for(int i = indices.size()-2; i >= 0; i--){
        flatind = indices.at(i) + dimensions.at(i)*flatind;
    }
    return values.at(flatind);
}