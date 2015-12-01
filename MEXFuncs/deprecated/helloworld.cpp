// helloworld for mex file
// see: www.shawnlankton.com/2008/03/getting-started-with-mex-a-short-tutorial/

#include<math.h>
#include<matrix.h>
#include<mex.h>

void mexFunction(int nlhs, mxArray *plhs[], int rhs, const mxArray *prhs[])
{

    /*
	// declare variables
	mxArray *a_in_m, *b_in_m, *c_out_m, *d_out_m;
	const mwSize *dims;
	double *a, *b, *c, *d;
	int dimx, dimy, numdims;

	
	// associate inputs
	a_in_m = mxDuplicateArray(prhs[0]);
	b_in_m = mxDuplicateArray(prhs[1]);

	// figure out dimensions
	dims = mxGetDimensions(prhs[0]);
	numdims = mxGetNumberOfDimensions(prhs[0]);
	dimy = (int)dims[0]; dimx = (int)dims[1];

	// associate outputs
	c_out_m = plhs[0] = mxCreateDoubleMatrix(dimy,dimx,mxREAL);
	d_out_m = plhs[1] = mxCreateDoubleMatrix(dimy,dimx,mxREAL);
	
	a = mxGetPr(a_in_m);
	b = mxGetPr(b_in_m);
	c = mxGetPr(c_in_m);
	d = mxGetPr(d_in_m);
	
	mexPrintf("Hello World!\n");
}
