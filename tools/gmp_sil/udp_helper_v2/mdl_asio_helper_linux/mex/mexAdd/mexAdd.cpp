/************************************************************************
 Sample MEX code written by Fang Liu (leoliuf@gmail.com).
************************************************************************/

/* system header */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* MEX header */
#include <mex.h> 
#include "matrix.h"

/* MEX entry function */
void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[])

{
    double *A, *B, *C;
    mwSignedIndex Am, An, Bm, Bn; 
    
    /* argument check */
    if ( nrhs != 2) {
        mexErrMsgIdAndTxt("MATLAB:cudaAdd:inputmismatch",
                          "Input arguments must be 2!");
    }
    if ( nlhs != 1) {
        mexErrMsgIdAndTxt("MATLAB:cudaAdd:outputmismatch",
                          "Output arguments must be 1!");
    }

    A = mxGetPr(prhs[0]); 
    B = mxGetPr(prhs[1]);

    /* matrix size */
    Am = (mwSignedIndex)mxGetM(prhs[0]);
    An = (mwSignedIndex)mxGetN(prhs[0]);    
    Bm = (mwSignedIndex)mxGetM(prhs[1]);
    Bn = (mwSignedIndex)mxGetN(prhs[1]);
    if ( Am != Bm || An != Bn) {
        mexErrMsgIdAndTxt("MATLAB:cudaAdd:sizemismatch",
                          "Input matrices must have the same size!");
    }

    /* allocate output */
    plhs[0] = mxCreateDoubleMatrix(Am, An, mxREAL);
    C = mxGetPr(plhs[0]);

    /* do loop calculation */
    for (int i=0; i<Am; i++){
        for (int j=0; j<An; j++){
            C[i*An+j]=A[i*An+j]+B[i*An+j];
        }
    }

}
