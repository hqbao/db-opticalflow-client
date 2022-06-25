// This is a wrapper for Ce Liu's Coarse2Fine optical flow implementation.
// It converts the contiguous image array to the format needed by the optical
// flow code. Handling conversion in the wrapper makes the cythonization
// simpler.
// Author: Deepak Pathak (c) 2016

#ifndef Coarse2FineFlowWrapper_h
#define Coarse2FineFlowWrapper_h

#ifdef __cplusplus
extern "C" {
#endif

// override-include-guard
void Coarse2FineFlowWrapper(double * vx, double * vy, double * warpI2,
                              const double * Im1, const double * Im2,
                              double alpha, double ratio, int minWidth,
                              int nOuterFPIterations, int nInnerFPIterations,
                              int nSORIterations, int colType,
                              int h, int w, int c);

#ifdef __cplusplus
}
#endif

#endif /* Coarse2FineFlowWrapper_h */
