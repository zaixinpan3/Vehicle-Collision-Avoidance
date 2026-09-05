#include "mex.h"
#include <algorithm>
#include <cmath>
#include <limits>

// Batched exhaustive projection. Retain every segment and select the first
// minimizer, matching MATLAB min, including at vertices and path endpoints.
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 6 || nlhs != 1) {
        mexErrMsgIdAndTxt("projectLanePolylineMex:arguments", "Expected six inputs and one output.");
    }
    for (int k = 0; k < nrhs; ++k) {
        if (!mxIsDouble(prhs[k]) || mxIsComplex(prhs[k]) || mxIsSparse(prhs[k])) {
            mexErrMsgIdAndTxt("projectLanePolylineMex:type", "Inputs must be full real doubles.");
        }
        const double* values = mxGetDoubles(prhs[k]);
        for (mwSize j = 0; j < mxGetNumberOfElements(prhs[k]); ++j) {
            if (!std::isfinite(values[j])) {
                mexErrMsgIdAndTxt("projectLanePolylineMex:finite", "Inputs must be finite.");
            }
        }
    }
    const mwSize segments = mxGetM(prhs[1]);
    if (mxGetM(prhs[0]) != 2 || segments == 0 || mxGetN(prhs[1]) != 2
            || mxGetM(prhs[2]) != segments || mxGetN(prhs[2]) != 2
            || mxGetNumberOfElements(prhs[3]) != segments
            || mxGetNumberOfElements(prhs[4]) != segments
            || mxGetM(prhs[5]) != segments || mxGetN(prhs[5]) != 2) {
        mexErrMsgIdAndTxt("projectLanePolylineMex:dimensions", "Inconsistent point or segment dimensions.");
    }
    const double* positions = mxGetDoubles(prhs[0]);
    const double* starts = mxGetDoubles(prhs[1]);
    const double* directions = mxGetDoubles(prhs[2]);
    const double* lengths = mxGetDoubles(prhs[3]);
    const double* stations = mxGetDoubles(prhs[4]);
    const double* tangents = mxGetDoubles(prhs[5]);
    for (mwSize j = 0; j < segments; ++j) {
        if (lengths[j] <= 0) {
            mexErrMsgIdAndTxt("projectLanePolylineMex:length", "Segment lengths must be positive.");
        }
    }
    const mwSize count = mxGetN(prhs[0]);
    plhs[0] = mxCreateDoubleMatrix(5, count, mxREAL);
    double* result = mxGetDoubles(plhs[0]);
    for (mwSize point = 0; point < count; ++point) {
        const double x = positions[2*point];
        const double y = positions[2*point+1];
        double best = std::numeric_limits<double>::infinity();
        mwSize selected = 0;
        for (mwSize j = 0; j < segments; ++j) {
            const double fraction = std::min(1.0, std::max(0.0,
                ((x-starts[j])*directions[j] + (y-starts[j+segments])*directions[j+segments])
                / (lengths[j]*lengths[j])));
            const double px = starts[j] + fraction*directions[j];
            const double py = starts[j+segments] + fraction*directions[j+segments];
            const double dx = x-px;
            const double dy = y-py;
            const double distance = dx*dx + dy*dy;
            if (distance < best) {
                best = distance;
                selected = j;
                result[5*point] = px;
                result[5*point+1] = py;
                result[5*point+3] = stations[j] + fraction*lengths[j];
                result[5*point+4] = tangents[j]*dy - tangents[j+segments]*dx;
            }
        }
        result[5*point+2] = std::atan2(tangents[selected+segments], tangents[selected]);
    }
}
