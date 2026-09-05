#include "mex.h"
#include <algorithm>
#include <cmath>
#include <vector>

// Exact corner enumeration for a piecewise affine Frenet chart, batched
// across prediction nodes. Binary searches retain both sides of vertices.
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 7 || nlhs != 1) {
        mexErrMsgIdAndTxt("laneFrameBoundsMex:arguments", "Expected seven inputs and one output.");
    }
    for (int k = 0; k < nrhs; ++k) {
        if (!mxIsDouble(prhs[k]) || mxIsComplex(prhs[k]) || mxIsSparse(prhs[k])) {
            mexErrMsgIdAndTxt("laneFrameBoundsMex:type", "Inputs must be full real doubles.");
        }
        const double* values = mxGetDoubles(prhs[k]);
        for (mwSize j = 0; j < mxGetNumberOfElements(prhs[k]); ++j) {
            if (!std::isfinite(values[j])) {
                mexErrMsgIdAndTxt("laneFrameBoundsMex:finite", "Inputs must be finite.");
            }
        }
    }
    const mwSize segments = mxGetM(prhs[3]);
    if (segments == 0 || mxGetN(prhs[3]) != 2 || mxGetM(prhs[6]) != segments
            || mxGetN(prhs[6]) != 2 || mxGetNumberOfElements(prhs[4]) != segments
            || mxGetNumberOfElements(prhs[5]) != segments
            || mxGetNumberOfElements(prhs[1]) != 1 || mxGetNumberOfElements(prhs[2]) != 1) {
        mexErrMsgIdAndTxt("laneFrameBoundsMex:dimensions", "Inconsistent frame or segment dimensions.");
    }
    const double* queries = mxGetDoubles(prhs[0]);
    const double radius = mxGetScalar(prhs[1]);
    const double lateralRadius = mxGetScalar(prhs[2]);
    const double* starts = mxGetDoubles(prhs[3]);
    const double* lengths = mxGetDoubles(prhs[4]);
    const double* stations = mxGetDoubles(prhs[5]);
    const double* tangents = mxGetDoubles(prhs[6]);
    if (radius < 0 || lateralRadius < 0) {
        mexErrMsgIdAndTxt("laneFrameBoundsMex:radius", "Frame radii must be nonnegative.");
    }
    for (mwSize j = 0; j < segments; ++j) {
        if (lengths[j] <= 0 || (j > 0 && (stations[j] <= stations[j-1]
                || stations[j]+lengths[j] < stations[j-1]+lengths[j-1]))) {
            mexErrMsgIdAndTxt("laneFrameBoundsMex:order", "Lengths must be positive and stations increasing.");
        }
    }
    const mwSize count = mxGetNumberOfElements(prhs[0]);
    plhs[0] = mxCreateDoubleMatrix(11, count, mxREAL);
    double* result = mxGetDoubles(plhs[0]);
    std::vector<double> ends(segments);
    for (mwSize j = 0; j < segments; ++j) ends[j] = stations[j]+lengths[j];
    for (mwSize point = 0; point < count; ++point) {
        auto after = std::upper_bound(stations, stations+segments, queries[point]);
        const mwSize selected = after == stations ? 0 : after-stations-1;
        const double tx = tangents[selected];
        const double ty = tangents[selected+segments];
        const double ox = starts[selected]-tx*stations[selected];
        const double oy = starts[selected+segments]-ty*stations[selected];
        const double heading = std::atan2(ty, tx);
        const double lower = std::max(stations[0], queries[point]-radius);
        const double upper = std::min(ends.back(), queries[point]+radius);
        double errorX = 0, errorY = 0, errorHeading = 0;
        const mwSize first = std::lower_bound(ends.begin(), ends.end(), lower)-ends.begin();
        const mwSize last = std::upper_bound(stations, stations+segments, upper)-stations;
        for (mwSize j = first; j < last; ++j) {
            const double low = std::max(lower, stations[j]);
            const double high = std::min(upper, ends[j]);
            const double ex = (ty-tangents[j+segments])*lateralRadius;
            const double ey = (tangents[j]-tx)*lateralRadius;
            for (double station : {low, high}) {
                const double dx = (starts[j]-ox)+tangents[j]*(station-stations[j])-tx*station;
                const double dy = (starts[j+segments]-oy)+tangents[j+segments]*(station-stations[j])-ty*station;
                errorX = std::max(errorX, std::max(std::abs(dx+ex), std::abs(dx-ex)));
                errorY = std::max(errorY, std::max(std::abs(dy+ey), std::abs(dy-ey)));
            }
            const double difference = std::atan2(tangents[j+segments], tangents[j])-heading;
            errorHeading = std::max(errorHeading, std::abs(std::atan2(std::sin(difference), std::cos(difference))));
        }
        const double values[] = {ox, oy, tx, ty, heading, static_cast<double>(selected+1),
            lower, upper, errorX, errorY, errorHeading};
        std::copy(values, values+11, result+11*point);
    }
}
