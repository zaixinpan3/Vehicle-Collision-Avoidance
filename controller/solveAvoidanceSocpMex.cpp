// MATLAB bridge to the pinned Clarabel C API. The solver owns its workspace
// only during this call; no controller instance or certificate is cached here.
#include "mex.h"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>
extern "C" {
#include "clarabel.h"
}

namespace {
void require(bool condition, const char* message) {
    if (!condition)
        mexErrMsgIdAndTxt("collisionAvoidanceController:invalidSocpData", "%s", message);
}

void finiteValues(const double* values, mwSize count) {
    for (mwSize index = 0; index < count; ++index)
        require(std::isfinite(values[index]), "SOCP data must be finite.");
}

ClarabelCscMatrix sparseMatrix(const mxArray* value) {
    require(mxIsSparse(value) && mxIsDouble(value) && !mxIsComplex(value),
        "SOCP matrices must be real double sparse arrays.");
    static_assert(sizeof(mwIndex) == sizeof(uintptr_t), "Sparse index widths must agree.");
    finiteValues(mxGetDoubles(value), mxGetJc(value)[mxGetN(value)]);
    return {mxGetM(value), mxGetN(value),
        reinterpret_cast<const uintptr_t*>(mxGetJc(value)),
        reinterpret_cast<const uintptr_t*>(mxGetIr(value)), mxGetDoubles(value)};
}

const double* vector(const mxArray* value, mwSize count) {
    require(mxIsDouble(value) && !mxIsSparse(value) && !mxIsComplex(value)
        && mxGetNumberOfElements(value) == count, "SOCP vector dimensions or types disagree.");
    const double* data = mxGetDoubles(value);
    finiteValues(data, count);
    return data;
}
}

void mexFunction(int outputs, mxArray* result[], int inputs, const mxArray* argument[]) {
    require(inputs == 6 && outputs == 2,
        "Use [decision, information] = solveAvoidanceSocpMex(P, q, A, b, cones, options).");
    const auto hessian = sparseMatrix(argument[0]);
    const auto matrix = sparseMatrix(argument[2]);
    require(hessian.m == hessian.n && hessian.n > 0 && matrix.n == hessian.n,
        "SOCP matrix dimensions disagree.");
    const auto linear = vector(argument[1], hessian.n);
    const auto bound = vector(argument[3], matrix.m);
    const mwSize coneCount = mxGetNumberOfElements(argument[4]);
    require(coneCount >= 2, "Cone sizes must specify zero and nonnegative cones first.");
    const auto dimensions = vector(argument[4], coneCount);
    const auto options = vector(argument[5], 3);
    require(options[0] > 0 && options[1] > 0 && options[2] >= 1
        && options[2] == std::floor(options[2])
        && options[2] <= std::numeric_limits<uint32_t>::max(), "Invalid solver tolerances or iteration limit.");

    // Validate before allocating C++ resources: mexErrMsg does not unwind them.
    double total = 0;
    for (mwSize index = 0; index < coneCount; ++index) {
        const double size = dimensions[index];
        require(size >= 0 && size == std::floor(size) && size <= matrix.m,
            "Cone dimensions must be nonnegative integers.");
        require(index < 2 || size >= 2, "A Lorentz cone must have at least two entries.");
        total += size;
    }
    require(total == matrix.m, "Cone sizes must sum to the constraint row count.");
    std::vector<ClarabelSupportedConeT> cones;
    cones.reserve(coneCount);
    for (mwSize index = 0; index < coneCount; ++index) {
        const auto size = static_cast<uintptr_t>(dimensions[index]);
        if (size == 0) continue;
        if (index == 0) cones.push_back(ClarabelZeroConeT(size));
        else if (index == 1) cones.push_back(ClarabelNonnegativeConeT(size));
        else cones.push_back(ClarabelSecondOrderConeT(size));
    }
    auto settings = clarabel_DefaultSettings_default();
    settings.verbose = false;
    settings.max_iter = static_cast<uint32_t>(options[2]);
    settings.tol_feas = options[0];
    settings.tol_infeas_abs = options[0];
    settings.tol_infeas_rel = options[0];
    settings.tol_gap_abs = options[1];
    settings.tol_gap_rel = options[1];
    settings.direct_solve_method = QDLDL;
    settings.max_threads = 1;
    auto solver = clarabel_DefaultSolver_new(&hessian, linear, &matrix, bound,
        cones.size(), cones.data(), &settings);
    clarabel_DefaultSolver_solve(solver);
    const auto solution = clarabel_DefaultSolver_solution(solver);
    result[0] = mxCreateDoubleMatrix(solution.x_length, 1, mxREAL);
    std::memcpy(mxGetDoubles(result[0]), solution.x, solution.x_length * sizeof(double));
    const char* names[] = {"status", "iterations", "solveTime", "primalResidual", "dualResidual", "objectiveValue"};
    result[1] = mxCreateStructMatrix(1, 1, 6, names);
    const double values[] = {static_cast<double>(solution.status), static_cast<double>(solution.iterations),
        solution.solve_time, solution.r_prim, solution.r_dual, solution.obj_val};
    for (int index = 0; index < 6; ++index)
        mxSetFieldByNumber(result[1], 0, index, mxCreateDoubleScalar(values[index]));
    clarabel_DefaultSolver_free(solver);
}
