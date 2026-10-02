// Sparse conic quadratic program adapter. The controller owns the formulation.
// min 0.5*x'*P*x + q'*x, A*x+s=b, s in zero/nonnegative/second-order cones.
#include "mex.h"
extern "C" {
#include "clarabel.h"
}
#include <cmath>
#include <cstdint>
#include <cstring>

static void require(bool condition, const char* message) {
    if (!condition) mexErrMsgIdAndTxt("predictiveConicSolver:invalidData", "%s", message);
}

static void requireReal(const mxArray* value, bool sparse) {
    require(mxIsDouble(value) && !mxIsComplex(value) && mxIsSparse(value)==sparse,
        "Expected real double arrays, with sparse P and A only.");
    const size_t count = sparse ? mxGetJc(value)[mxGetN(value)] : mxGetNumberOfElements(value);
    const double* data = mxGetDoubles(value);
    for (size_t k=0; k<count; ++k) require(std::isfinite(data[k]), "Problem data must be finite.");
}

static ClarabelCscMatrix_f64 matrix(const mxArray* value) {
    static_assert(sizeof(mwIndex)==sizeof(uintptr_t), "Sparse index width mismatch.");
    return {mxGetM(value), mxGetN(value),
        reinterpret_cast<const uintptr_t*>(mxGetJc(value)),
        reinterpret_cast<const uintptr_t*>(mxGetIr(value)), mxGetDoubles(value)};
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    require(nrhs==7 && nlhs>=2 && nlhs<=3, "Use [x,flag,info] = predictiveConicSolverMex(P,q,A,b,dimensions,types,settings).");
    for (int k=0; k<7; ++k) requireReal(prhs[k], k==0 || k==2);
    const size_t n=mxGetN(prhs[0]), m=mxGetM(prhs[2]);
    require(mxGetM(prhs[0])==n && mxGetNumberOfElements(prhs[1])==n && mxGetN(prhs[2])==n
        && mxGetNumberOfElements(prhs[3])==m, "Incompatible matrix dimensions.");
    const size_t count=mxGetNumberOfElements(prhs[4]);
    require(count==mxGetNumberOfElements(prhs[5]) && mxGetNumberOfElements(prhs[6])==5,
        "Incompatible cone or settings dimensions.");
    const double* dimensions=mxGetDoubles(prhs[4]);
    const double* types=mxGetDoubles(prhs[5]);
    size_t total=0;
    for (size_t k=0; k<count; ++k) {
        require(dimensions[k]>=0 && dimensions[k]<=m && dimensions[k]==std::floor(dimensions[k])
            && types[k]>=0 && types[k]<=2 && types[k]==std::floor(types[k]), "Invalid cone descriptor.");
        require(types[k]!=2 || dimensions[k]>=2, "Second-order cones need at least two coordinates.");
        total+=static_cast<size_t>(dimensions[k]);
    }
    require(total==m, "Cone dimensions must sum to the constraint count.");
    const double* options=mxGetDoubles(prhs[6]);
    require(options[0]>=1 && options[0]<=UINT32_MAX && options[0]==std::floor(options[0])
        && options[1]>0 && options[2]>0 && options[3]>0 && options[4]>=options[2], "Invalid solver settings.");
    for (size_t j=0; j<n; ++j)
        for (size_t k=mxGetJc(prhs[0])[j]; k<mxGetJc(prhs[0])[j+1]; ++k)
            require(mxGetIr(prhs[0])[k]<=j, "P must store only its upper triangle.");
    auto* cones=static_cast<ClarabelSupportedConeT_f64*>(mxCalloc(count,sizeof(ClarabelSupportedConeT_f64)));
    for (size_t k=0; k<count; ++k) {
        cones[k].tag=static_cast<ClarabelSupportedConeT_Tag>(static_cast<int>(types[k]));
        if (types[k]==0) cones[k].zero_cone_t=static_cast<uintptr_t>(dimensions[k]);
        else if (types[k]==1) cones[k].nonnegative_cone_t=static_cast<uintptr_t>(dimensions[k]);
        else cones[k].second_order_cone_t=static_cast<uintptr_t>(dimensions[k]);
    }
    auto settings=clarabel_DefaultSettings_f64_default();
    settings.verbose=false; settings.max_threads=1;
    settings.max_iter=static_cast<uint32_t>(options[0]); settings.time_limit=options[1];
    settings.tol_feas=options[2]; settings.tol_gap_abs=options[3]; settings.tol_gap_rel=options[3];
    // Desired precision and declared numerical acceptance precision are
    // distinct. A stalled interior-point solve may satisfy the latter.
    settings.reduced_tol_feas=options[4];
    settings.reduced_tol_gap_abs=options[4];
    settings.reduced_tol_gap_rel=options[4];
    const auto P=matrix(prhs[0]), A=matrix(prhs[2]);
    auto* solver=clarabel_DefaultSolver_f64_new(&P,mxGetDoubles(prhs[1]),&A,mxGetDoubles(prhs[3]),count,cones,&settings);
    mxFree(cones);
    if (!solver) mexErrMsgIdAndTxt("predictiveConicSolver:constructionFailed", "Clarabel could not construct the problem.");
    clarabel_DefaultSolver_f64_solve(solver);
    const auto result=clarabel_DefaultSolver_f64_solution(solver);
    double flag=-7;
    if (result.status==ClarabelSolved) flag=1;
    else if (result.status==ClarabelAlmostSolved) flag=2;
    else if (result.status==ClarabelPrimalInfeasible || result.status==ClarabelAlmostPrimalInfeasible) flag=-2;
    else if (result.status==ClarabelDualInfeasible || result.status==ClarabelAlmostDualInfeasible) flag=-3;
    else if (result.status==ClarabelMaxIterations || result.status==ClarabelMaxTime) flag=0;
    // Infeasibility certificates and unfinished iterates are not solutions.
    plhs[0]=mxCreateDoubleMatrix(flag>0 ? n : 0,1,mxREAL);
    if (flag>0) std::memcpy(mxGetDoubles(plhs[0]),result.x,n*sizeof(double));
    plhs[1]=mxCreateDoubleScalar(flag);
    if (nlhs==3) {
        const char* names[]={"nativeStatus","iterations","solveSeconds","primalResidual","dualResidual","objective"};
        const double values[]={static_cast<double>(result.status),static_cast<double>(result.iterations),
            result.solve_time,result.r_prim,result.r_dual,result.obj_val};
        plhs[2]=mxCreateStructMatrix(1,1,6,names);
        for (int k=0; k<6; ++k) mxSetFieldByNumber(plhs[2],0,k,mxCreateDoubleScalar(values[k]));
    }
    clarabel_DefaultSolver_f64_free(solver);
}
