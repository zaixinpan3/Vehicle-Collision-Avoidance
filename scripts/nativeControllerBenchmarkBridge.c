/* Standalone benchmark glue: same pinned Clarabel settings as the MEX bridge. */
#include "nativeControllerBenchmarkBridge.h"
#include "clarabel.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

double benchmark_clock(void) {
    struct timespec stamp;
    clock_gettime(CLOCK_MONOTONIC, &stamp);
    return (double)stamp.tv_sec + 1e-9 * (double)stamp.tv_nsec;
}

/* MATLAB find emits column-sorted, one-based double triplets. */
static ClarabelCscMatrix matrix(int32_t m, int32_t n, int32_t nz,
    const double *rows, const double *columns, const double *values) {
    uintptr_t *col = calloc((size_t)n + 1, sizeof(uintptr_t));
    uintptr_t *row = malloc(((size_t)nz + 1) * sizeof(uintptr_t));
    if (!col || !row) abort();
    for (int32_t k = 0; k < nz; ++k) {
        ++col[(uintptr_t)columns[k]];
        row[k] = (uintptr_t)rows[k] - 1;
    }
    for (int32_t k = 0; k < n; ++k) col[k + 1] += col[k];
    ClarabelCscMatrix result = {(uintptr_t)m, (uintptr_t)n, col, row, values};
    return result;
}

double benchmark_solve(int32_t n, int32_t m,
    int32_t pn, const double *pr, const double *pc, const double *pv,
    int32_t an, const double *ar, const double *ac, const double *av,
    const double *q, const double *b, int32_t nc, const double *sizes,
    const double *options, double *x) {
    ClarabelCscMatrix p = matrix(n,n,pn,pr,pc,pv);
    ClarabelCscMatrix a = matrix(m,n,an,ar,ac,av);
    ClarabelSupportedConeT *cones = malloc((size_t)nc * sizeof(*cones));
    if (!cones) abort();
    uintptr_t count = 0;
    for (int32_t k = 0; k < nc; ++k) {
        uintptr_t size = (uintptr_t)sizes[k];
        if (!size) continue;
        cones[count++] = k == 0 ? ClarabelZeroConeT(size) :
            k == 1 ? ClarabelNonnegativeConeT(size) : ClarabelSecondOrderConeT(size);
    }
    ClarabelDefaultSettings settings = clarabel_DefaultSettings_default();
    settings.verbose = false; settings.max_iter = (uint32_t)options[2];
    settings.tol_feas = options[0]; settings.tol_infeas_abs = options[0];
    settings.tol_infeas_rel = options[0]; settings.tol_gap_abs = options[1];
    settings.tol_gap_rel = options[1]; settings.direct_solve_method = QDLDL;
    settings.max_threads = 1;
    ClarabelDefaultSolver *solver = clarabel_DefaultSolver_new(&p,q,&a,b,count,cones,&settings);
    clarabel_DefaultSolver_solve(solver);
    ClarabelDefaultSolution solution = clarabel_DefaultSolver_solution(solver);
    memcpy(x, solution.x, (size_t)n * sizeof(double));
    double status = (double)solution.status;
    clarabel_DefaultSolver_free(solver);
    free((void *)p.colptr); free((void *)p.rowval);
    free((void *)a.colptr); free((void *)a.rowval); free(cones);
    return status;
}
