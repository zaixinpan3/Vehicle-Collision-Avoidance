#ifndef NATIVE_CONTROLLER_BENCHMARK_BRIDGE_H
#define NATIVE_CONTROLLER_BENCHMARK_BRIDGE_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
double benchmark_clock(void);
double benchmark_solve(int32_t n, int32_t m,
    int32_t pn, const double *pr, const double *pc, const double *pv,
    int32_t an, const double *ar, const double *ac, const double *av,
    const double *q, const double *b, int32_t nc, const double *sizes,
    const double *options, double *x);
#ifdef __cplusplus
}
#endif
#endif
