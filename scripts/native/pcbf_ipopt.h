#ifndef PCBF_IPOPT_H
#define PCBF_IPOPT_H
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

/* The oracle takes scaled control knots. Its analytic Jacobian uses column-major order.
   In the primary problem the output rows are stage deficits, model-domain rows and
   terminal residuals. In the zero-deficit search the oracle returns the scalar merit. */
typedef int (*pcbf_evaluate)(void *context, const double *knots, double *values);
typedef int (*pcbf_stop)(void *context, const double *knots);
typedef struct {
    int knots, stages, members, input_rows, seek_zero, max_iterations;
    double prior_cap, clearance, tolerance, wall_seconds, mu_init;
    const double *start, *lower, *upper, *input_matrix, *input_upper;
    void *context;
    pcbf_evaluate values, jacobian;
    pcbf_stop stop;
} pcbf_ipopt_problem;

/* Returns an IPOPT trial in scaled coordinates. Dispatch requires separate validation.
   0 means IPOPT reported success, 1 means an interrupted/unsuccessful call returned
   a trial, and -1 means the call failed. The optional message includes the solver status. */
int pcbf_ipopt_trial(const pcbf_ipopt_problem *problem,double *trial,char *message,size_t capacity);
#ifdef __cplusplus
}
#endif
#endif
