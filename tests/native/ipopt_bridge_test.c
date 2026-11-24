/* Exercise both PCBF NLP transcriptions and the analytic-Jacobian interface. */
#include "pcbf_ipopt.h"
#include <math.h>
#include <stdio.h>

/* An affine dual gap gives an exact one-dimensional PCBF test problem.
   The candidate must reach x >= 0.7 (terminal) and x >= 0.8 (zero slack). */
typedef struct { int merit, evaluated, differentiated; } fixture;
static int values(void *context,const double *x,double *v)
{
    fixture *f=context;f->evaluated++;
    double stage=.8-x[0],terminal=.7-x[0];
    if (f->merit) v[0]=pow(fmax(0,stage),2)+pow(fmax(0,terminal),2);
    else { v[0]=stage;v[1]=-1;v[2]=terminal; }
    return 0;
}
static int jacobian(void *context,const double *x,double *g)
{
    fixture *f=context;f->differentiated++;
    if (f->merit) g[0]=-2*(fmax(0,.8-x[0])+fmax(0,.7-x[0]));
    else { g[0]=-1;g[1]=0;g[2]=-1; }
    return 0;
}
int main(void)
{
    for (int mode=0;mode<3;mode++) {
        fixture f={mode==2,0,0};
        double start=0,lo=-1,hi=mode==1 ? .75 : 1,a=1,b=1,x=NAN;
        char status[1024];
        pcbf_ipopt_problem p={.knots=1,.stages=1,.members=1,.input_rows=1,.seek_zero=f.merit,
            .max_iterations=100,.prior_cap=mode==1 ? .06 : 0,.clearance=.1,.tolerance=1e-7,
            .wall_seconds=INFINITY,.mu_init=1e-6,.start=&start,.lower=&lo,.upper=&hi,
            .input_matrix=&a,.input_upper=&b,.context=&f,.values=values,.jacobian=jacobian};
        int rc=pcbf_ipopt_trial(&p,&x,status,sizeof(status));
        printf("mode %d, %s, x %.9g, evaluations %d, Jacobians %d\n",mode,status,x,f.evaluated,f.differentiated);
        if (rc || !isfinite(x) || x<.7 || .8-x>p.prior_cap+1e-5 || !f.differentiated) return 1;
    }
    return 0;
}
