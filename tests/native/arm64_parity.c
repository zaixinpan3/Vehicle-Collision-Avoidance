#include "nrmm_track.h"
#include "pcbf_kernels.h"
#include "pcbfNodeClearance.h"
#include "pcbfNodeClearance_emxAPI.h"
#include "pcbfTargetEnvelope.h"
#include "pcbfTerminalSector.h"
#include "pcbfTerminalKnown.h"
#include "pcbfFieldInputs.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static size_t comparisons;
static double worst;
static void check(double got, double expected)
{
    if (got==expected) { comparisons++;return; }
    double error = fabs(got - expected);
    if (!isfinite(error) || error > 1e-8 * (1 + fabs(expected))) {
        fprintf(stderr,"Mismatch at scalar %zu: %.17g vs %.17g\n", comparisons, got, expected);
        exit(1);
    }
    if (error > worst) worst = error;
    comparisons++;
}
static void read_values(FILE *file, double *values, size_t count)
{
    if (fread(values,sizeof(*values),count,file) != count) exit(2);
}
static void check_array(const double *got,const double *expected,size_t count)
{
    for (size_t j=0;j<count;j++) check(got[j],expected[j]);
}
static void observer(FILE *file)
{
    nrmm_track *track=nrmm_track_create();
    if (!track) exit(2);
    for (int k=0;k<50;k++) {
        double v[32];
        read_values(file,v,32);
        nrmm_estimate est;
        if (nrmm_track_update(track,v[0],v[6] ? &v[1] : NULL,v[3],v[4],v[5],&est)) exit(2);
        check_array(est.state,v+7,6);
        check_array(est.motion,v+13,4);
        check_array(est.position_bounds,v+17,4);
        check_array(est.motion_bounds,v+21,4);
        check_array(est.component_radii,v+25,3);
        check(est.initialized,v[28]);check(est.motion_available,v[29]);
        check(est.bounds_available,v[30]);check(est.rest_compatible,v[31]);
    }
    nrmm_track_destroy(track);
}
typedef int (*kernel)(const double **,double **,casadi_int *,double *,int);
typedef int (*work)(casadi_int *,casadi_int *,casadi_int *,casadi_int *);
static int length(FILE *file)
{
    int32_t n;
    if (fread(&n,sizeof(n),1,file)!=1 || n<1 || n>10000000) exit(2);
    return n;
}
static void graph(FILE *file,kernel evaluate,work sizes,int inputs,int outputs)
{
    casadi_int a,b,i,w;
    sizes(&a,&b,&i,&w);
    const double **args=calloc(a,sizeof(*args));
    double **results=calloc(b,sizeof(*results)),*expected[2];
    double *values[4];int n[2];
    for (int j=0;j<inputs;j++) {
        int count=length(file);values[j]=calloc(count,sizeof(double));
        read_values(file,values[j],count);args[j]=values[j];
    }
    for (int j=0;j<outputs;j++) {
        n[j]=length(file);expected[j]=calloc(n[j],sizeof(double));results[j]=calloc(n[j],sizeof(double));
        read_values(file,expected[j],n[j]);
    }
    double *workspace=calloc(w,sizeof(double));
    casadi_int *indices=calloc(i,sizeof(*indices));
    if (evaluate(args,results,indices,workspace,0)) exit(2);
    for (int j=0;j<outputs;j++) {check_array(results[j],expected[j],n[j]);free(results[j]);free(expected[j]);}
    for (int j=0;j<inputs;j++) free(values[j]);
    free(args);free(results);free(workspace);free(indices);
}
static void geometry(FILE *file,double hold)
{
    double x[6],q[11],shape[4],expected[89];
    read_values(file,x,6);read_values(file,q,11);read_values(file,shape,4);read_values(file,expected,89);
    double gap,grad[3],v,n[2],d[6],steps,speeds[2]={0,x[3]};
    pcbfNodeClearance(x,q,shape,&gap,grad);
    check(gap,expected[0]);check_array(grad,expected+1,3);
    pcbfTerminalSector(x,q,speeds,.3,.15,.03,&v,n,d,&steps);
    check(v,expected[4]);check_array(n,expected+5,2);check_array(d,expected+7,6);check(steps,expected[13]);
    double t[]={0,.3,1,3,10},limits[]={-2.1,-1.9,-.03,-.01};
    emxArray_real_T *times=emxCreateWrapper_real_T(t,1,5),*boxes=emxCreate_real_T(0,0),*centers=emxCreate_real_T(0,0);
    pcbfTargetEnvelope(q,limits,times,boxes,centers);
    if (boxes->size[0]!=11 || boxes->size[1]!=5 || centers->size[0]!=4 || centers->size[1]!=5) exit(2);
    check_array(boxes->data,expected+14,55);check_array(centers->data,expected+69,20);
    double known[10];read_values(file,known,10);
    pcbfTerminalKnown(x,q,0,&v,n,d,&steps);
    check(v,known[0]);check_array(n,known+1,2);check_array(d,known+3,6);check(steps,known[9]);
    double prior[2]={0,0}, times_data[13],commands[24];
    struct7_T curve={ .origin={-100,0},.heading=0,.curvature=.001,.length=1000 };
    emxDestroyArray_real_T(times);
    for (int k=0;k<13;k++) times_data[k]=k*hold;
    times=emxCreateWrapper_real_T(times_data,1,13);
    emxArray_real_T *inputs=emxCreate_real_T(0,0);
    for (int covered=0;covered<2;covered++) {
        if (covered) pcbfTargetEnvelope(q,limits,times,boxes,centers);
        else { boxes->size[1]=0;centers->size[1]=0; }
        pcbfFieldInputs(x,prior,q,&curve,boxes,centers,12,inputs);
        if (inputs->size[0]!=2 || inputs->size[1]!=12) exit(2);
        read_values(file,commands,24);check_array(inputs->data,commands,24);
    }
    emxDestroyArray_real_T(inputs);
    emxDestroyArray_real_T(times);emxDestroyArray_real_T(boxes);emxDestroyArray_real_T(centers);
}
int main(int argc,char **argv)
{
    if (argc!=2) return 2;
    FILE *file=fopen(argv[1],"rb");if (!file) return 2;
    double hold;read_values(file,&hold,1);
    for (int mode=0;mode<3;mode++) observer(file);
    for (int sample=0;sample<3;sample++) {
        graph(file,pcbf_rollout,pcbf_rollout_work,4,2);
        graph(file,pcbf_sensitivity,pcbf_sensitivity_work,4,2);
        graph(file,pcbf_inference,pcbf_inference_work,3,1);
        graph(file,pcbf_hold,pcbf_hold_work,3,2);
    }
    for (int i=0;i<3;i++) geometry(file,hold);
    if (fgetc(file)!=EOF) return 2;
    fclose(file);
    printf("%zu scalar comparisons, max absolute difference %.12g\n",comparisons,worst);
    return 0;
}
