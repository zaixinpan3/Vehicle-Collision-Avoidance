// Independent outward-rounded verifier of the authoritative world Fiala equations.
// MPFR evaluates every elementary operation with directed rounding. Constants
// received as doubles define exact binary parameters; no sampled residual fit.
#include "mex.h"
#include <mpfr.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
using Unary = int (*)(mpfr_ptr,mpfr_srcptr,mpfr_rnd_t);
using Binary = int (*)(mpfr_ptr,mpfr_srcptr,mpfr_srcptr,mpfr_rnd_t);
struct I {
    double lo,hi;
    I(double v=0):I(v,v){}
    I(double l,double h):lo(l),hi(h){
        if(!std::isfinite(l)||!std::isfinite(h)||l>h)
            throw std::runtime_error("Interval arithmetic overflowed or produced invalid bounds.");
    }
};
double rounded(Binary f,double a,double b,mpfr_rnd_t mode) {
    mpfr_t x,y,z;mpfr_inits2(128,x,y,z,(mpfr_ptr)0);
    mpfr_set_d(x,a,MPFR_RNDN);mpfr_set_d(y,b,MPFR_RNDN);f(z,x,y,mode);
    double v=mpfr_get_d(z,mode);mpfr_clears(x,y,z,(mpfr_ptr)0);return v;
}
double rounded(Unary f,double a,mpfr_rnd_t mode) {
    mpfr_t x,z;mpfr_inits2(128,x,z,(mpfr_ptr)0);mpfr_set_d(x,a,MPFR_RNDN);f(z,x,mode);
    double v=mpfr_get_d(z,mode);mpfr_clears(x,z,(mpfr_ptr)0);return v;
}
I operator+(I a,I b){return {rounded(mpfr_add,a.lo,b.lo,MPFR_RNDD),rounded(mpfr_add,a.hi,b.hi,MPFR_RNDU)};}
I operator-(I a){return {-a.hi,-a.lo};}
I operator-(I a,I b){return a+(-b);}
I operator*(I a,I b){
    double low=INFINITY,high=-INFINITY;
    for(double x:{a.lo,a.hi})for(double y:{b.lo,b.hi}){
        low=std::min(low,rounded(mpfr_mul,x,y,MPFR_RNDD));
        high=std::max(high,rounded(mpfr_mul,x,y,MPFR_RNDU));
    }return {low,high};
}
I operator/(I a,I b){
    if(b.lo<=0 && b.hi>=0)throw std::runtime_error("Interval division includes zero.");
    I inverse(rounded(mpfr_div,1,b.hi,MPFR_RNDD),rounded(mpfr_div,1,b.lo,MPFR_RNDU));return a*inverse;
}
I hull(I a,I b){return {std::min(a.lo,b.lo),std::max(a.hi,b.hi)};}
I square(I a){
    double l=std::min(std::abs(a.lo),std::abs(a.hi)),h=std::max(std::abs(a.lo),std::abs(a.hi));
    if(a.lo<=0 && a.hi>=0)l=0;
    return {rounded(mpfr_mul,l,l,MPFR_RNDD),rounded(mpfr_mul,h,h,MPFR_RNDU)};
}
I absolute(I a){return {a.lo<=0 && a.hi>=0?0:std::min(std::abs(a.lo),std::abs(a.hi)),std::max(std::abs(a.lo),std::abs(a.hi))};}
I monotone(I a,Unary f){return {rounded(f,a.lo,MPFR_RNDD),rounded(f,a.hi,MPFR_RNDU)};}
I sine(I a){
    // Deliberately local world chart: all angles below are restricted to +/-1.5 rad.
    if(a.lo < -1.5 || a.hi > 1.5)throw std::runtime_error("Angle enclosure exceeds the certified local chart.");
    return monotone(a,mpfr_sin);
}
I cosine(I a){
    if(a.lo < -1.5 || a.hi > 1.5)throw std::runtime_error("Angle enclosure exceeds the certified local chart.");
    double l=std::max(std::abs(a.lo),std::abs(a.hi));
    double h=a.lo<=0 && a.hi>=0?0:std::min(std::abs(a.lo),std::abs(a.hi));
    return {rounded(mpfr_cos,l,MPFR_RNDD),rounded(mpfr_cos,h,MPFR_RNDU)};
}
struct D { I v;std::array<I,8> d{};D(double x=0):v(x){} D(I x):v(x){} };
D operator+(D a,D b){D c(a.v+b.v);for(int j=0;j<8;++j)c.d[j]=a.d[j]+b.d[j];return c;}
D operator-(D a){D c(-a.v);for(int j=0;j<8;++j)c.d[j]=-a.d[j];return c;}
D operator-(D a,D b){return a+(-b);}
D operator*(D a,D b){D c(a.v*b.v);for(int j=0;j<8;++j)c.d[j]=a.d[j]*b.v+a.v*b.d[j];return c;}
D operator/(D a,D b){D c(a.v/b.v);for(int j=0;j<8;++j)c.d[j]=(a.d[j]-c.v*b.d[j])/b.v;return c;}
D chain(D a,I value,I derivative){D c(value);for(int j=0;j<8;++j)c.d[j]=derivative*a.d[j];return c;}
D sq(D a){return chain(a,square(a.v),I(2)*a.v);}
D sinD(D a){return chain(a,sine(a.v),cosine(a.v));}
D cosD(D a){return chain(a,cosine(a.v),-sine(a.v));}
D atanD(D a){return chain(a,monotone(a.v,mpfr_atan),I(1)/(I(1)+square(a.v)));}
D tanD(D a){
    if(a.v.lo < -1.5 || a.v.hi > 1.5)throw std::runtime_error("Actual slip enclosure exceeds +/-1.5 rad.");
    I t=monotone(a.v,mpfr_tan);return chain(a,t,I(1)+square(t));
}
D tanhD(D a){I t=monotone(a.v,mpfr_tanh);return chain(a,t,I(1)-square(t));}
D sqrtD(D a){
    if(a.v.lo<=0)throw std::runtime_error("Square-root domain must be strictly positive.");
    I v=monotone(a.v,mpfr_sqrt);return chain(a,v,I(1)/(I(2)*v));
}
D absD(D a){I derivative=a.v.lo>0?I(1):(a.v.hi<0?I(-1):I(-1,1));return chain(a,absolute(a.v),derivative);}
D hull(D a,D b){D c(hull(a.v,b.v));for(int j=0;j<8;++j)c.d[j]=hull(a.d[j],b.d[j]);return c;}
D tire(D slip,D beta,double stiffness,double capacity){
    D t=tanD(slip),q=D(capacity)*sqrtD(D(1)-sq(beta));
    I threshold=(D(3)*q/D(stiffness)).v;I magnitude=absolute(t.v);
    D result;bool initialized=false;
    auto add=[&](D value){result=initialized?hull(result,value):value;initialized=true;};
    if(magnitude.lo<=threshold.hi){
        D ratio=D(stiffness)*absD(t)/(D(3)*q);
        add(-D(stiffness)*t*(D(1)-ratio+sq(ratio)/D(3)));
    }
    if(t.v.hi>=threshold.lo)add(-q);
    if(-t.v.lo>=threshold.lo)add(q);
    if(!initialized)throw std::runtime_error("No tire branch enclosed.");
    return result;
}
using State = std::array<D,8>;
using Flow = std::array<D,6>;
// p: m, Iz, lf, lr, C_f, C_r, capacity_f, capacity_r, rho_air, Cd,
// area, rolling0, rolling1, rolling4, rollingTransition, gravity, speedFloor.
Flow flow(const State& y,const double* p){
    D vx=y[3],vy=y[4],r=y[5],delta=y[6],beta=y[7];
    if(vx.v.lo<=p[16] || beta.v.lo<=-1 || beta.v.hi>=1)
        throw std::runtime_error("Domain requires vx above speed floor and abs(beta)<1.");
    D frontSlip=atanD((vy+D(p[2])*r)/vx)-delta;
    D rearSlip=atanD((vy-D(p[3])*r)/vx);
    D front=tire(frontSlip,beta,p[4],p[6]),rear=tire(rearSlip,beta,p[5],p[7]);
    D frontX=D(p[6])*beta*cosD(delta)-front*sinD(delta);
    D frontY=D(p[6])*beta*sinD(delta)+front*cosD(delta);
    D road=D(.5)*D(p[8])*D(p[9])*D(p[10])*sq(vx)
        +D(p[0])*D(p[15])*(D(p[11])+D(p[12])*vx+D(p[13])*sq(sq(vx)))*tanhD(vx/D(p[14]));
    return {vx*cosD(y[2])-vy*sinD(y[2]),vx*sinD(y[2])+vy*cosD(y[2]),r,
        (frontX+D(p[7])*beta-road)/D(p[0])+vy*r,
        (frontY+rear)/D(p[0])-vx*r,(D(p[2])*frontY-D(p[3])*rear)/D(p[1])};
}
const double* vector(const mxArray* a,mwSize count){
    if(!mxIsDouble(a)||mxIsComplex(a)||mxIsSparse(a)||mxGetNumberOfElements(a)!=count)
        throw std::runtime_error("Verifier arguments must be full real doubles with specified dimensions.");
    const double* x=mxGetDoubles(a);for(mwSize i=0;i<count;++i)if(!std::isfinite(x[i]))throw std::runtime_error("Arguments must be finite.");return x;
}
mxArray* intervals(const Flow& f){mxArray* a=mxCreateDoubleMatrix(6,2,mxREAL);double* out=mxGetDoubles(a);for(int i=0;i<6;++i){out[i]=f[i].v.lo;out[i+6]=f[i].v.hi;}return a;}
void mexFunction(int nlhs,mxArray* out[],int nrhs,const mxArray* in[]){
    try {
        if(!((nrhs==5 && nlhs==3)||(nrhs==11 && nlhs==4)))throw std::runtime_error("Expected five model arguments, optionally six held-feedback arguments.");
        const double* lower=vector(in[0],8);const double* upper=vector(in[1],8);
        const double* linear=vector(in[2],48);const double* offset=vector(in[3],6);const double* p=vector(in[4],17);
        if(mxGetM(in[2])!=6 || mxGetN(in[2])!=8)throw std::runtime_error("Linear map must be 6 by 8.");
        for(int j:{0,1,2,3,4,5,6,7,14,15,16})if(p[j]<=0)throw std::runtime_error("Physical parameters must be positive.");
        for(int j:{8,9,10,11,12,13})if(p[j]<0)throw std::runtime_error("Road-load parameters must be nonnegative.");
        State domain,center;std::array<I,8> displacement;
        for(int j=0;j<8;++j){
            if(lower[j]>upper[j])throw std::runtime_error("Domain bounds are reversed.");
            double m=std::clamp(lower[j]/2+upper[j]/2,lower[j],upper[j]);
            domain[j]=D(I(lower[j],upper[j]));domain[j].d[j]=I(1);center[j]=D(m);
            displacement[j]=domain[j].v-I(m);
        }
        Flow enclosure=flow(domain,p),middle=flow(center,p),residual;
        for(int i=0;i<6;++i){
            I value=middle[i].v-I(offset[i]);
            for(int j=0;j<8;++j)value=value-I(linear[i+6*j])*center[j].v;
            for(int j=0;j<8;++j)value=value+(enclosure[i].d[j]-I(linear[i+6*j]))*displacement[j];
            if(!std::isfinite(value.lo)||!std::isfinite(value.hi))throw std::runtime_error("Residual enclosure overflowed.");
            residual[i]=D(value);
        }
        out[0]=intervals(residual);out[1]=intervals(enclosure);
        mwSize dimensions[3]={6,8,2};out[2]=mxCreateNumericArray(3,dimensions,mxDOUBLE_CLASS,mxREAL);double* jac=mxGetDoubles(out[2]);
        for(int i=0;i<6;++i)for(int j=0;j<8;++j){jac[i+6*j]=enclosure[i].d[j].lo;jac[i+6*j+48]=enclosure[i].d[j].hi;}
        if(nrhs==11){
            const double* inletLower=vector(in[5],6);const double* inletUpper=vector(in[6],6);
            const double* nominal=vector(in[7],8);const double* gain=vector(in[8],12);
            const double* noise=vector(in[9],6);double h=vector(in[10],1)[0];
            if(h<=0 || mxGetM(in[8])!=2 || mxGetN(in[8])!=6)throw std::runtime_error("Positive duration and 2 by 6 gain required.");
            State held=domain;bool accepted=true;
            for(int j=0;j<6;++j){
                if(inletLower[j]>inletUpper[j] || noise[j]<0)throw std::runtime_error("Invalid inlet or measurement bounds.");
                accepted=accepted && inletLower[j]>=lower[j] && inletUpper[j]<=upper[j];
            }
            for(int u=0;u<2;++u){
                I command(nominal[6+u]);
                for(int j=0;j<6;++j)command=command+I(gain[u+2*j])*(I(inletLower[j],inletUpper[j])-I(nominal[j])+I(-noise[j],noise[j]));
                held[6+u]=D(command);accepted=accepted && command.lo>=lower[6+u] && command.hi<=upper[6+u];
            }
            // The affine residual above covers the full declared domain.
            // This independent first-exit test validates its use for the
            // entire held-feedback cell, without assuming tube containment.
            Flow heldFlow=flow(held,p),swept,endpoint;
            for(int j=0;j<6;++j){
                I initial(inletLower[j],inletUpper[j]);
                swept[j]=D(initial+I(0,h)*heldFlow[j].v);
                endpoint[j]=D(initial+I(h)*heldFlow[j].v);
                accepted=accepted && swept[j].v.lo>lower[j] && swept[j].v.hi<upper[j];
            }
            const char* names[]={"accepted","swept","endpoint","heldInput"};
            out[3]=mxCreateStructMatrix(1,1,4,names);
            mxSetField(out[3],0,"accepted",mxCreateLogicalScalar(accepted));
            mxSetField(out[3],0,"swept",intervals(swept));mxSetField(out[3],0,"endpoint",intervals(endpoint));
            mxArray* commands=mxCreateDoubleMatrix(2,2,mxREAL);double* uv=mxGetDoubles(commands);
            for(int j=0;j<2;++j){uv[j]=held[6+j].v.lo;uv[j+2]=held[6+j].v.hi;}
            mxSetField(out[3],0,"heldInput",commands);
        }
    }catch(const std::exception& e){mexErrMsgIdAndTxt("collisionAvoidanceController:invalidFialaInterval", "%s",e.what());}
}
