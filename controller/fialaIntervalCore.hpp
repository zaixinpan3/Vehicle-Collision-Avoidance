#pragma once
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
// Reuse scratch limbs per native thread. Every operation still has explicit
// MPFR rounding; no host rounding-mode change or empirical epsilon is used.
struct Scratch {
    mpfr_t x,y,z;
    Scratch(){mpfr_inits2(128,x,y,z,(mpfr_ptr)0);}
    ~Scratch(){mpfr_clears(x,y,z,(mpfr_ptr)0);}
};
Scratch& scratch(){static thread_local Scratch value;return value;}
double rounded(Binary f,double a,double b,mpfr_rnd_t mode) {
    Scratch& s=scratch();mpfr_set_d(s.x,a,MPFR_RNDN);mpfr_set_d(s.y,b,MPFR_RNDN);
    f(s.z,s.x,s.y,mode);return mpfr_get_d(s.z,mode);
}
double rounded(Unary f,double a,mpfr_rnd_t mode) {
    Scratch& s=scratch();mpfr_set_d(s.x,a,MPFR_RNDN);f(s.z,s.x,mode);return mpfr_get_d(s.z,mode);
}
bool zero(I a){return a.lo==0 && a.hi==0;}
I operator+(I a,I b){
    if(zero(a))return b;
    if(zero(b))return a;
    return {rounded(mpfr_add,a.lo,b.lo,MPFR_RNDD),rounded(mpfr_add,a.hi,b.hi,MPFR_RNDU)};
}
I operator-(I a){return {-a.hi,-a.lo};}
I operator-(I a,I b){return a+(-b);}
I products(double a,double b,double c,double d){
    return {rounded(mpfr_mul,a,b,MPFR_RNDD),rounded(mpfr_mul,c,d,MPFR_RNDU)};
}
I operator*(I a,I b){
    if(zero(a)||zero(b))return I(0);
    if(a.lo==1 && a.hi==1)return b;
    if(b.lo==1 && b.hi==1)return a;
    // Sign monotonicity identifies the exact extremizing endpoints before
    // rounding; rounded products are never used to choose an endpoint pair.
    if(a.lo>=0){
        if(b.lo>=0)return products(a.lo,b.lo,a.hi,b.hi);
        if(b.hi<=0)return products(a.hi,b.lo,a.lo,b.hi);
        return products(a.hi,b.lo,a.hi,b.hi);
    }
    if(a.hi<=0){
        if(b.lo>=0)return products(a.lo,b.hi,a.hi,b.lo);
        if(b.hi<=0)return products(a.hi,b.hi,a.lo,b.lo);
        return products(a.lo,b.hi,a.lo,b.lo);
    }
    if(b.lo>=0)return products(a.lo,b.hi,a.hi,b.hi);
    if(b.hi<=0)return products(a.hi,b.lo,a.lo,b.lo);
    return {std::min(rounded(mpfr_mul,a.lo,b.hi,MPFR_RNDD),rounded(mpfr_mul,a.hi,b.lo,MPFR_RNDD)),
        std::max(rounded(mpfr_mul,a.lo,b.lo,MPFR_RNDU),rounded(mpfr_mul,a.hi,b.hi,MPFR_RNDU))};
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
I clippedRatio(I t,I capacity,double stiffness){
    I q=I(stiffness)*absolute(t)/(I(3)*capacity);
    return {std::min(1.0,q.lo),std::min(1.0,q.hi)};
}
I forceAt(double t,double capacity,double stiffness){
    I q=clippedRatio(I(t),I(capacity),stiffness);
    I value=I(capacity)*q*(I(3)-I(3)*q+square(q));
    // The monotone cubic lies in [0,1] throughout the clipped domain.
    value=I(std::max(0.0,value.lo),std::min(capacity,value.hi));
    return t>=0?-value:value;
}
I capacitySlopeAt(double q){return square(I(q))*(I(3)-I(2)*I(q));}
struct TireBounds {I value,slipSlope,capacitySlope,ratio;};
TireBounds tireBounds(I t,I available,double stiffness){
    I q=clippedRatio(t,available,stiffness);
    I slipSlope=-I(stiffness)*square(I(1)-q);
    I magnitudeSlope(std::max(0.0,capacitySlopeAt(q.lo).lo),
        std::min(1.0,capacitySlopeAt(q.hi).hi));
    I sign=t.lo>=0?I(1):(t.hi<=0?I(-1):I(-1,1));
    I capacitySlope=-sign*magnitudeSlope;
    I low=forceAt(t.hi,t.hi>=0?available.hi:available.lo,stiffness);
    I high=forceAt(t.lo,t.lo<=0?available.hi:available.lo,stiffness);
    return {I(low.lo,high.hi),slipSlope,capacitySlope,q};
}
D tire(D slip,D beta,double stiffness,double capacity){
    D t=tanD(slip),available=D(capacity)*sqrtD(D(1)-sq(beta));
    TireBounds b=tireBounds(t.v,available.v,stiffness);D result(b.value);
    for(int j=0;j<8;++j)result.d[j]=b.slipSlope*t.d[j]+b.capacitySlope*available.d[j];
    return result;
}
// One interval direction suffices to bound d' H(y) d. This avoids storing
// an 8-by-8 Hessian or assuming that a sampled Hessian bounds the domain.
struct J {I v,first,second;J(double value=0):v(value){} J(I value,I d=I(),I dd=I()):v(value),first(d),second(dd){} };
J operator+(J a,J b){return {a.v+b.v,a.first+b.first,a.second+b.second};}
J operator-(J a){return {-a.v,-a.first,-a.second};}
J operator-(J a,J b){return a+(-b);}
J operator*(J a,J b){return {a.v*b.v,a.first*b.v+a.v*b.first,
    a.second*b.v+I(2)*a.first*b.first+a.v*b.second};}
J chain(J a,I value,I first,I second){return {value,first*a.first,second*square(a.first)+first*a.second};}
J operator/(J a,J b){return a*chain(b,I(1)/b.v,-I(1)/square(b.v),I(2)/(square(b.v)*b.v));}
J sq(J a){return chain(a,square(a.v),I(2)*a.v,I(2));}
J sinD(J a){return chain(a,sine(a.v),cosine(a.v),-sine(a.v));}
J cosD(J a){return chain(a,cosine(a.v),-sine(a.v),-cosine(a.v));}
J atanD(J a){I denominator=I(1)+square(a.v);return chain(a,monotone(a.v,mpfr_atan),I(1)/denominator,-I(2)*a.v/square(denominator));}
J tanD(J a){
    if(a.v.lo < -1.5 || a.v.hi > 1.5)throw std::runtime_error("Actual slip enclosure exceeds +/-1.5 rad.");
    I t=monotone(a.v,mpfr_tan),first=I(1)+square(t);return chain(a,t,first,I(2)*t*first);
}
J tanhD(J a){I t=monotone(a.v,mpfr_tanh),first=I(1)-square(t);return chain(a,t,first,-I(2)*t*first);}
J sqrtD(J a){
    if(a.v.lo<=0)throw std::runtime_error("Square-root domain must be strictly positive.");
    I v=monotone(a.v,mpfr_sqrt);return chain(a,v,I(1)/(I(2)*v),-I(1)/(I(4)*a.v*v));
}
J tire(J slip,J beta,double stiffness,double capacity){
    J t=tanD(slip),available=J(capacity)*sqrtD(J(1)-sq(beta));
    TireBounds b=tireBounds(t.v,available.v,stiffness);I q=b.ratio;
    I sign=t.v.lo>0?I(1):(t.v.hi<0?I(-1):I(-1,1));
    I tt=sign*I(2)*square(I(stiffness))*(I(1)-q)/(I(3)*available.v);
    I tq=-I(2)*I(stiffness)*(I(1)-q)*q/available.v;
    I qq=sign*I(6)*square(q)*(I(1)-q)/available.v;
    return {b.value,b.slipSlope*t.first+b.capacitySlope*available.first,
        b.slipSlope*t.second+b.capacitySlope*available.second+tt*square(t.first)
        +I(2)*tq*t.first*available.first+qq*square(available.first)};
}
using State = std::array<D,8>;
using Flow = std::array<D,6>;
// p: m, Iz, lf, lr, C_f, C_r, capacity_f, capacity_r, rho_air, Cd,
// area, rolling0, rolling1, rolling4, rollingTransition, gravity, speedFloor.
template<class Scalar> std::array<Scalar,6> flow(const std::array<Scalar,8>& y,const double* p){
    Scalar vx=y[3],vy=y[4],r=y[5],delta=y[6],beta=y[7];
    if(vx.v.lo<=p[16] || beta.v.lo<=-1 || beta.v.hi>=1)
        throw std::runtime_error("Domain requires vx above speed floor and abs(beta)<1.");
    Scalar frontSlip=atanD((vy+Scalar(p[2])*r)/vx)-delta;
    Scalar rearSlip=atanD((vy-Scalar(p[3])*r)/vx);
    Scalar front=tire(frontSlip,beta,p[4],p[6]),rear=tire(rearSlip,beta,p[5],p[7]);
    Scalar frontX=Scalar(p[6])*beta*cosD(delta)-front*sinD(delta);
    Scalar frontY=Scalar(p[6])*beta*sinD(delta)+front*cosD(delta);
    Scalar road=Scalar(.5)*Scalar(p[8])*Scalar(p[9])*Scalar(p[10])*sq(vx)
        +Scalar(p[0])*Scalar(p[15])*(Scalar(p[11])+Scalar(p[12])*vx+Scalar(p[13])*sq(sq(vx)))*tanhD(vx/Scalar(p[14]));
    return {vx*cosD(y[2])-vy*sinD(y[2]),vx*sinD(y[2])+vy*cosD(y[2]),r,
        (frontX+Scalar(p[7])*beta-road)/Scalar(p[0])+vy*r,
        (frontY+rear)/Scalar(p[0])-vx*r,(Scalar(p[2])*frontY-Scalar(p[3])*rear)/Scalar(p[1])};
}
const double* vector(const mxArray* a,mwSize count){
    if(!mxIsDouble(a)||mxIsComplex(a)||mxIsSparse(a)||mxGetNumberOfElements(a)!=count)
        throw std::runtime_error("Verifier arguments must be full real doubles with specified dimensions.");
    const double* x=mxGetDoubles(a);for(mwSize i=0;i<count;++i)if(!std::isfinite(x[i]))throw std::runtime_error("Arguments must be finite.");return x;
}
mxArray* intervals(const Flow& f){mxArray* a=mxCreateDoubleMatrix(6,2,mxREAL);double* out=mxGetDoubles(a);for(int i=0;i<6;++i){out[i]=f[i].v.lo;out[i+6]=f[i].v.hi;}return a;}

void validateParameters(const double* p){
    for(int j:{0,1,2,3,4,5,6,7,14,15,16})if(p[j]<=0)throw std::runtime_error("Physical parameters must be positive.");
    for(int j:{8,9,10,11,12,13})if(p[j]<0)throw std::runtime_error("Road-load parameters must be nonnegative.");
}
