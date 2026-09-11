// Test adapter for the public interval arithmetic and differential kernels.
#include "../controller/fialaIntervalCore.hpp"
#include <limits>
void mexFunction(int nlhs,mxArray* out[],int nrhs,const mxArray* in[]){
    try{
        if(nrhs!=4 || nlhs!=3)throw std::runtime_error("Four inputs and three outputs required.");
        const double* lower=vector(in[0],8);const double* upper=vector(in[1],8);
        const double* direction=vector(in[2],8);const double* p=vector(in[3],17);
        std::array<J,8> domain;
        for(int j=0;j<8;++j)domain[j]=J(I(lower[j],upper[j]),I(direction[j]));
        auto result=flow(domain,p);Flow first,second;
        for(int j=0;j<6;++j){first[j]=D(result[j].first);second[j]=D(result[j].second);}
        out[0]=intervals(first);out[1]=intervals(second);
        double tiny=std::numeric_limits<double>::denorm_min(),large=std::numeric_limits<double>::max();
        std::array<I,10> cases={I(-5,-2),I(-1,0),I(0),I(1,4),I(-2,3),I(tiny),I(-tiny,tiny),I(large),I(1e-300,1e-299),I(-large)};
        int verified=0;
        for(I a:cases)for(I b:cases){
            double low=INFINITY,high=-INFINITY;
            // Exhaustive endpoint enumeration is independent of the optimized
            // sign table, including underflow and overflow outcomes.
            for(double x:{a.lo,a.hi})for(double y:{b.lo,b.hi}){
                low=std::min(low,rounded(mpfr_mul,x,y,MPFR_RNDD));
                high=std::max(high,rounded(mpfr_mul,x,y,MPFR_RNDU));
            }
            bool overflow=!std::isfinite(low)||!std::isfinite(high);
            try{I product=a*b;if(!overflow && product.lo==low && product.hi==high)++verified;}
            catch(const std::runtime_error&){if(overflow)++verified;}
        }
        out[2]=mxCreateDoubleScalar(verified);
    }catch(const std::exception& error){mexErrMsgIdAndTxt("fialaIntervalKernelTest:failure","%s",error.what());}
}
