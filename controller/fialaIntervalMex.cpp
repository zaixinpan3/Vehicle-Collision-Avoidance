#include "fialaIntervalCore.hpp"

void mexFunction(int nlhs,mxArray* out[],int nrhs,const mxArray* in[]){
    try {
        if(!((nrhs==5 && nlhs==3)||(nrhs==11 && nlhs==4)))throw std::runtime_error("Expected five model arguments, optionally six held-feedback arguments.");
        const double* lower=vector(in[0],8);const double* upper=vector(in[1],8);
        const double* linear=vector(in[2],48);const double* offset=vector(in[3],6);const double* p=vector(in[4],17);
        if(mxGetM(in[2])!=6 || mxGetN(in[2])!=8)throw std::runtime_error("Linear map must be 6 by 8.");
        validateParameters(p);
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
