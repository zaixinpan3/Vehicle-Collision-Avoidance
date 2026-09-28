// Directed geometry, target flow and invariant-ball checks for nonlinear MPC.
// Reuse the validated flow and matrix exponential implementation, including its
// explicit Taylor tail, without duplicating the sampled-flow algorithm.
#define mexFunction nonlinearSampleImplementation
#include "fialaFeedbackSampleMex.cpp"
#undef mexFunction

using Point2=std::array<I,2>;
using Vertices=std::array<Point2,4>;
Box readBox(const mxArray* a,int rows){
    const double* p=vector(a,2*rows);Box b{};
    for(int j=0;j<rows;++j)b[j]=I(p[j],p[j+rows]);return b;
}
I root(I a){if(a.lo<0)throw std::runtime_error("Negative square-root argument.");return monotone(a,mpfr_sqrt);}
I norm2(Point2 a){return root(square(a[0])+square(a[1]));}
I dot(Point2 a,Point2 b){return a[0]*b[0]+a[1]*b[1];}
Point2 rotate(Point2 p,I yaw){I c=cosine(yaw),s=sine(yaw);return {c*p[0]-s*p[1],s*p[0]+c*p[1]};}
Vertices vertices(const Box& box,const double* shape){
    Vertices result;
    for(int j=0;j<4;++j){
        Point2 p={I(shape[2])+I(j<2?-shape[0]:shape[0]),I(shape[3])+I(j%2?-shape[1]:shape[1])};
        p=rotate(p,box[2]);result[j]={box[0]+p[0],box[1]+p[1]};
    }return result;
}
I support(const Box& box,const double* shape,Point2 n){
    Vertices v=vertices(box,shape);double lo=INFINITY,hi=-INFINITY;
    for(const Point2& p:v){I d=dot(n,p);lo=std::min(lo,d.lo);hi=std::max(hi,d.hi);}return {lo,hi};
}
I sincInterval(I a){
    if(magnitude(a)<.01){
        I value(1),power(1);for(int k=1;k<=6;++k){power=power*(-square(a))/I((2*k)*(2*k+1));value=value+power;}
        I remainder(1);for(int k=1;k<=14;++k)remainder=remainder*I(magnitude(a))/I(k+1);
        return value+I(-remainder.hi,remainder.hi);
    }
    if(a.lo<=0 && a.hi>=0)return {-1,1};
    return sine(a)/a;
}
Box targetBox(const double* q,I time){
    I angle=I(q[5])*time/I(2),distance=I(q[3])*time*sincInterval(angle);
    I course=I(q[2])+I(q[4])+angle;Box b{};
    b[0]=I(q[0])+distance*cosine(course);b[1]=I(q[1])+distance*sine(course);
    b[2]=I(q[2])+I(q[5])*time;return b;
}
I quadratic(I a,I b,I c,I x){return a*square(x)+b*x+c;}
I absoluteTime(const mxArray* a){
    if(mxGetNumberOfElements(a)==4){const double* t=vector(a,4);return I(t[0])*I(t[1])+I(t[2],t[3]);}
    const double* t=vector(a,2);return I(t[0],t[1]);
}
I squaredNorm(const Box& box,const Matrix& t){Box mapped=multiply(t,box);I sum;for(int i=0;i<5;++i)sum=sum+square(mapped[i]);return sum;}
Box errorBox(const Box& box,const double* lane,const double* reference){
    Point2 tangent={cosine(I(lane[2])),sine(I(lane[2]))},normal={-tangent[1],tangent[0]};
    Point2 displacement={box[0]-I(lane[0]),box[1]-I(lane[1])};I lateral;
    if(lane[3]==0){lateral=dot(normal,displacement);}
    else{I radius=I(1)/I(lane[3]);Point2 radial={displacement[0]-radius*normal[0],displacement[1]-radius*normal[1]};
        I distance=norm2(radial);if(distance.lo<=0)throw std::runtime_error("Circle projection includes its center.");
        double sign=lane[3]>0?1:-1;tangent={-I(sign)*radial[1]/distance,I(sign)*radial[0]/distance};
        lateral=I(sign)*(absolute(radius)-distance);}
    Point2 direction={cosine(box[2]),sine(box[2])};I c=dot(tangent,direction);
    if(c.lo<=0)throw std::runtime_error("Heading error is outside the local reference chart.");
    I s=tangent[0]*direction[1]-tangent[1]*direction[0];Box e{};
    e[0]=lateral-I(reference[1]);e[1]=monotone(s/c,mpfr_atan)-I(reference[2]);
    for(int i=2;i<5;++i)e[i]=box[i+1]-I(reference[i+1]);return e;
}
std::array<D,5> errorDifferential(const Box& box,const double* lane,const double* reference){
    State x=state(box);D tx(cosine(I(lane[2]))),ty(sine(I(lane[2])));
    D dx=x[0]-D(lane[0]),dy=x[1]-D(lane[1]),lateral;
    if(lane[3]==0)lateral=-ty*dx+tx*dy;
    else{D radius(I(1)/I(lane[3]));D rx=dx+radius*ty,ry=dy-radius*tx;
        D distance=sqrtD(sq(rx)+sq(ry));double sign=lane[3]>0?1:-1;
        tx=-D(sign)*ry/distance;ty=D(sign)*rx/distance;
        lateral=D(sign)*(D(absolute(radius.v))-distance);}
    D c=tx*cosD(x[2])+ty*sinD(x[2]);
    if(c.v.lo<=0)throw std::runtime_error("CLF gradient leaves its local chart.");
    D s=tx*sinD(x[2])-ty*cosD(x[2]);
    return {lateral-D(reference[1]),atanD(s/c)-D(reference[2]),
        x[3]-D(reference[3]),x[4]-D(reference[4]),x[5]-D(reference[5])};
}
Matrix intervalMap(const Matrix& f,I duration){
    // Peano--Baker products: each occurrence of the interval Jacobian may
    // take an independent value. This also encloses time-varying Jacobians.
    Matrix result=identity(),power=identity();I norm;
    for(int i=0;i<8;++i){I sum;for(int j=0;j<8;++j)sum=sum+I(magnitude(f[i][j]));norm=I(std::max(norm.hi,sum.hi));}
    constexpr int order=12;
    for(int k=1;k<=order;++k){power=multiply(power,f);
        for(int i=0;i<8;++i)for(int j=0;j<8;++j){power[i][j]=power[i][j]*duration/I(k);result[i][j]=result[i][j]+power[i][j];}}
    I z=norm*duration,tail(1);for(int k=1;k<=order+1;++k)tail=tail*z/I(k);
    tail=tail*monotone(z,mpfr_exp);
    for(int i=0;i<6;++i)for(int j=0;j<8;++j)result[i][j]=result[i][j]+I(-tail.hi,tail.hi);
    return result;
}
void clfGradient(mxArray** out,const mxArray** in){
    const mxArray* sample=in[1];const mxArray* accepted=mxGetField(sample,0,"accepted");
    if(!accepted || !mxIsLogicalScalarTrue(accepted))throw std::runtime_error("CLF bounds require an accepted region flow.");
    const double* parameters=vector(in[2],17),*lane=vector(in[3],6),*reference=vector(in[4],6),*factor=vector(in[5],25),*scale=vector(in[6],2);
    const mxArray* cells=mxGetField(sample,0,"cells");Matrix transition=identity();
    for(size_t index=0;index<mxGetNumberOfElements(cells);++index){
        Box domain=readBox(mxGetField(cells,index,"domain"),8);Flow field=flow(state(domain),parameters);Matrix jacobian{};
        for(int i=0;i<6;++i)for(int j=0;j<8;++j)jacobian[i][j]=field[i].d[j];
        I duration=I(vector(mxGetField(cells,index,"end"),1)[0])-I(vector(mxGetField(cells,index,"start"),1)[0]);
        transition=multiply(intervalMap(jacobian,duration),transition);
    }
    auto errors=errorDifferential(readBox(mxGetField(sample,0,"endpoint"),8),lane,reference);D value;
    for(int i=0;i<5;++i){D component;for(int j=0;j<5;++j)component=component+D(factor[i+5*j])*errors[j];value=value+sq(component);}
    out[0]=mxCreateDoubleMatrix(2,2,mxREAL);double* result=mxGetDoubles(out[0]);
    for(int u=0;u<2;++u){I gradient;for(int j=0;j<6;++j)gradient=gradient+value.d[j]*transition[j][6+u];
        gradient=gradient*I(scale[u]);double middle=midpoint(gradient);result[u]=middle;result[u+2]=magnitude(gradient-I(middle));}
}
struct Orbit {Point2 center;I minimum,maximum;};
Orbit orbit(const double* q){
    I radius=I(q[3])/I(q[5]);Point2 velocityDirection={cosine(I(q[2])+I(q[4])),sine(I(q[2])+I(q[4]))};
    Point2 center={I(q[0])-radius*velocityDirection[1],I(q[1])+radius*velocityDirection[0]};
    Point2 w={-radius*sine(I(q[4]))-I(q[8]),radius*cosine(I(q[4]))-I(q[9])};
    Point2 near,far;for(int j=0;j<2;++j){I d=absolute(w[j])-I(q[6+j]);near[j]=I(std::max(0.0,d.lo),std::max(0.0,d.hi));far[j]=absolute(w[j])+I(q[6+j]);}
    return {center,norm2(near),norm2(far)};
}
double tailMargin(const Box& entry,const Box& domain,const double* shape,const double* q,const double* lane,double clearance,I time){
    Point2 t={cosine(I(lane[2])),sine(I(lane[2]))},n={-t[1],t[0]},origin={I(lane[0]),I(lane[1])};
    Box local=domain;local[0]=I();I lateral=support(local,shape,{I(0),I(1)});
    double road=std::min((lateral+I(lane[4])).lo,(I(lane[5])-lateral).lo);
    if(lane[3]!=0){
        I body=norm2({I(shape[0])+I(std::abs(shape[2])),I(shape[1])+I(std::abs(shape[3]))});
        I radius=absolute(I(1)/I(lane[3])),width=absolute(domain[1])+body;
        road=std::min((I(lane[4])-width).lo,(I(lane[5])-width).lo);
        if(!q)return road;
        I signedRadius=I(1)/I(lane[3]);Point2 center={origin[0]+signedRadius*n[0],origin[1]+signedRadius*n[1]};
        I targetMinimum,targetMaximum;
        if(q[5]!=0){Orbit o=orbit(q);I d=norm2({o.center[0]-center[0],o.center[1]-center[1]});
            targetMinimum=I(std::max({0.0,(d-o.maximum).lo,(o.minimum-d).lo}));targetMaximum=d+o.maximum;
        }else{
            Box current=targetBox(q,time);Point2 d={current[0]-center[0],current[1]-center[1]},v={I(q[3])*cosine(I(q[2])+I(q[4])),I(q[3])*sine(I(q[2])+I(q[4]))};
            I reach=norm2({I(q[6])+I(std::abs(q[8])),I(q[7])+I(std::abs(q[9]))});
            I time;if(q[3]>0){I raw=-dot(d,v)/square(I(q[3]));time=I(std::max(0.0,raw.lo),std::max(0.0,raw.hi));}
            targetMinimum=norm2({d[0]+time*v[0],d[1]+time*v[1]})-reach;
            if(q[3]>0)return std::min(road,(targetMinimum-radius-width-I(clearance)).lo);
            targetMaximum=norm2(d)+reach;
        }
        double collision=std::max((targetMinimum-radius-width-I(clearance)).lo,(radius-width-targetMaximum-I(clearance)).lo);
        return std::min(road,collision);
    }
    if(!q)return road;
    Box target=targetBox(q,time);I targetLateral=support(target,q+6,n)-dot(n,origin);
    double lower=targetLateral.lo,upper=targetLateral.hi;Point2 velocity={I(q[3])*cosine(I(q[2])+I(q[4])),I(q[3])*sine(I(q[2])+I(q[4]))};
    if(q[5]!=0){Orbit o=orbit(q);I projection=dot(n,{o.center[0]-origin[0],o.center[1]-origin[1]});
        lower=(projection-o.maximum).lo;upper=(projection+o.maximum).hi;
    }else if(q[3]>0){I speed=dot(n,velocity);if(speed.lo<0)lower=-INFINITY;if(speed.hi>0)upper=INFINITY;}
    double collision=-INFINITY;
    if(std::isfinite(upper))collision=std::max(collision,(I(lateral.lo)-I(upper)-I(clearance)).lo);
    if(std::isfinite(lower))collision=std::max(collision,(I(lower)-I(lateral.hi)-I(clearance)).lo);
    // A separating forward halfspace is invariant when ego progress dominates
    // straight-target progress, or when ego is beyond the entire circular sweep.
    I forward=domain[3]*cosine(domain[2])-domain[4]*sine(domain[2]);
    I egoCenter=dot(t,{entry[0],entry[1]});I body=support(local,shape,{I(1),I(0)});
    I targetSupport=support(target,q+6,t);bool movingApart=false;
    if(q[5]!=0){Orbit o=orbit(q);targetSupport=dot(t,o.center)+o.maximum;movingApart=forward.lo>=0;}
    else movingApart=forward.lo>=dot(t,velocity).hi;
    if(movingApart)collision=std::max(collision,(egoCenter+I(body.lo)-I(targetSupport.hi)-I(clearance)).lo);
    return std::min(road,collision);
}
double roadMargin(const Box& box,const double* shape,const double* lane,const mxArray* boundaries){
    Vertices v=vertices(box,shape);double margin=INFINITY;
    Point2 tangent={cosine(I(lane[2])),sine(I(lane[2]))};Point2 normal={-tangent[1],tangent[0]};
    if(lane[3]==0){
        I value=support(box,shape,normal)-dot(normal,{I(lane[0]),I(lane[1])});
        margin=std::min((value+I(lane[4])).lo,(I(lane[5])-value).lo);
    }else{
        I radius=I(1)/I(lane[3]);Point2 center={I(lane[0])+radius*normal[0],I(lane[1])+radius*normal[1]};
        I radial[2]={v[0][0]-center[0],v[0][1]-center[1]};double maximum=0;
        for(const Point2& p:v){Point2 d={p[0]-center[0],p[1]-center[1]};
            radial[0]=hull(radial[0],d[0]);radial[1]=hull(radial[1],d[1]);maximum=std::max(maximum,norm2(d).hi);}
        I range(root(square(radial[0])+square(radial[1])).lo,maximum);
        I lateral=lane[3]>0?radius-range:range+radius;
        margin=std::min((lateral+I(lane[4])).lo,(I(lane[5])-lateral).lo);
    }
    // Each row: origin(2), tangent(2), normal(2), quadratic(3), range(2), sign.
    const double* b=mxGetDoubles(boundaries);size_t rows=mxGetM(boundaries);
    if(mxGetN(boundaries)!=12)throw std::runtime_error("Road boundaries need twelve columns.");
    for(size_t j=0;j<rows;++j){
        Point2 t={I(b[j+2*rows]),I(b[j+3*rows])},n={I(b[j+4*rows]),I(b[j+5*rows])};
        Point2 origin={I(b[j]),I(b[j+rows])};I xs,ys;bool first=true;
        for(const Point2& p:v){Point2 d={p[0]-origin[0],p[1]-origin[1]};I x=dot(t,d),y=dot(n,d);
            if(first){xs=x;ys=y;first=false;}else{xs=hull(xs,x);ys=hull(ys,y);}}
        I residual=I(b[j+11*rows])*(ys-quadratic(I(b[j+6*rows]),I(b[j+7*rows]),I(b[j+8*rows]),xs));
        margin=std::min({margin,residual.lo,(xs-I(b[j+9*rows])).lo,(I(b[j+10*rows])-xs).lo});
    }return margin;
}
Matrix inverseMatrix(Matrix a,int count){
    Matrix b=identity();
    for(int k=0;k<count;++k){
        int pivot=k;for(int j=k+1;j<count;++j)if(magnitude(a[j][k])>magnitude(a[pivot][k]))pivot=j;
        std::swap(a[k],a[pivot]);std::swap(b[k],b[pivot]);I d=a[k][k];
        for(int j=0;j<count;++j){a[k][j]=a[k][j]/d;b[k][j]=b[k][j]/d;}
        for(int i=0;i<count;++i)if(i!=k){I q=a[i][k];
            for(int j=0;j<count;++j){a[i][j]=a[i][j]-q*a[k][j];b[i][j]=b[i][j]-q*b[k][j];}}
    }return b;
}
bool positiveDefinite(Matrix a,int count){
    // Interval LDL: positive lower pivots prove SPD for the enclosed matrix.
    for(int k=0;k<count;++k){if(a[k][k].lo<=0)return false;
        for(int i=k+1;i<count;++i)for(int j=i;j<count;++j){
            a[j][i]=a[j][i]-a[i][k]*a[j][k]/a[k][k];a[i][j]=a[j][i];}}
    return true;
}
double operatorNorm(const Matrix& a,int count){
    Matrix gram{};for(int i=0;i<count;++i)for(int j=0;j<count;++j)
        for(int k=0;k<count;++k)gram[i][j]=gram[i][j]+a[k][i]*a[k][j];
    double low=0,high=2;
    auto valid=[&](double value){Matrix m{};for(int i=0;i<count;++i)for(int j=0;j<count;++j)
        m[i][j]=(i==j?square(I(value)):I())-gram[i][j];return positiveDefinite(m,count);};
    while(!valid(high)){high*=2;if(high>1e6)throw std::runtime_error("Norm verification failed.");}
    for(int k=0;k<45;++k){double mid=low+(high-low)/2;if(valid(mid))high=mid;else low=mid;}
    return high;
}
Matrix readMatrix(const mxArray* a,int rows,int cols){
    const double* p=vector(a,rows*cols);Matrix m{};
    for(int i=0;i<rows;++i)for(int j=0;j<cols;++j)m[i][j]=I(p[i+rows*j]);return m;
}
Maps wholeMaps(const Matrix& f,double h){
    int count=0;double step=h;while(step>.001){step/=2;++count;}
    Maps result=maps(f,step);Matrix positive{};
    // Metzler comparison preserves stable diagonal entries:
    // |exp(A t)| <= exp(M t), M_ii=A_ii and M_ij=|A_ij| for i != j.
    for(int i=0;i<8;++i)for(int j=0;j<8;++j)positive[i][j]=i==j?f[i][j]:I(magnitude(f[i][j]));
    Maps comparison=maps(positive,step);Matrix absoluteTransition=comparison.transition;
    result.absoluteIntegral=comparison.integral;
    for(int k=0;k<count;++k){
        Matrix integrated=multiply(result.transition,result.integral),absolute=multiply(absoluteTransition,result.absoluteIntegral);
        for(int i=0;i<8;++i)for(int j=0;j<8;++j){result.integral[i][j]=result.integral[i][j]+integrated[i][j];result.absoluteIntegral[i][j]=result.absoluteIntegral[i][j]+absolute[i][j];}
        result.transition=multiply(result.transition,result.transition);absoluteTransition=multiply(absoluteTransition,absoluteTransition);
    }
    result.tail=0;return result;
}
void terminalBound(mxArray** out,const mxArray** in){
    const double* reference=vector(in[1],8);Matrix gain=readMatrix(in[2],2,5),factor=readMatrix(in[3],5,5);
    Matrix a=readMatrix(in[4],5,5),b=readMatrix(in[5],5,2);Box domain=readBox(in[6],8);
    double h=vector(in[7],1)[0];const double* p=vector(in[8],17);fialaFrenetCurvature=vector(in[9],1)[0];
    Point point;std::copy(reference,reference+8,point.begin());Flow nominal=flow(state(pointBox(point)),p);
    std::array<J,8> directions;
    for(int j=0;j<8;++j)directions[j]=J(domain[j],domain[j]-I(reference[j]));
    auto curvature=flow(directions,p);Box residual{};
    for(int i=0;i<5;++i){I value=nominal[i+1].v;
        for(int j=0;j<5;++j)value=value+(nominal[i+1].d[j+1]-a[i][j])*(domain[j+1]-I(reference[j+1]));
        for(int j=0;j<2;++j)value=value+(nominal[i+1].d[j+6]-b[i][j])*(domain[j+6]-I(reference[j+6]));
        value=value+I(.5)*curvature[i+1].second;residual[i]=I(magnitude(value));
        for(int j=0;j<2;++j)residual[i]=residual[i]+absolute(b[i][j])*I(1e-12);}
    Matrix generator=a;for(int i=0;i<5;++i)for(int j=0;j<2;++j)generator[i][j+5]=b[i][j];
    Maps sampled=wholeMaps(generator,h);Matrix closed{};
    for(int i=0;i<5;++i)for(int j=0;j<5;++j){closed[i][j]=sampled.transition[i][j];
        for(int k=0;k<2;++k)closed[i][j]=closed[i][j]+sampled.transition[i][k+5]*gain[k][j];}
    Matrix transformed=multiply(multiply(factor,closed),inverseMatrix(factor,5));
    double contraction=operatorNorm(transformed,5);
    Maps linear=wholeMaps(a,h);Box integrated=multiply(linear.absoluteIntegral,residual);double maximum=0;
    for(int i=0;i<5;++i)maximum=std::max(maximum,residual[i].hi);
    I tail=I(h)*I(linear.tail)*I(maximum),sum;
    for(int i=0;i<5;++i){I component;for(int j=0;j<5;++j)component=component+absolute(factor[i][j])*(integrated[j]+tail);
        sum=sum+square(component);}
    I radius(vector(in[10],1)[0]);out[0]=mxCreateDoubleMatrix(3,1,mxREAL);mxGetDoubles(out[0])[0]=contraction;mxGetDoubles(out[0])[1]=root(sum).hi;
    mxGetDoubles(out[0])[2]=(radius-I(contraction)*radius-root(sum)).lo;
}
extern "C" void mexFunction(int nlhs,mxArray* out[],int nrhs,const mxArray* in[]){
    try{
        if(nrhs<1 || !mxIsChar(in[0]) || nlhs!=1)throw std::runtime_error("Expected action and one output.");
        char* name=mxArrayToString(in[0]);std::string action(name);mxFree(name);
        fialaFrenetCurvature=std::numeric_limits<double>::quiet_NaN();
        if(action=="feedbackInput" && nrhs==4){const double* x=vector(in[1],6),*nominal=vector(in[2],8);Matrix k=readMatrix(in[3],2,6);
            out[0]=mxCreateDoubleMatrix(2,1,mxREAL);for(int i=0;i<2;++i){I u(nominal[i+6]);
                for(int j=0;j<6;++j)u=u+k[i][j]*(I(x[j])-I(nominal[j]));double value=midpoint(u);
                if(magnitude(u-I(value))>1e-12)throw std::runtime_error("Feedback evaluation exceeds its certified rounding allowance.");
                mxGetDoubles(out[0])[i]=value;}return;}
        if(action=="backupInput" && nrhs==5){const double* x=vector(in[1],6),*lane=vector(in[2],6),*reference=vector(in[3],8);
            Box box{};for(int j=0;j<6;++j)box[j]=I(x[j]);Box e=errorBox(box,lane,reference);Matrix k=readMatrix(in[4],2,5);
            out[0]=mxCreateDoubleMatrix(2,1,mxREAL);for(int i=0;i<2;++i){I u(reference[i+6]);
                for(int j=0;j<5;++j)u=u+k[i][j]*e[j];double value=midpoint(u);
                if(magnitude(u-I(value))>1e-12)throw std::runtime_error("Backup evaluation exceeds its certified rounding allowance.");
                mxGetDoubles(out[0])[i]=value;}return;}
        if(action=="backupWorld" && nrhs==4){const double* x=vector(in[1],6),*lane=vector(in[3],6);Box domain=readBox(in[2],8),point{};
            for(int j=0;j<6;++j)point[j]=I(x[j]);double zeroReference[6]={};Box e=errorBox(point,lane,zeroReference);
            I heading=point[2]-e[1];Point2 t={cosine(heading),sine(heading)},n={-t[1],t[0]};
            Point2 origin={point[0]-e[0]*n[0],point[1]-e[0]*n[1]};I angle=I(lane[3])*domain[0],along=domain[0],across;
            if(lane[3]!=0){along=sine(angle)/I(lane[3]);across=(I(1)-cosine(angle))/I(lane[3]);}
            Point2 normal={n[0]*cosine(angle)-t[0]*sine(angle),n[1]*cosine(angle)-t[1]*sine(angle)};
            Box world=domain;for(int j=0;j<2;++j)world[j]=origin[j]+t[j]*along+n[j]*across+normal[j]*domain[1];
            world[2]=heading+angle+domain[2];out[0]=boxArray(world,8);return;}
        if(action=="terminalBound" && nrhs==11){terminalBound(out,in);return;}
        if(action=="clfGradient" && nrhs==7){clfGradient(out,in);return;}
        if(action=="inputBoxContains" && nrhs==4){size_t count=mxGetNumberOfElements(in[1]);
            const double* candidate=vector(in[1],count),*anchor=vector(in[2],count),*radius=vector(in[3],count);bool contains=true;
            for(size_t j=0;j<count;++j)contains=contains && radius[j]>=0 && magnitude(I(candidate[j])-I(anchor[j]))<=radius[j];
            out[0]=mxCreateLogicalScalar(contains);return;}
        if(action=="inlet" && nrhs==3){const double* x=vector(in[1],6);const double* r=vector(in[2],5);Box box{};
            for(int j=0;j<6;++j)box[j]=I(x[j])+(j?I(-r[j-1],r[j-1]):I());out[0]=boxArray(box,6);return;}
        if(action=="clf" && nrhs==5){Matrix t=readMatrix(in[3],5,5);I before=squaredNorm(readBox(in[1],5),t),after=squaredNorm(readBox(in[2],5),t);
            I residual=I(after.hi)-(I(1)-I(vector(in[4],1)[0]))*I(before.lo);out[0]=mxCreateDoubleMatrix(4,1,mxREAL);
            double* p=mxGetDoubles(out[0]);p[0]=before.lo;p[1]=after.hi;p[2]=std::max(0.0,residual.hi);p[3]=residual.hi;return;}
        if(action=="radii" && nrhs==3){Matrix inverse=inverseMatrix(readMatrix(in[1],5,5),5);I radius(vector(in[2],1)[0]);out[0]=mxCreateDoubleMatrix(5,1,mxREAL);
            for(int i=0;i<5;++i){I sum;for(int j=0;j<5;++j)sum=sum+square(inverse[i][j]);mxGetDoubles(out[0])[i]=(radius*root(sum)).hi;}return;}
        if(action=="error" && nrhs==4){Box e=errorBox(readBox(in[1],6),vector(in[2],6),vector(in[3],6));out[0]=boxArray(e,5);return;}
        if(action=="tail" && nrhs==8){const double* q=mxIsEmpty(in[4])?nullptr:vector(in[4],10);
            I time;if(mxGetNumberOfElements(in[7])==1)time=I(vector(in[7],1)[0]);else{const double* t=vector(in[7],2);time=I(t[0])*I(t[1]);}
            out[0]=mxCreateDoubleScalar(tailMargin(readBox(in[1],6),readBox(in[2],8),vector(in[3],4),q,vector(in[5],6),vector(in[6],1)[0],time));return;}
        if(action=="target" && nrhs==3){const double* q=vector(in[1],10);const double* t=vector(in[2],2);
            out[0]=boxArray(targetBox(q,I(t[0],t[1])),3);return;}
        if(action=="support" && nrhs==4){Box box=readBox(in[1],3);const double* shape=vector(in[2],4);const double* n=vector(in[3],2);
            I s=support(box,shape,{I(n[0]),I(n[1])});out[0]=mxCreateDoubleMatrix(1,2,mxREAL);
            mxGetDoubles(out[0])[0]=s.lo;mxGetDoubles(out[0])[1]=s.hi;return;}
        if(action=="norm" && nrhs==3){Box box=readBox(in[1],5);Matrix t=readMatrix(in[2],5,5);Box transformed=multiply(t,box);I sum;
            for(int i=0;i<5;++i)sum=sum+square(transformed[i]);out[0]=mxCreateDoubleMatrix(1,2,mxREAL);
            mxGetDoubles(out[0])[0]=root(sum).lo;mxGetDoubles(out[0])[1]=root(sum).hi;return;}
        if(action=="geometry" && nrhs==9){Box box=readBox(in[1],6);const double* shape=vector(in[2],4);
            const double* lane=vector(in[6],6);double clearance=vector(in[8],1)[0];double separation=INFINITY;
            if(!mxIsEmpty(in[3])){const double* q=vector(in[3],10);I time=absoluteTime(in[4]);const double* direction=vector(in[5],2);
                Point2 n={I(direction[0]),I(direction[1])};I norm=norm2(n);if(norm.lo<=0)throw std::runtime_error("Zero separation normal.");
                n={n[0]/norm,n[1]/norm};I ego=support(box,shape,n),target=support(targetBox(q,time),q+6,n);
                separation=(I(ego.lo)-I(target.hi)-I(clearance)).lo;}
            double road=roadMargin(box,shape,lane,in[7]);out[0]=mxCreateDoubleMatrix(2,1,mxREAL);
            mxGetDoubles(out[0])[0]=separation;mxGetDoubles(out[0])[1]=road;return;}
        throw std::runtime_error("Unknown action or invalid argument count.");
    }catch(const std::exception& e){mexErrMsgIdAndTxt("collisionAvoidanceController:invalidNonlinearCertificate","%s",e.what());}
}
