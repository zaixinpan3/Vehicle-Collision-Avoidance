// Complete zero-order-held feedback sample; correlated augmented-state tube.
#include "fialaIntervalCore.hpp"
#include <vector>
#include <chrono>
#include <numeric>
struct Profile {double enclosure=0,linearization=0,matrixMaps=0,nominal=0,endpoint=0;};
using Clock=std::chrono::steady_clock;
struct Timer {
    double& total;Clock::time_point start=Clock::now();
    explicit Timer(double& value):total(value){}
    ~Timer(){total+=std::chrono::duration<double>(Clock::now()-start).count();}
};
using Box=std::array<I,8>;
using Point=std::array<double,8>;
using Matrix=std::array<std::array<I,8>,8>;
using Generators=std::array<std::vector<double>,8>;
struct Tube {Point center{};Generators generators{};Box remainder{};};
struct Cell {double start,end;Box domain,swept,endpoint;Flow residual;Tube tube;};
double midpoint(I a){return std::clamp(a.lo/2+a.hi/2,a.lo,a.hi);}
double magnitude(I a){return std::max(std::abs(a.lo),std::abs(a.hi));}
Box outer(const Tube& t){
    Box box;
    for(int i=0;i<8;++i){I radius;for(double g:t.generators[i])radius=radius+I(std::abs(g));
        box[i]=I(t.center[i])+I(-radius.hi,radius.hi)+t.remainder[i];}
    return box;
}
State state(const Box& box,bool derivatives=true){
    State y;for(int i=0;i<8;++i){y[i]=D(box[i]);if(derivatives)y[i].d[i]=I(1);}return y;
}
Box pointBox(const Point& p){Box box;for(int i=0;i<8;++i)box[i]=I(p[i]);return box;}
Matrix identity(){Matrix a{};for(int i=0;i<8;++i)a[i][i]=I(1);return a;}
Matrix multiply(const Matrix& a,const Matrix& b){
    Matrix c{};
    for(int i=0;i<8;++i)for(int k=0;k<8;++k)if(!zero(a[i][k])){
        for(int j=0;j<8;++j)if(!zero(b[k][j]))c[i][j]=c[i][j]+a[i][k]*b[k][j];
    }
    return c;
}
Box multiply(const Matrix& a,const Box& b){
    Box c{};for(int i=0;i<8;++i)for(int j=0;j<8;++j)if(!zero(a[i][j]))c[i]=c[i]+a[i][j]*b[j];return c;
}
struct Maps {Matrix transition,integral,absoluteIntegral;double tail;};
Maps maps(const Matrix& f,double duration){
    // Directed Taylor enclosure. The scalar tail bounds an induced infinity
    // norm, hence every matrix entry. Bottom rows are known exactly.
    constexpr int order=12;I h(duration),norm;
    for(int i=0;i<8;++i){I sum;for(int j=0;j<8;++j)sum=sum+I(magnitude(f[i][j]));norm=I(std::max(norm.hi,sum.hi));}
    I z=norm*h;
    Matrix power=identity(),positivePower=identity(),positive{};
    for(int i=0;i<8;++i)for(int j=0;j<8;++j)positive[i][j]=I(magnitude(f[i][j]));
    Maps result{identity(),{}, {},0};
    for(int i=0;i<8;++i){result.integral[i][i]=h;result.absoluteIntegral[i][i]=h;}
    for(int k=1;k<=order;++k){
        power=multiply(power,f);positivePower=multiply(positivePower,positive);
        for(int i=0;i<8;++i)for(int j=0;j<8;++j){
            power[i][j]=power[i][j]*h/I(k);positivePower[i][j]=positivePower[i][j]*h/I(k);
            result.transition[i][j]=result.transition[i][j]+power[i][j];
            result.integral[i][j]=result.integral[i][j]+power[i][j]*h/I(k+1);
            result.absoluteIntegral[i][j]=result.absoluteIntegral[i][j]+positivePower[i][j]*h/I(k+1);
        }
    }
    I tail(1);for(int k=1;k<=order+1;++k)tail=tail*z/I(k);
    tail=tail*monotone(z,mpfr_exp);result.tail=tail.hi;
    I entry(-tail.hi,tail.hi),integralError=entry*h;
    for(int i=0;i<6;++i)for(int j=0;j<8;++j){
        result.transition[i][j]=result.transition[i][j]+entry;
        result.integral[i][j]=result.integral[i][j]+integralError;
    }
    // absoluteIntegral uses its polynomial only; its tail is charged once
    // by an induced-norm bound when integrating arbitrary time-varying error.
    return result;
}
Point rk4(const Point& initial,double h,const double* parameters){
    // Nominal proposal only. All discrepancy from the validated affine flow
    // is charged to the returned tube remainder; RK4 is never a certificate.
    auto evaluate=[&](Point x){Flow f=flow(state(pointBox(x),false),parameters);Point y{};
        for(int i=0;i<6;++i)y[i]=midpoint(f[i].v);
        return y;};
    Point k1=evaluate(initial),p=initial;
    for(int i=0;i<6;++i)p[i]=initial[i]+h*k1[i]/2;
    Point k2=evaluate(p);
    for(int i=0;i<6;++i)p[i]=initial[i]+h*k2[i]/2;
    Point k3=evaluate(p);
    for(int i=0;i<6;++i)p[i]=initial[i]+h*k3[i];
    Point k4=evaluate(p);
    Point result=initial;for(int i=0;i<6;++i)result[i]=initial[i]+h*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6;return result;
}
void resize(Tube& tube,size_t count){for(auto& row:tube.generators)row.resize(count,0);}
void promoteRemainder(Tube& tube,int dimensions=8){
    // Before the next sampled feedback evaluation, preserve each old box
    // direction as a shared generator rather than duplicating K*R and R.
    size_t count=tube.generators[0].size();resize(tube,count+dimensions);
    for(int j=0;j<dimensions;++j){
        double center=midpoint(tube.remainder[j]);I shift=I(tube.center[j])+I(center);
        tube.center[j]=midpoint(shift);
        tube.generators[j][count+j]=magnitude(tube.remainder[j]-I(center));
        tube.remainder[j]=shift-I(tube.center[j]);
    }
}
void reduceGenerators(Tube& tube,size_t maximum){
    size_t count=tube.generators[0].size();
    if(count<=maximum)return;
    // Ranking affects tightness only. Every discarded direction is charged
    // outward to the remainder before it becomes shared at the next sample.
    constexpr double scales[8]={1,1,.1,10,1,1,.1,1};
    std::vector<size_t> order(count);std::iota(order.begin(),order.end(),0);
    std::vector<double> scores(count,0);
    for(size_t g=0;g<count;++g)for(int j=0;j<8;++j)scores[g]+=std::abs(tube.generators[j][g])/scales[j];
    std::stable_sort(order.begin(),order.end(),[&](size_t a,size_t b){return scores[a]>scores[b];});
    Generators retained;
    for(int j=0;j<8;++j){
        retained[j].reserve(maximum);I radius;
        for(size_t g=0;g<maximum;++g)retained[j].push_back(tube.generators[j][order[g]]);
        for(size_t g=maximum;g<count;++g)radius=radius+I(std::abs(tube.generators[j][order[g]]));
        tube.remainder[j]=tube.remainder[j]+I(-radius.hi,radius.hi);
    }
    tube.generators=std::move(retained);
}
Tube inletTube(const double* lower,const double* upper,const double* previous,const mxArray* representation,size_t maximumGenerators){
    Tube tube;
    if(mxIsEmpty(representation)){
        resize(tube,6);
        for(int j=0;j<6;++j){
            tube.center[j]=midpoint(I(lower[j],upper[j]));
            tube.generators[j][j]=magnitude(I(lower[j],upper[j])-I(tube.center[j]));
        }
        for(int j=0;j<2;++j)tube.center[6+j]=previous[j];
        return tube;
    }
    if(!mxIsStruct(representation)||mxGetNumberOfElements(representation)!=1)
        throw std::runtime_error("Correlated inlet must be a scalar tube structure.");
    auto field=[&](const char* name){const mxArray* value=mxGetField(representation,0,name);
        if(!value)throw std::runtime_error("Correlated inlet requires center, generators and remainder.");
        return value;};
    const double* center=vector(field("center"),8);
    const mxArray* matrix=field("generators");size_t count=mxGetN(matrix);
    if(mxGetM(matrix)!=8 || count>4096)throw std::runtime_error("Invalid correlated inlet generator dimensions.");
    const double* generators=vector(matrix,8*count);
    const mxArray* remainder=field("remainder");const double* bounds=vector(remainder,16);
    if(mxGetM(remainder)!=8 || mxGetN(remainder)!=2)throw std::runtime_error("Remainder must be 8 by 2.");
    resize(tube,count);
    for(int j=0;j<8;++j){tube.center[j]=center[j];tube.remainder[j]=I(bounds[j],bounds[8+j]);
        for(size_t g=0;g<count;++g)tube.generators[j][g]=generators[j+8*g];}
    reduceGenerators(tube,maximumGenerators-14);promoteRemainder(tube);return tube;
}
Tube feedbackTube(const Tube& inlet,const double* nominal,const double* gain,const double* noise,Box& slew){
    Tube tube=inlet;size_t count=inlet.generators[0].size();resize(tube,count+6);
    for(int u=0;u<2;++u){
        I command(nominal[6+u]),remainder;
        for(int j=0;j<6;++j){
            command=command+I(gain[u+2*j])*(I(inlet.center[j])-I(nominal[j]));
            remainder=remainder+I(gain[u+2*j])*inlet.remainder[j];
        }
        tube.center[6+u]=midpoint(command);
        tube.remainder[6+u]=remainder+command-I(tube.center[6+u]);
        I difference=I(tube.center[6+u])-I(inlet.center[6+u]);
        I radius;
        for(size_t g=0;g<count+6;++g){
            I coefficient;
            if(g<count){for(int j=0;j<6;++j)coefficient=coefficient+I(gain[u+2*j])*I(inlet.generators[j][g]);}
            else coefficient=I(gain[u+2*(g-count)])*I(noise[g-count]);
            tube.generators[6+u][g]=midpoint(coefficient);
            double error=magnitude(coefficient-I(tube.generators[6+u][g]));
            tube.remainder[6+u]=tube.remainder[6+u]+I(-error,error);
            I change=I(tube.generators[6+u][g])-I(g<count?inlet.generators[6+u][g]:0);
            radius=radius+I(magnitude(change));
        }
        slew[u]=difference+I(-radius.hi,radius.hi)+tube.remainder[6+u]-inlet.remainder[6+u];
    }return tube;
}
bool enclose(const Tube& tube,double h,const double* parameters,Box& domain,Box& swept,Flow& field){
    Box inlet=outer(tube);Flow initialFlow=flow(state(inlet,false),parameters);
    for(int i=0;i<8;++i){
        if(i>=6){domain[i]=inlet[i];continue;}
        I probe=hull(inlet[i]+I(0,h)*initialFlow[i].v,I(tube.center[i]));
        double center=midpoint(probe);I r=I(magnitude(probe-I(center)))*I(1.05)+I(1e-10);
        domain[i]=I(center)+I(-r.hi,r.hi);
    }
    for(int iteration=0;iteration<20;++iteration){
        field=flow(state(domain,false),parameters);bool inside=true;
        for(int i=0;i<8;++i){swept[i]=i<6?inlet[i]+I(0,h)*field[i].v:inlet[i];
            if(i<6)inside=inside && swept[i].lo>domain[i].lo && swept[i].hi<domain[i].hi;}
        if(inside)return true;
        for(int i=0;i<6;++i){
            if(swept[i].lo>domain[i].lo && swept[i].hi<domain[i].hi)continue;
            I both=hull(domain[i],swept[i]);double center=midpoint(both);
            I r=I(magnitude(both-I(center)))*I(1.1)+I(1e-10);domain[i]=I(center)+I(-r.hi,r.hi);
        }
    }return false;
}
Tube propagate(const Tube& prior,const Box& domain,double duration,const double* parameters,Flow& residual,Profile& profile){
    auto mark=Clock::now();
    Flow atCenter=flow(state(pointBox(prior.center)),parameters);
    std::array<J,8> directions;
    for(int j=0;j<8;++j)directions[j]=J(domain[j],domain[j]-I(prior.center[j]));
    auto curvature=flow(directions,parameters);
    Matrix f{};Box offset{};
    for(int i=0;i<6;++i){
        I intercept=atCenter[i].v;
        for(int j=0;j<8;++j){f[i][j]=I(midpoint(atCenter[i].d[j]));intercept=intercept-f[i][j]*I(prior.center[j]);}
        offset[i]=I(midpoint(intercept));
        I value=atCenter[i].v-offset[i];
        for(int j=0;j<8;++j)value=value-f[i][j]*I(prior.center[j]);
        for(int j=0;j<8;++j)value=value+(atCenter[i].d[j]-f[i][j])*(domain[j]-I(prior.center[j]));
        residual[i]=D(value+I(.5)*curvature[i].second);
    }
    profile.linearization+=std::chrono::duration<double>(Clock::now()-mark).count();
    mark=Clock::now();Maps transition=maps(f,duration);Tube next=prior;
    profile.matrixMaps+=std::chrono::duration<double>(Clock::now()-mark).count();
    mark=Clock::now();next.center=rk4(prior.center,duration,parameters);
    profile.nominal+=std::chrono::duration<double>(Clock::now()-mark).count();
    Timer endpointTimer(profile.endpoint);
    Box linearCenter=multiply(transition.transition,pointBox(prior.center));
    Box bias=multiply(transition.integral,offset),remainder=multiply(transition.transition,prior.remainder);
    Box residualRadius{};double maximum=0;
    for(int i=0;i<6;++i){residualRadius[i]=I(magnitude(residual[i].v));maximum=std::max(maximum,residualRadius[i].hi);}
    Box integrated=multiply(transition.absoluteIntegral,residualRadius);
    I tail=I(duration)*I(transition.tail)*I(maximum);
    for(int i=0;i<6;++i){
        I correction;
        for(size_t g=0;g<prior.generators[0].size();++g){
            I coefficient;for(int j=0;j<8;++j)coefficient=coefficient+transition.transition[i][j]*I(prior.generators[j][g]);
            next.generators[i][g]=midpoint(coefficient);
            I error=coefficient-I(next.generators[i][g]);double radius=magnitude(error);
            correction=correction+I(-radius,radius);
        }
        double radius=(integrated[i]+tail).hi;
        next.remainder[i]=linearCenter[i]+bias[i]-I(next.center[i])+remainder[i]+correction+I(-radius,radius);
    }
    // Preserve held-command center, every shared generator and remainder
    // verbatim. There is no feedback evaluation at certification boundaries.
    return next;
}
mxArray* boxArray(const Box& box,int rows=8){mxArray* a=mxCreateDoubleMatrix(rows,2,mxREAL);double* x=mxGetDoubles(a);
    for(int i=0;i<rows;++i){x[i]=box[i].lo;x[i+rows]=box[i].hi;}return a;}
mxArray* generatorsArray(const Generators& g){mxArray* a=mxCreateDoubleMatrix(8,g[0].size(),mxREAL);double* x=mxGetDoubles(a);
    for(int i=0;i<8;++i)for(size_t j=0;j<g[0].size();++j)x[i+8*j]=g[i][j];
    return a;}
mxArray* centerArray(const Point& p){mxArray* a=mxCreateDoubleMatrix(8,1,mxREAL);std::copy(p.begin(),p.end(),mxGetDoubles(a));return a;}
void mexFunction(int nlhs,mxArray* out[],int nrhs,const mxArray* in[]){
    try{
        Profile profile;auto started=Clock::now();
        if(nrhs!=10 || nlhs!=1)throw std::runtime_error("Ten arguments and one output are required.");
        const double* lower=vector(in[0],6);const double* upper=vector(in[1],6);const double* nominal=vector(in[2],8);
        const double* gain=vector(in[3],12);const double* noise=vector(in[4],6);const double* p=vector(in[5],17);
        validateParameters(p);
        const double* timing=vector(in[6],6);const double* previous=vector(in[7],2);
        if(mxGetM(in[3])!=2 || mxGetN(in[3])!=6)throw std::runtime_error("Gain must be 2 by 6.");
        if(!mxIsDouble(in[8])||mxIsComplex(in[8])||mxIsSparse(in[8])||mxGetNumberOfElements(in[8])!=6)throw std::runtime_error("Execution bounds require six real doubles.");
        const double* execution=mxGetDoubles(in[8]);
        for(int j=0;j<6;++j)if(std::isnan(execution[j]) || (j<4 && !std::isfinite(execution[j])) || (j>=4 && execution[j]<0))throw std::runtime_error("Invalid execution bound.");
        double duration=timing[0],maxStep=timing[1],minStep=timing[2],maxCells=timing[3],maxGenerators=timing[4];
        if(maxGenerators<20 || maxGenerators>4096 || maxGenerators!=std::floor(maxGenerators))
            throw std::runtime_error("Generator budget must be an integer between 20 and 4096.");
        if(duration<=0 || minStep<=0 || maxStep<minStep || maxCells<1 || maxCells!=std::floor(maxCells) || maxCells>100000)
            throw std::runtime_error("Invalid sampling/subdivision budget.");
        for(int j=0;j<6;++j)if(noise[j]<0)throw std::runtime_error("Measurement radii must be nonnegative.");
        Tube inlet=inletTube(lower,upper,previous,in[9],static_cast<size_t>(maxGenerators));Box slew;
        Tube tube=feedbackTube(inlet,nominal,gain,noise,slew);Box initial=outer(tube);
        bool accepted=true;std::string reason="complete";std::vector<Cell> cells;int rejected=0;double time=0;
        for(int u=0;u<2;++u){
            I command=initial[6+u];I difference=slew[u];
            bool inputOk=command.lo>=execution[u] && command.hi<=execution[2+u];
            bool slewOk=std::isinf(execution[4+u]) || magnitude(difference)<=(I(duration)*I(execution[4+u])).lo;
            if(!inputOk || !slewOk){accepted=false;reason="executionBounds";}
        }
        auto timedOut=[&](){return std::chrono::duration<double>(Clock::now()-started).count()>=timing[5];};
        if(timing[5]<=0)throw std::runtime_error("Computation budget must be positive.");
        while(accepted && time<duration){
            if(timedOut()){accepted=false;reason="timeBudget";break;}
            if(tube.generators[0].size()>4090){accepted=false;reason="generatorBudget";break;}
            if(cells.size()>=static_cast<size_t>(maxCells)){accepted=false;reason="cellBudget";break;}
            double end=std::min(duration,time+maxStep);bool stepAccepted=false;
            if(end<=time){accepted=false;reason="minimumCellDuration";break;}
            while(!stepAccepted){
                if(timedOut()){accepted=false;reason="timeBudget";break;}
                I elapsed=I(end)-I(time);Box domain,swept;Flow residual,field;Tube candidate;
                try{
                    bool enclosed;
                    {Timer timer(profile.enclosure);enclosed=enclose(tube,elapsed.hi,p,domain,swept,field);}
                    if(enclosed){
                        // Integrate to an outward-rounded duration. A final
                        // endpoint enclosure is widened below for the tiny
                        // difference from the exact serialized time interval.
                        candidate=propagate(tube,domain,elapsed.hi,p,residual,profile);
                        I gap=I(0,(I(elapsed.hi)-I(elapsed.lo)).hi);
                        for(int i=0;i<6;++i)candidate.remainder[i]=candidate.remainder[i]-gap*field[i].v;
                        // Fresh local remainder directions are propagated with
                        // their signs in later cells instead of repeatedly boxed.
                        promoteRemainder(candidate,6);
                        Box endpoint=outer(candidate);bool tight=true;
                        for(int i=0;i<6;++i)tight=tight && endpoint[i].lo>=domain[i].lo && endpoint[i].hi<=domain[i].hi;
                        if(tight){cells.push_back({time,end,domain,swept,endpoint,residual,candidate});
                            tube=candidate;time=end;stepAccepted=true;}
                    }
                }catch(const std::runtime_error&){ /* An invalid candidate domain requires refinement, never acceptance. */ }
                if(!stepAccepted){
                    ++rejected;double next=time+(end-time)/2;
                    if(next<=time || (I(next)-I(time)).lo<minStep){accepted=false;reason="minimumCellDuration";break;}
                    end=next;
                }
            }
        }
        const char* fields[]={"accepted","reason","sampleTime","verifiedThrough","rejectedCells","initialBox","endpoint", "center","generators","remainder","cells","timingSeconds","inputChange"};
        out[0]=mxCreateStructMatrix(1,1,13,fields);
        mxSetField(out[0],0,"accepted",mxCreateLogicalScalar(accepted && time==duration));
        mxSetField(out[0],0,"reason",mxCreateString(reason.c_str()));mxSetField(out[0],0,"sampleTime",mxCreateDoubleScalar(duration));
        mxSetField(out[0],0,"verifiedThrough",mxCreateDoubleScalar(time));mxSetField(out[0],0,"rejectedCells",mxCreateDoubleScalar(rejected));
        mxSetField(out[0],0,"inputChange",boxArray(slew,2));
        mxSetField(out[0],0,"initialBox",boxArray(initial));mxSetField(out[0],0,"endpoint",boxArray(outer(tube)));
        mxSetField(out[0],0,"center",centerArray(tube.center));mxSetField(out[0],0,"generators",generatorsArray(tube.generators));mxSetField(out[0],0,"remainder",boxArray(tube.remainder));
        const char* names[]={"start","end","domain","swept","endpoint","residual","center","generators","remainder"};
        mxArray* list=mxCreateStructMatrix(cells.size(),1,9,names);
        for(size_t k=0;k<cells.size();++k){const Cell& cell=cells[k];
            mxSetField(list,k,"start",mxCreateDoubleScalar(cell.start));mxSetField(list,k,"end",mxCreateDoubleScalar(cell.end));
            mxSetField(list,k,"domain",boxArray(cell.domain));mxSetField(list,k,"swept",boxArray(cell.swept));mxSetField(list,k,"endpoint",boxArray(cell.endpoint));
            mxSetField(list,k,"residual",intervals(cell.residual));mxSetField(list,k,"center",centerArray(cell.tube.center));
            mxSetField(list,k,"generators",generatorsArray(cell.tube.generators));mxSetField(list,k,"remainder",boxArray(cell.tube.remainder));
        }mxSetField(out[0],0,"cells",list);
        const char* timingNames[]={"enclosure","linearization","matrixMaps","nominal","endpoint","total"};
        mxArray* timingOutput=mxCreateStructMatrix(1,1,6,timingNames);
        double measurements[]={profile.enclosure,profile.linearization,profile.matrixMaps,profile.nominal,profile.endpoint,
            std::chrono::duration<double>(Clock::now()-started).count()};
        for(int j=0;j<6;++j)mxSetField(timingOutput,0,timingNames[j],mxCreateDoubleScalar(measurements[j]));
        mxSetField(out[0],0,"timingSeconds",timingOutput);
    }catch(const std::exception& e){mexErrMsgIdAndTxt("collisionAvoidanceController:invalidFeedbackTube","%s",e.what());}
}
