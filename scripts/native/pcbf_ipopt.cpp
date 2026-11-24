#include "pcbf_ipopt.h"
#include <casadi/casadi.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

using namespace casadi;
namespace {
DM matrix(const double *data, int rows, int columns) {
    DM m = DM::zeros(rows,columns);
    if (rows && columns) std::copy_n(data,rows*columns,m.nonzeros().begin());
    return m;
}
class Numeric : public Callback {
public:
    Numeric(const char *name,const pcbf_ipopt_problem& p,int rows,int cols,
            pcbf_evaluate f,const Numeric *jac=nullptr)
        : p(p),rows(rows),cols(cols),f(f),jac(jac) { construct(name); }
    Sparsity get_sparsity_in(casadi_int) override { return Sparsity::dense(p.knots,1); }
    Sparsity get_sparsity_out(casadi_int) override { return Sparsity::dense(rows,cols); }
    std::vector<DM> eval(const std::vector<DM>& in) const override {
        DM out=DM::zeros(rows,cols);
        if (f(p.context,in[0].ptr(),out.ptr())) throw std::runtime_error("PCBF evaluation failed");
        return {out};
    }
    bool has_jacobian() const override { return jac!=nullptr; }
    Function get_jacobian(const std::string& name,const std::vector<std::string>& in,
                          const std::vector<std::string>& out,const Dict& opts) const override {
        MX x=MX::sym("x",p.knots);
        MX unused=MX::sym("out",Sparsity(rows,cols));
        return Function(name,{x,unused},(*jac)(std::vector<MX>{x}),in,out,opts);
    }
private:
    const pcbf_ipopt_problem& p;
    int rows,cols;
    pcbf_evaluate f;
    const Numeric *jac;
};
class Stop : public Callback {
public:
    Stop(const pcbf_ipopt_problem& p,int constraints):p(p),ng(constraints) { construct("pcbf_stop"); }
    casadi_int get_n_in() override { return 6; }
    Sparsity get_sparsity_in(casadi_int i) override {
        const int sizes[]={p.knots,1,ng,p.knots,ng,0};
        return Sparsity::dense(sizes[i],1);
    }
    std::vector<DM> eval(const std::vector<DM>& arg) const override {
        return {DM(p.stop(p.context,arg[0].ptr())!=0)};
    }
private:
    const pcbf_ipopt_problem& p;
    int ng;
};
}

int pcbf_ipopt_trial(const pcbf_ipopt_problem *problem,double *trial,char *message,size_t capacity)
{
    auto report=[&](const char *s){ if (message && capacity) std::snprintf(message,capacity,"%s",s); };
    if (!problem || !trial) { report("Missing problem or output");return -1; }
    const auto& p=*problem;
    if (p.knots<1 || p.stages<1 || p.members<1 || p.input_rows<0 || !p.values || !p.jacobian ||
        !p.start || !p.lower || !p.upper || (p.input_rows && (!p.input_matrix || !p.input_upper)) ||
        p.max_iterations<1 || !(p.tolerance>0) || !(p.wall_seconds>0) || !(p.mu_init>0) ||
        !(p.prior_cap>=0) || !(p.clearance>=0)) { report("Invalid PCBF problem");return -1; }
    try {
        int rows=p.seek_zero ? 1 : 2*p.stages+p.members;
        Numeric jac("pcbf_derivatives",p,rows,p.knots,p.jacobian);
        Numeric eval("pcbf_values",p,rows,1,p.values,&jac);
        MX x=MX::sym("knots",p.knots),value=eval(std::vector<MX>{x})[0];
        MX g=mtimes(sparsify(matrix(p.input_matrix,p.input_rows,p.knots)),x)
             -matrix(p.input_upper,p.input_rows,1);
        MX objective=value;
        if (!p.seek_zero) {
            MX stage=value(Slice(0,p.stages));
            objective=sum1(fmax(stage,0));
            MX hard=vertcat(value(Slice(p.stages,rows)),g);
            if (p.prior_cap==0) g=vertcat(stage,hard);
            else {
                g=vertcat(mmax(stage)-p.clearance,hard);
                if (std::isfinite(p.prior_cap)) g=vertcat(g,objective-p.prior_cap);
            }
        }
        Stop stop(p,g.numel());
        Dict opts={{"print_time",false},{"error_on_fail",false},{"calc_lam_p",false},
            {"show_eval_warnings",false},{"ipopt.print_level",0},{"ipopt.sb","yes"},
            {"ipopt.max_iter",p.max_iterations},{"ipopt.hessian_approximation","limited-memory"},
            {"ipopt.bound_relax_factor",0.0},{"ipopt.mu_strategy","monotone"},
            {"ipopt.mu_init",p.mu_init},{"ipopt.bound_push",1e-6},{"ipopt.bound_frac",1e-6},
            {"ipopt.tol",p.tolerance},{"ipopt.constr_viol_tol",p.tolerance}};
        if (p.seek_zero) opts["ipopt.alpha_red_factor"]=0.25;
        if (std::isfinite(p.wall_seconds)) opts["ipopt.max_wall_time"]=p.wall_seconds;
        if (p.stop) opts["iteration_callback"]=static_cast<const Function&>(stop);
        auto nlp=nlpsol("compact_pcbf","ipopt",MXDict{{"x",x},{"f",objective},{"g",g}},opts);
        auto result=nlp(DMDict{{"x0",matrix(p.start,p.knots,1)},{"lbx",matrix(p.lower,p.knots,1)},
            {"ubx",matrix(p.upper,p.knots,1)},{"lbg",-DM::inf(g.numel())},{"ubg",DM::zeros(g.numel())}});
        auto& answer=result.at("x").nonzeros();
        std::copy(answer.begin(),answer.end(),trial);
        auto stats=nlp.stats();
        report(stats.at("return_status").to_string().c_str());
        return stats.at("success").to_bool() ? 0 : 1;
    } catch (const std::exception& e) { report(e.what());return -1; }
}
