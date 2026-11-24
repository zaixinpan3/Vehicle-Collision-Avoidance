#include "planner.hpp"
#include <numeric>
namespace pc {
namespace {
struct Eval {
    Vec values;
    M jac;
    bool feasible = true;
    double stability = 0, tracking = 0;
};
struct Gap {
    double value;
    Eigen::RowVectorXd gradient;
};
struct Context {
    const Config &c;
    const Curve &curve;
    const std::vector<Forecast> &members;
    U previous;
    int N, nominal;
    double cap, floor, budget;
    Clock::time_point start;
    M map, offset;
    Vec cached, output, bestPoint, domainPoint;
    bool geometryPassed = false, domainPassed = false;
    M derivative;
    double transition = 0, bestResidual = inf;
    bool merit = true, preparing = false;
    Plan best;
    std::string chosen;
    bool timedOut() const {
        return elapsed(start) >= budget;
    }
    bool optimal() const {
        return best.inputs.size() && best.slacks.sum() <= floor + roundoff();
    }
    double roundoff() const {
        return 128 * std::numeric_limits<double>::epsilon() * N * std::max(1., c.margin);
    }
    Eigen::RowVector3d reference(const X &initial, int j) const {
        U o(curve.origin), p = initial.head<2>() + direction(initial[2]) * ((j + 1) * c.h * c.speed);
        U ref;
        double angle = curve.heading, k = curve.curvature;
        if (k == 0) {
            U axis = direction(angle);
            ref = o + axis * axis.dot(p - o);
        } else {
            U center = o + rotate(U(0, 1), angle) / k;
            U radial = p - center;
            angle = std::atan2(k * radial[0], -k * radial[1]);
            ref = center - rotate(U(0, 1), angle) / k;
        }
        return {ref.x(), ref.y(), angle};
    }
    Eval evaluate(const M &inputs, bool wantJac) const;
    void consider(const M &inputs, const Eval &e, const char *source) {
        const Vec slacks = e.values.head(N).cwiseMax(0.);
        double loss = slacks.sum();
        if (!e.feasible || !e.values.allFinite() || !std::isfinite(e.stability + e.tracking) ||
            loss > cap + roundoff() ||
            (best.inputs.size() &&
             (loss > best.slacks.sum() || (loss == best.slacks.sum() && (e.stability >= best.stability ||
                                                                         e.tracking > best.tracking + c.tol))))) {
            return;
        }
        best = {inputs, slacks, e.tracking, e.stability};
        chosen = source;
    }
    int callback(const double *knots, double *dest, bool gradient);
};
static M nextSensitivity(const Eigen::Matrix<double, 6, 8> &local, const M &state, const M &input) {
    return local.leftCols<6>() * state + local.rightCols<2>() * input;
}
Eval Context::evaluate(const M &inputs, bool wantJac) const {
    const int nu = wantJac ? map.cols() : 0, rows = 2 * N + members.size();
    Eval e{Vec::Constant(rows, -inf), M::Zero(rows, nu)};
    U prev = previous;
    for (int i = 0; i < N; i++) {
        U input = inputs.col(i);
        if (!input.allFinite() || (input - c.limit(input, prev)).lpNorm<Eigen::Infinity>() > c.tol) {
            e.feasible = false;
        }
        double ratio = input[1];
        e.stability += c.h * .5 * (ratio * ratio + std::pow(ratio, 4));
        e.tracking += .5 * c.inputWeight * (input - prev).squaredNorm();
        prev = input;
    }
    for (size_t s = 0; s < members.size(); s++) {
        const auto &m = members[s];
        int terminalRow = 2 * N + s;
        X x = m.x, domainStart = x;
        M S = M::Zero(6, nu), dense(6, c.K), segment(6, c.K + 1);
        M dInput = M::Zero(2, nu);
        bool remaining = true;
        auto gapAt = [&](int node) -> Gap {
            Gap result{inf, Eigen::RowVectorXd::Zero(nu)};
            if (!m.target || (m.covered && !remaining)) {
                return result;
            }
            Q q = m.boxes.col(node);
            if (!node) {
                q = m.root;
                q.segment<2>(7) = m.rootHalf;
            }
            V<3> g;
            pcbfNodeClearance(x.data(), q.data(), c.shape.data(), &result.value, g.data());
            if (wantJac) {
                result.gradient = g.transpose() * S.topRows(3);
            }
            if (m.covered && m.range(x, node) > c.R + m.radius) {
                remaining = false;
            }
            if (!(result.value > 0)) {
                e.feasible = false;
            }
            return result;
        };
        Gap first = gapAt(0);
        auto maxRow = [&](int row, double v, const Eigen::RowVectorXd &g) {
            if (v > e.values[row]) {
                e.values[row] = v;
                if (wantJac) {
                    e.jac.row(row) = g;
                }
            }
        };
        for (int i = 0; i <= N; i++) {
            bool bridge = i == N;
            if (bridge && (!m.target || (m.covered ? !remaining : m.range(x, i) > c.R + m.radius))) {
                e.values[terminalRow] = -c.R;
                break;
            }
            U u = U::Zero();
            if (!bridge) {
                u = inputs.col(i);
            }
            if (wantJac) {
                if (bridge) {
                    dInput.setZero();
                } else {
                    dInput = map.middleRows(2 * i, 2);
                }
            }
            X root = x;
            x = hold(root, u, m.scales, dense);
            require(x.allFinite() && dense.allFinite(), "Nonfinite physical rollout");
            segment.col(0) = bridge ? root : domainStart;
            segment.rightCols(c.K) = dense;
            domainStart = dense.rightCols(1);
            V<11> domain;
            pcbfHoldDomain(segment.data(), u.data(), domain.data());
            Eigen::RowVectorXd domainGradient = Eigen::RowVectorXd::Zero(nu);
            if (wantJac) {
                const double *args[std::max(pcbf_sensitivity_SZ_ARG, pcbf_exact_tangent_SZ_ARG)] = {
                    root.data(), u.data(), m.scales.data(), domain.data() + 1};
                Eigen::Matrix<double, 6, 8> E, D;
                double *res[std::max(pcbf_sensitivity_SZ_RES, pcbf_exact_tangent_SZ_RES)] = {E.data(), D.data()};
                double w[std::max(pcbf_sensitivity_SZ_W, pcbf_exact_tangent_SZ_W)];
                auto tangent = merit ? pcbf_sensitivity : pcbf_exact_tangent;
                casadi_int indices[std::max(1, pcbf_exact_tangent_SZ_IW)];
                require(!tangent(args, res, indices, w, 0), "Sensitivity evaluation failed");
                M selected = domain[1] == 0 ? S : nextSensitivity(D, S, dInput);
                domainGradient =
                    domain.segment<6>(2).transpose() * selected + domain.segment<2>(8).transpose() * dInput;
                S = nextSensitivity(E, S, dInput);
            }
            if (!domain[10] || domain[0] > c.tol) {
                e.feasible = false;
            }
            Gap second = gapAt(i + 1);
            const Gap &limiting = first.value <= second.value ? first : second;
            double stage = std::isfinite(limiting.value) ? c.margin - limiting.value : -1.;
            if (!bridge) {
                maxRow(i, stage, -limiting.gradient);
                maxRow(N + i, domain[0], domainGradient);
                double slip = std::atan2(x[4] - c.lr * x[5], std::hypot(x[3], c.speedFloor)) /
                              (c.slipScale * m.scales[3] / m.scales[1]);
                e.stability += .5 * c.h * slip * slip / members.size();
                if (static_cast<int>(s) == nominal) {
                    auto r = reference(m.x, i);
                    V<5> err;
                    err << rotate(x.head<2>() - r.head<2>().transpose(), -r[2]).y(),
                        std::remainder(x[2] - r[2], 2 * std::acos(-1.)), x[3] - c.speed, x[4], x[5];
                    e.tracking += .5 * c.h * c.weights.dot(err.cwiseAbs2());
                }
            } else {
                double core = -c.R, steps;
                U normal;
                X g = X::Zero();
                if (!m.covered) {
                    pcbfTerminalKnown(x.data(), m.boxes.col(i + 1).data(), m.radius, &core, normal.data(),
                                      g.data(), &steps);
                } else if (remaining) {
                    auto z = m.centers.col(i + 1);
                    double radius = m.radius + std::hypot(std::max(std::abs(z[0]), std::abs(z[1])),
                                                          std::max(std::abs(z[2]), std::abs(z[3])));
                    pcbfTerminalSector(x.data(), m.boxes.col(i + 1).data(), m.speeds.data(), m.course, m.half,
                                       radius, &core, normal.data(), g.data(), &steps);
                }
                maxRow(terminalRow, core, g.transpose() * S);
                maxRow(terminalRow, stage, -limiting.gradient);
                maxRow(terminalRow, domain[0], domainGradient);
                for (int a = 0; a < 2; a++) {
                    Eigen::RowVectorXd ds = Eigen::RowVectorXd::Zero(nu);
                    if (wantJac) {
                        ds = (inputs(a, N - 1) >= 0 ? 1. : -1.) * map.row(2 * (N - 1) + a);
                    }
                    maxRow(terminalRow, std::abs(inputs(a, N - 1)) - c.h * c.rates[a], ds);
                }
                if (!(core < c.margin) || e.values[terminalRow] > c.tol) {
                    e.feasible = false;
                }
            }
            first = std::move(second);
        }
    }
    return e;
}

int Context::callback(const double *knots, double *dest, bool gradient) {
    if (timedOut()) {
        return 1;
    }
    Vec point = Eigen::Map<const Vec>(knots, map.cols());
    bool same = cached.size() == point.size() && (cached == point);
    if (!same || (gradient && !derivative.size())) {
        Vec flat = map * point + Eigen::Map<const Vec>(offset.data(), offset.size());
        M inputs = Eigen::Map<const M>(flat.data(), 2, N);
        Eval e = evaluate(inputs, gradient);
        if (!same) {
            consider(inputs, e, "ipopt");
        }
        geometryPassed = e.values.head(N).maxCoeff() <= 0 && e.values.tail(members.size()).maxCoeff() <= c.tol;
        domainPassed = e.values.segment(N, N).maxCoeff() <= 0;
        Vec positive = e.values.cwiseMax(0.);
        double residual = positive.squaredNorm();
        if (residual < bestResidual) {
            bestResidual = residual;
            bestPoint = point;
        }
        if (merit) {
            Vec scale = (positive.array().square() + 1.).sqrt();
            double mass = (positive.array().square() / (1 + scale.array())).sum();
            double join = std::hypot(transition, c.tol), slope = 1,
                   value = mass + .5 * transition * transition - c.tol * transition * transition / (join + c.tol);
            if (2 * mass < transition * transition) {
                double root = std::sqrt(2 * mass + c.tol * c.tol);
                value = join * 2 * mass / (root + c.tol);
                slope = join / root;
            }
            output = Vec::Constant(1, value);
            if (gradient) {
                derivative = (positive.array() / scale.array()).matrix().transpose() * e.jac * slope;
            }
        } else {
            output = e.values;
            if (gradient) {
                derivative = std::move(e.jac);
            }
        }
        if (!gradient) {
            derivative.resize(0, 0);
        }
        cached = point;
    }
    if (gradient) {
        std::copy_n(derivative.data(), derivative.size(), dest);
    } else {
        std::copy_n(output.data(), output.size(), dest);
    }
    return 0;
}
std::vector<int> boundaries(int avoid, int N, const Config &c) {
    int nodes = std::min(avoid, c.knots), near = std::min(nodes - 1, c.near);
    std::vector<int> out{0};
    for (int j = 1; j <= near; j++) {
        out.push_back(j);
    }
    for (int j = 1; j <= nodes - near; j++) {
        out.push_back(near + std::lround(double(j) * (avoid - near) / (nodes - near)));
    }
    int width = 1;
    for (size_t j = 1; j < out.size(); j++) {
        width = std::max(width, out[j] - out[j - 1]);
    }
    int extra = N - avoid, n = std::min(c.knots, (extra + width - 1) / width);
    for (int j = 1; j <= n; j++) {
        out.push_back(avoid + std::lround(double(j) * extra / n));
    }
    return out;
}
void optimize(Context &ctx, const M &seed, const std::vector<int> &ends, int &attempts) {
    const Config &c = ctx.c;
    int n = ends.size() - 1, nu = 2 * n;
    M &map = ctx.map;
    map = M::Zero(2 * ctx.N, nu);
    Vec x0(nu), lo(nu), hi(nu);
    const auto &root = ctx.members[ctx.nominal].x;
    U scale{std::min(c.upper[0], std::atan2(c.wb * c.grip, root.segment<2>(3).squaredNorm())), 1};
    for (int j = 0; j < n; j++) {
        x0.segment<2>(2 * j) = seed.col(ends[j + 1] - 1).cwiseQuotient(scale);
        lo.segment<2>(2 * j) = c.lower.cwiseQuotient(scale);
        hi.segment<2>(2 * j) = c.upper.cwiseQuotient(scale);
        for (int i = ends[j]; i < ends[j + 1]; i++) {
            double f = double(i - ends[j] + 1) / (ends[j + 1] - ends[j]);
            for (int a = 0; a < 2; a++) {
                map(2 * i + a, 2 * j + a) = f * scale[a];
                if (j) {
                    map(2 * i + a, 2 * j - 2 + a) = (1 - f) * scale[a];
                }
            }
        }
    }
    Vec bias = Eigen::Map<const Vec>(seed.data(), seed.size()) - map * x0;
    ctx.offset = Eigen::Map<const M>(bias.data(), 2, ctx.N);
    std::vector<Eigen::RowVectorXd> rows;
    std::vector<double> bounds;
    auto row = [&](Eigen::RowVectorXd r, double b) {
        rows.push_back(std::move(r));
        bounds.push_back(b);
    };
    for (int i = 0; i < ctx.N; i++) {
        for (int a = 0; a < 2; a++) {
            int k = 2 * i + a;
            double missing = 1. - map.row(k).sum() / scale[a];
            if (bias[k] > missing * c.upper[a] + 1e-15) {
                row(map.row(k), c.upper[a] - bias[k]);
            }
            if (bias[k] < missing * c.lower[a] - 1e-15) {
                row(-map.row(k), bias[k] - c.lower[a]);
            }
            if (std::isfinite(c.rates[a])) {
                Eigen::RowVectorXd d = map.row(k);
                if (i) {
                    d -= map.row(k - 2);
                }
                double off = bias[k] - (i ? bias[k - 2] : ctx.previous[a]);
                row(d, c.h * c.rates[a] - off);
                row(-d, c.h * c.rates[a] + off);
            }
        }
    }
    M A(rows.size(), nu);
    for (size_t i = 0; i < rows.size(); i++) {
        A.row(i) = rows[i];
    }
    pcbf_ipopt_problem p{};
    p.knots = nu;
    p.stages = ctx.N;
    p.members = ctx.members.size();
    p.input_rows = A.rows();
    p.max_iterations = c.iterations;
    p.prior_cap = ctx.cap;
    p.clearance = c.margin;
    p.tolerance = c.tol;
    p.mu_init = 1e-6;
    p.lower = lo.data();
    p.upper = hi.data();
    p.input_matrix = A.data();
    p.input_upper = bounds.data();
    p.context = &ctx;
    p.values = [](void *o, const double *x, double *v) {
        return static_cast<Context *>(o)->callback(x, v, false);
    };
    p.jacobian = [](void *o, const double *x, double *v) {
        return static_cast<Context *>(o)->callback(x, v, true);
    };
    p.stop = [](void *o, const double *raw) -> int {
        auto &w = *static_cast<Context *>(o);
        if (w.preparing) {
            return 0;
        }
        if (w.optimal() || w.timedOut()) {
            return 1;
        }
        if (!w.merit) {
            return 0;
        }
        Vec x = Eigen::Map<const Vec>(raw, w.map.cols());
        if (w.cached.size() != x.size() || w.cached != x) {
            w.domainPoint.resize(0);
            return 0;
        }
        if (!w.geometryPassed && w.domainPassed) {
            w.domainPoint = x;
            return 0;
        }
        if (w.domainPoint.size() && w.geometryPassed && !w.domainPassed) {
            Vec mid = w.map * ((w.domainPoint + x) * .5) + Eigen::Map<const Vec>(w.offset.data(), w.offset.size());
            M inputs = Eigen::Map<const M>(mid.data(), 2, w.N);
            w.consider(inputs, w.evaluate(inputs, false), "ipoptMidpoint");
        }
        w.domainPoint.resize(0);
        return w.optimal();
    };
    Eval first = ctx.evaluate(seed, false);
    Vec equil = Vec::Ones(nu);
    bool clearanceSearch =
        first.values.head(ctx.N).maxCoeff() > 0 && first.values.tail(first.values.size() - ctx.N).maxCoeff() <= 0;
    if (clearanceSearch) {
        ctx.transition = c.margin;
        first = ctx.evaluate(seed, true);
        for (int i = 0; i < ctx.N; i++) {
            if (first.values[i] > 0) {
                equil.array() += first.jac.row(i).transpose().array().square() / std::hypot(1., first.values[i]);
            }
        }
        equil = equil.cwiseSqrt();
        x0.array() *= equil.array();
        lo.array() *= equil.array();
        hi.array() *= equil.array();
        map.array().rowwise() /= equil.transpose().array();
        A.array().rowwise() /= equil.transpose().array();
    } else if (first.values.tail(first.values.size() - ctx.N).maxCoeff() > c.tol) {
        double distance = std::min((x0 - lo).minCoeff(), (hi - x0).minCoeff());
        for (int j = 0; j < A.rows(); j++) {
            if (A.row(j).norm() > 0) {
                distance = std::min(distance, (bounds[j] - A.row(j).dot(x0)) / A.row(j).norm());
            }
        }
        p.mu_init = std::clamp(std::pow(std::max(0., distance), 2), 1e-6, .1);
    }
    Vec trial(nu);
    for (bool seek : {true, false}) {
        if (ctx.optimal() || ctx.timedOut()) {
            break;
        }
        ctx.merit = seek;
        p.seek_zero = seek;
        ctx.cached.resize(0);
        ctx.derivative.resize(0, 0);
        if (!seek) {
            if (ctx.bestPoint.size()) {
                x0 = ctx.bestPoint;
            }
            p.mu_init = 1e-6;
            x0.array() /= equil.array();
            lo.array() /= equil.array();
            hi.array() /= equil.array();
            map.array().rowwise() *= equil.transpose().array();
            A.array().rowwise() *= equil.transpose().array();
        }
        p.start = x0.data();
        p.wall_seconds = std::max(.001, ctx.budget - elapsed(ctx.start));
        char status[512];
        attempts++;
        int rc = pcbf_ipopt_trial(&p, trial.data(), status, sizeof(status));
        if (rc < 0 && !ctx.best.inputs.size() && !ctx.timedOut()) {
            throw std::runtime_error(status);
        }
        if (rc >= 0 && trial.allFinite() && !ctx.timedOut()) {
            Vec buf(seek ? 1 : 2 * ctx.N + p.members);
            ctx.callback(trial.data(), buf.data(), false);
        }
    }
}
} // namespace
int warmup(const Config &c) {
    Member m;
    m.ego << 0, 0, 0, c.speed, 0, 0;
    m.scales.setOnes();
    m.hasTarget = true;
    m.target << c.R / 2, c.width + c.referenceWidth, 0, 0, 0, 0, c.targetAxle, c.referenceLength / 2,
        c.referenceWidth / 2, 0, 0;
    int size = 11;
    double avoid, connect;
    pcbfMemberPreview(m.ego.data(), m.target.data(), &size, &avoid, &connect);
    int N = int(avoid + connect);
    Curve curve{{0, 0}, 0, 0, 1000};
    std::vector<Forecast> fc;
    fc.emplace_back(m, true, N, c);
    Context ctx{c, curve, fc, U::Zero(), N, 0, inf, 0, inf, Clock::now()};
    M seed = guide(fc.front(), U::Zero(), curve, U::Zero(), int(avoid), N, c);
    ctx.preparing = true;
    int attempts = 0;
    optimize(ctx, seed, boundaries(int(avoid), N, c), attempts);
    require(ctx.best.inputs.size() > 0, "IPOPT warmup did not produce a validated private plan");
    return attempts;
}
Decision plan(const std::vector<Member> &members, bool active, bool previousActive, const Plan &prior,
              const U &previous, const Curve &curve, const U &trimOffset, const Config &c,
              Clock::time_point start) {
    auto solveStart = Clock::now();
    bool inherit = prior.inputs.cols() > 0 && !(active && !previousActive);
    int avoid = 0, connect = 0, nominal = 0;
    X mean = X::Zero();
    for (const Member &m : members) {
        int size = active ? 11 : 0;
        double a, b;
        pcbfMemberPreview(m.ego.data(), m.target.data(), &size, &a, &b);
        avoid = std::max(avoid, int(a));
        connect = std::max(connect, int(b));
        mean += m.ego;
    }
    int suffix = inherit ? prior.inputs.cols() - 1 : 0, N = std::max(suffix, avoid + connect);
    require(N >= 1 && N <= c.maxSteps, "Calculated window exceeds capacity");
    avoid = N - connect;
    mean /= members.size();
    double nearest = inf;
    std::vector<Forecast> predictions;
    double floor = 0, cap = inherit ? prior.slacks.tail(suffix).sum() : inf;
    for (const auto &m : members) {
        double error = (m.ego - mean).squaredNorm();
        if (error < nearest) {
            nominal = predictions.size();
            nearest = error;
        }
        predictions.emplace_back(m, active, N, c);
        if (active) {
            auto &f = predictions.back();
            Q root = f.root;
            root.segment<2>(7) = f.rootHalf;
            double gap;
            V<3> grad;
            pcbfNodeClearance(m.ego.data(), root.data(), c.shape.data(), &gap, grad.data());
            require(gap > 0, "Current estimated footprints overlap");
            floor = std::max(floor, c.margin - gap);
        }
    }
    Context ctx{
        c, curve, predictions, previous, N, nominal, cap, floor, inherit ? c.deadline : c.initialDeadline, start};
    require(floor <= cap + ctx.roundoff(), "Current node exceeds inherited PCBF deficit");
    M shifted;
    auto check = [&](const M &u, const char *source) { ctx.consider(u, ctx.evaluate(u, false), source); };
    if (inherit) {
        shifted.resize(2, N);
        shifted.leftCols(suffix) = prior.inputs.rightCols(suffix);
        U prev = prior.inputs.rightCols(1), tail = active ? U::Zero() : c.trim;
        for (int j = suffix; j < N; j++) {
            prev = c.limit(tail, prev);
            shifted.col(j) = prev;
        }
        M proposal = shifted;
        proposal.col(0) = guide(predictions[nominal], previous, curve, trimOffset, 1, 1, c);
        prev = previous;
        for (int j = 0; j < N; j++) {
            prev = c.limit(proposal.col(j), prev);
            proposal.col(j) = prev;
        }
        if (proposal != shifted) {
            check(proposal, "guideWithDrivingSuffix");
        }
        if (!ctx.optimal() || (active && !ctx.timedOut())) {
            check(shifted, "shiftedContinuation");
        }
    }
    int attempts = 0;
    if (!ctx.optimal()) {
        M seed =
            inherit && active ? shifted : guide(predictions[nominal], previous, curve, trimOffset, avoid, N, c);
        check(seed, active ? "field" : "cruise");
        if (!ctx.optimal()) {
            optimize(ctx, seed, boundaries(avoid, N, c), attempts);
        }
    }
    require(ctx.best.inputs.size() > 0, "No validated PCBF plan for the current sensor block");
    return {std::move(ctx.best), ctx.chosen, attempts, N, avoid, elapsed(solveStart)};
}
} // namespace pc
