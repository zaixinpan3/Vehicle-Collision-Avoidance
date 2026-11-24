#include "model.hpp"
namespace pc {
static U arcSpeed(double v, double a, double t) {
    if (a < 0) {
        t = std::min(t, -v / a);
    }
    return {v * t + .5 * a * t * t, std::max(0., v + a * t)};
}
Forecast::Forecast(const Member &m, bool active, int N, const Config &c)
    : x(m.ego), scales(m.scales), target(active && m.hasTarget), root(m.target) {
    if (!target) {
        return;
    }
    rootHalf = root.segment<2>(7);
    const auto &e = m.estimate;
    if (e.initialized) {
        auto b = Eigen::Map<const Eigen::Matrix2d>(root[3] == 0 && e.rest_compatible ? e.rest_bounds
                                                                                     : e.position_bounds);
        root.head<2>() = .5 * (b.col(0) + b.col(1));
        U span = (.5 * (b.col(1) - b.col(0))).cwiseMax(0.);
        radius = span.norm();
        double cs = std::abs(std::cos(root[2])), sn = std::abs(std::sin(root[2]));
        rootHalf += U(cs * span[0] + sn * span[1], sn * span[0] + cs * span[1]);
        if (root[3] > 0) {
            root.segment<2>(7).array() += radius;
        } else {
            root.segment<2>(7) = rootHalf;
        }
    }
    auto limits = Eigen::Map<const Eigen::Matrix2d>(e.motion_bounds);
    if (root[3] > 0 && e.bounds_available) {
        root.segment<2>(4) =
            root.segment<2>(4).cwiseMax(limits.row(0).transpose()).cwiseMin(limits.row(1).transpose());
        covered = (limits.row(0) != limits.row(1));
    }
    if (!covered) {
        boxes.resize(11, N + 2);
        for (int i = 0; i < N + 2; i++) {
            pcbfTargetAt(root.data(), i * c.h, boxes.col(i).data());
        }
        return;
    }
    M t = Eigen::RowVectorXd::LinSpaced(N + 2, 0., (N + 1) * c.h);
    auto times = view(t), b = array(), ce = array();
    pcbfTargetEnvelope(root.data(), limits.data(), times.get(), b.get(), ce.get());
    boxes = copy(b);
    centers = copy(ce);
    double end = t(N + 1);
    U slow = arcSpeed(root[3], limits(0, 0), end), fast = arcSpeed(root[3], limits(1, 0), end);
    U k = limits.col(1).array().sin() / root[6];
    V<4> turn;
    turn << k * slow[0], k * fast[0];
    double lo = std::clamp(turn.minCoeff(), -end * c.yawMax, end * c.yawMax) + limits(0, 1);
    double hi = std::clamp(turn.maxCoeff(), -end * c.yawMax, end * c.yawMax) + limits(1, 1);
    course = root[2] + .5 * (lo + hi);
    half = std::min(std::acos(-1.), .5 * (hi - lo));
    speeds << std::min(slow[1], c.speedMax), std::min(fast[1], c.speedMax);
}
double Forecast::range(const X &ego, int j) const {
    U d = ego.head<2>() - boxes.col(j).head<2>();
    if (!covered) {
        return d.norm();
    }
    d = rotate(d, -boxes(2, j));
    const auto z = centers.col(j);
    return std::hypot(std::max({0., z[0] - d[0], d[0] - z[1]}), std::max({0., z[2] - d[1], d[1] - z[3]}));
}
U curveTrim(double k, const Config &c) {
    if (k == 0) {
        return U::Zero();
    }
    V<3> point{0, std::atan(c.wb * k), .02}, r;
    U offset;
    for (int i = 0; i < 40; i++) {
        pcbfTrimResidual(point.data(), k, r.data(), offset.data());
        if (r.lpNorm<Eigen::Infinity>() < 1e-12) {
            return offset;
        }
        Eigen::Matrix3d J;
        for (int j = 0; j < 3; j++) {
            V<3> lo = point, hi = point, a, b;
            U unused;
            double h = 1e-6;
            lo[j] -= h;
            hi[j] += h;
            pcbfTrimResidual(lo.data(), k, a.data(), unused.data());
            pcbfTrimResidual(hi.data(), k, b.data(), unused.data());
            J.col(j) = (b - a) / (2 * h);
        }
        V<3> step = J.fullPivLu().solve(-r);
        bool improved = false;
        for (double fraction = 1; fraction >= std::ldexp(1., -20); fraction *= .5) {
            V<3> trial = point + fraction * step, v;
            U unused;
            pcbfTrimResidual(trial.data(), k, v.data(), unused.data());
            if (v.norm() < r.norm()) {
                point = trial;
                improved = true;
                break;
            }
        }
        if (!improved) {
            break;
        }
    }
    throw std::runtime_error("The reference curve has no resolved cruise trim");
}
M guide(const Forecast &m, const U &prior, const Curve &curve, const U &offset, int avoid, int N,
        const Config &c) {
    auto result = array();
    M out(2, N);
    if (!m.target) {
        X ego = m.x;
        U held = prior;
        pcbfCruiseInputs(ego.data(), held.data(), &curve, offset.data(), N, result.get());
        return copy(result);
    }
    M noBoxes(11, 0), noCenters(4, 0);
    auto b = view(m.covered ? m.boxes : noBoxes), z = view(m.covered ? m.centers : noCenters);
    pcbfFieldInputs(m.x.data(), prior.data(), m.root.data(), &curve, b.get(), z.get(), avoid, result.get());
    out.leftCols(avoid) = copy(result);
    out.rightCols(N - avoid).colwise() = c.trim;
    U prev = prior;
    for (int i = 0; i < N; i++) {
        prev = c.limit(out.col(i), prev);
        out.col(i) = prev;
    }
    return out;
}
} // namespace pc
