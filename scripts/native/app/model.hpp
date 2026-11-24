#pragma once
#include <Eigen/Core>
#include <Eigen/LU>
#include <nlohmann/json.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>
#include "nrmm_track.h"
#include "pcbf_ipopt.h"
#include "pcbf_kernels.h"
#include "pcbfCruiseInputs.h"
#include "pcbfFieldInputs.h"
#include "pcbfHoldDomain.h"
#include "pcbfMemberPreview.h"
#include "pcbfNodeClearance.h"
#include "pcbfNodeClearance_emxAPI.h"
#include "pcbfTargetAt.h"
#include "pcbfTargetEnvelope.h"
#include "pcbfTerminalKnown.h"
#include "pcbfTerminalSector.h"
#include "pcbfTrimResidual.h"

namespace pc {
using json = nlohmann::json;
template <int N> using V = Eigen::Matrix<double, N, 1>;
using X = V<6>;
using U = V<2>;
using Q = V<11>;
using M = Eigen::MatrixXd;
using Vec = Eigen::VectorXd;
using Clock = std::chrono::steady_clock;
constexpr double inf = std::numeric_limits<double>::infinity();
inline void require(bool ok, const char *msg) {
    if (!ok) {
        throw std::runtime_error(msg);
    }
}
template <int N> V<N> vec(const json &j) {
    require(j.is_array() && j.size() == N, "Invalid vector length");
    V<N> v;
    for (int i = 0; i < N; i++) {
        v[i] = j.at(i).get<double>();
    }
    require(v.allFinite(), "Nonfinite input");
    return v;
}
inline double bound(const json &value) {
    return value.is_null() ? inf : value.get<double>();
}
struct Config {
    double h, period, speed, R, margin, tol, holdSpeed, lr, length, width, wb, grip, slipScale, speedFloor,
        inputWeight, referenceLength, referenceWidth, targetAxle, yawMax, speedMax, deadline, initialDeadline;
    int K, knots, near, iterations, maxHyp;
    double maxSteps;
    U lower, upper, rates, trim;
    V<4> shape, noise;
    V<5> weights;
    explicit Config(const json &doc) {
        const auto &cfg = doc.at("cfg");
        const auto &f = cfg.at("outputFeedback");
        const auto &car = cfg.at("vehicle");
        const auto &t = cfg.at("target");
        const auto &model = cfg.at("model");
        const auto &ctrl = cfg.at("controller");
        require(!ctrl.at("enforceRoadBoundaries").get<bool>() && model.at("speedMinimum") == 0,
                "Unsupported road or model domain");
        h = ctrl.at("sampleTime");
        period = f.at("sensorPeriod");
        speed = cfg.at("referenceSpeed");
        R = cfg.at("collision").at("encounterRangeMeters");
        margin = f.at("clearanceMeters");
        tol = f.at("nlpTolerance");
        holdSpeed = model.at("brakeHoldSpeed");
        lr = car.at("lr");
        wb = car.at("wheelbase");
        length = car.at("length");
        width = car.at("width");
        shape << length / 2, width / 2, vec<2>(car.at("rectangleOffset"));
        noise = vec<4>(f.at("noiseBounds"));
        lower << -f.at("steeringMaximum").get<double>(),
            std::max(-1., cfg.at("actuation").at("brakingRatioMinimum").get<double>());
        upper << -lower[0], std::min(1., cfg.at("actuation").at("brakingRatioMaximum").get<double>());
        rates << bound(f.at("steeringRateMaximum")), bound(model.at("brakingRatioRateMaximum"));
        trim = vec<2>(doc.at("trim"));
        slipScale = doc.at("rearSlipScale");
        speedFloor = model.at("scheduleSpeedFloor");
        grip = car.at("gravity").get<double>() * vec<2>(cfg.at("tire").at("frictionCoefficient")).minCoeff();
        weights = vec<5>(f.at("trackingWeights"));
        inputWeight = f.at("inputWeight");
        targetAxle = t.at("rearAxleDistance");
        referenceLength = t.at("defaultLength");
        referenceWidth = t.at("defaultWidth");
        const auto &domain = t.at("observerDomain");
        yawMax =
            domain.is_object() && domain.contains("yawRateMaximum") ? bound(domain.at("yawRateMaximum")) : inf;
        speedMax = domain.is_object() && domain.contains("speedMaximum") ? bound(domain.at("speedMaximum")) : inf;
        K = f.at("validationSubsteps");
        knots = f.at("controlNodes");
        near = f.at("nearControlNodes");
        iterations = f.at("nlpMaximumIterations");
        deadline = f.at("maximumSolveSeconds");
        initialDeadline = f.at("initializationSolveSeconds");
        maxSteps = bound(ctrl.at("maximumHorizonSteps"));
        maxHyp = f.at("maximumHypotheses");
    }
    U limit(U u, U prev) const {
        return u.cwiseMin(prev + h * rates).cwiseMax(prev - h * rates).cwiseMin(upper).cwiseMax(lower);
    }
};
using Array = std::unique_ptr<emxArray_real_T, decltype(&emxDestroyArray_real_T)>;
inline Array array() {
    return {emxCreate_real_T(0, 0), emxDestroyArray_real_T};
}
inline Array view(const M &m) {
    return {emxCreateWrapper_real_T(const_cast<double *>(m.data()), m.rows(), m.cols()), emxDestroyArray_real_T};
}
inline M copy(const Array &a) {
    return Eigen::Map<const M>(a->data, a->size[0], a->size[1]);
}
inline double elapsed(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}
struct Member {
    X ego;
    V<4> scales;
    V<7> bias;
    Q target = Q::Zero();
    bool hasTarget = false;
    std::unique_ptr<nrmm_track, decltype(&nrmm_track_destroy)> track{nullptr, nrmm_track_destroy};
    nrmm_estimate estimate{};
};
struct Forecast {
    X x;
    V<4> scales;
    bool target = false, covered = false;
    double radius = 0, course = 0, half = 0;
    U speeds = U::Zero(), rootHalf = U::Zero();
    Q root = Q::Zero();
    M boxes, centers;
    Forecast(const Member &m, bool active, int N, const Config &c);
    double range(const X &ego, int j) const;
};
using Curve = struct7_T;
M guide(const Forecast &m, const U &prior, const Curve &curve, const U &trimOffset, int avoid, int N,
        const Config &c);
U curveTrim(double curvature, const Config &c);
inline U direction(double a) {
    return {std::cos(a), std::sin(a)};
}
inline U rotate(const U &v, double a) {
    return {std::cos(a) * v.x() - std::sin(a) * v.y(), std::sin(a) * v.x() + std::cos(a) * v.y()};
}
inline X hold(const X &x, const U &u, const V<4> &scales, M &dense) {
    const double *args[pcbf_hold_SZ_ARG] = {x.data(), u.data(), scales.data()};
    X end;
    double *res[pcbf_hold_SZ_RES] = {end.data(), dense.data()};
    double work[pcbf_hold_SZ_W];
    require(!pcbf_hold(args, res, nullptr, work, 0), "Bicycle integration failed");
    return end;
}
} // namespace pc
