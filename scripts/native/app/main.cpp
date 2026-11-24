#include "planner.hpp"
#include "profile.hpp"
#include <filesystem>
#include <iostream>
namespace pc {
static json sequence(const json &j) {
    return j.is_object() ? json::array({j}) : j;
}
static double number(const json &j, const char *field) {
    double value = j.at(field);
    require(std::isfinite(value), "Nonfinite scalar");
    return value;
}
struct Session {
    const Config &c;
    std::vector<Member> members;
    Curve curve{};
    U trimOffset = U::Zero(), command = U::Zero();
    Plan prior;
    double time = 0;
    bool active = false, seen = false;
    explicit Session(const Config &cfg) : c(cfg) {}
    void init(const json &request, const json &block) {
        const auto &r = request.at("reference");
        Eigen::Map<U>(curve.origin) = vec<2>(r.at("origin"));
        curve.heading = number(r, "heading");
        curve.curvature = number(r, "curvature");
        curve.length = number(r, "length");
        require(curve.length > 0, "Reference length must be positive");
        trimOffset = curveTrim(curve.curvature, c);
        const auto list = sequence(block.at("initialHypotheses"));
        require(list.is_array() && !list.empty() && list.size() <= size_t(c.maxHyp),
                "Invalid initial information set");
        for (auto &v : list) {
            Member m;
            m.ego = vec<6>(v.at("ego"));
            m.scales = vec<4>(v.at("parameters"));
            m.bias = vec<7>(v.at("sensorBias"));
            require(m.scales.minCoeff() > 0, "Physical scales must be positive");
            require(!v.contains("target") || v.at("target").empty(),
                    "Live tracking acquires targets from sensor readings");
            members.push_back(std::move(m));
        }
    }
    bool condition(Member &m, const json &readings, const U &held, bool first) {
        M samples(6, readings.size());
        double tolerance = 1e-9;
        if (!first) {
            const double *in[pcbf_inference_SZ_ARG] = {m.ego.data(), held.data(), m.scales.data()};
            double *out[pcbf_inference_SZ_RES] = {samples.data()};
            double w[pcbf_inference_SZ_W];
            require(!pcbf_inference(in, out, nullptr, w, 0) && samples.allFinite(), "Invalid inference flow");
        }
        for (size_t i = 0; i < readings.size(); i++) {
            auto &y = readings[i];
            if (!first) {
                m.ego = samples.col(i);
                if (i + 1 == readings.size() && held[1] <= 0 && m.ego[3] < .2 * c.holdSpeed) {
                    m.ego.tail<3>().setZero();
                }
            }
            U acc;
            const double *in[pcbf_imu_SZ_ARG] = {m.ego.data(), held.data(), m.scales.data()};
            double *out[pcbf_imu_SZ_RES] = {acc.data()};
            require(!pcbf_imu(in, out, nullptr, nullptr, 0) && acc.allFinite(), "Invalid IMU prediction");
            if ((vec<2>(y.at("position")) - m.ego.head<2>() - m.bias.head<2>()).norm() > c.noise[0] + tolerance ||
                (vec<2>(y.at("acceleration")) - acc - m.bias.segment<2>(2)).norm() > c.noise[1] + tolerance ||
                std::abs(number(y, "yawRate") - m.ego[5] - m.bias[4]) > c.noise[2] + tolerance) {
                return false;
            }
            bool hit = y.at("radarAvailable").get<bool>();
            if (!m.track && !hit) {
                continue;
            }
            if (!m.track) {
                m.track.reset(nrmm_track_create());
                require(bool(m.track), "NRMM allocation failed");
            }
            U rho = U::Zero(), pos = U::Zero();
            double excess = 0, radius = c.noise[3], heading = NAN;
            if (hit) {
                U raw = vec<2>(y.at("relativePosition")) - m.bias.tail<2>();
                rho = raw * (raw.norm() > c.R ? c.R / raw.norm() : 1.);
                excess = (raw - rho).norm();
                if (excess > radius + tolerance) {
                    return false;
                }
                pos = m.ego.head<2>() + rotate(rho, m.ego[2]);
                if (y.contains("relativeHeading") && !y.at("relativeHeading").is_null()) {
                    heading = number(y, "relativeHeading");
                }
            }
            if (y.contains("radarPositionErrorMeters") && !y.at("radarPositionErrorMeters").is_null()) {
                radius = std::min(radius, number(y, "radarPositionErrorMeters"));
            }
            require(nrmm_track_update(m.track.get(), number(y, "time"), hit ? pos.data() : nullptr, m.ego[2],
                                      heading, radius, &m.estimate) == 0,
                    "Invalid NRMM update");
            auto &e = m.estimate;
            if (e.position_bounds[0] > e.position_bounds[2] + tolerance ||
                e.position_bounds[1] > e.position_bounds[3] + tolerance ||
                (e.initialized && !e.motion_available)) {
                return false;
            }
            if (e.motion_available) {
                U center = Eigen::Map<const U>(e.state);
                if (hit) {
                    U filtered = rotate(center - m.ego.head<2>(), -m.ego[2]);
                    if (filtered.norm() > c.R) {
                        filtered *= c.R / filtered.norm();
                    }
                    U delta = filtered - rho;
                    double room = std::max(0., c.noise[3] - excess);
                    if (delta.norm() > room) {
                        delta *= room / delta.norm();
                    }
                    center = pos + rotate(delta, m.ego[2]);
                }
                m.target << center, Eigen::Map<const V<4>>(e.motion), c.targetAxle, c.referenceLength / 2,
                    c.referenceWidth / 2, 0, 0;
                require(m.target.allFinite(), "Nonfinite NRMM motion estimate");
                m.hasTarget = true;
            }
        }
        return true;
    }
    json step(const json &req, bool initial) {
        auto start = Clock::now();
        const auto &b = req.at("block");
        U held = vec<2>(b.at("heldInput"));
        require(std::abs(held[1]) <= 1, "Invalid held longitudinal ratio");
        require(b.value("kind", std::string("jointSensorBlock-v1")) == "jointSensorBlock-v1",
                "Unsupported block kind");
        double t = number(b, "time");
        auto y = sequence(b.at("samples"));
        if (initial) {
            init(req, b);
        } else {
            require(!b.contains("initialHypotheses") && !req.contains("reference"),
                    "Continuation cannot reset the information contract");
            require((held - command).lpNorm<Eigen::Infinity>() < 1e-12,
                    "Held input differs from committed command");
            require(std::abs(t - time - c.h) < 1e-9, "Missing control interval");
        }
        require(y.is_array() && int(y.size()) == (initial ? 1 : std::lround(c.h / c.period)),
                "Incomplete sensor block");
        for (size_t i = 0; i < y.size(); i++) {
            require(std::abs(number(y[i], "time") - (initial ? t : time + (i + 1) * c.period)) < 1e-9,
                    "Unexpected sensor timestamp");
            const bool hit = y[i].at("radarAvailable").get<bool>();
            if (y[i].contains("relativeHeading") && !y[i].at("relativeHeading").is_null()) {
                require(hit && std::isfinite(number(y[i], "relativeHeading")),
                        "A target heading requires a detection");
            }
            if (y[i].contains("radarPositionErrorMeters") && !y[i].at("radarPositionErrorMeters").is_null()) {
                require(number(y[i], "radarPositionErrorMeters") >= 0, "Negative position error radius");
            }
            seen |= hit;
        }
        members.erase(std::remove_if(members.begin(), members.end(),
                                     [&](Member &m) { return !condition(m, y, held, initial); }),
                      members.end());
        require(!members.empty(), "No ego and NRMM hypothesis explains the sensor block");
        bool ready = true, inRange = false;
        for (auto &m : members) {
            ready &= m.hasTarget;
            if (!m.hasTarget) {
                continue;
            }
            auto bounds = Eigen::Map<const Eigen::Matrix2d>(m.estimate.position_bounds);
            U nearest = m.ego.head<2>().cwiseMax(bounds.col(0)).cwiseMin(bounds.col(1));
            inRange |= (nearest - m.ego.head<2>()).norm() <= c.R;
        }
        bool nextActive = ready && seen && (inRange || y.back().at("radarAvailable").get<bool>());
        Decision d = plan(members, nextActive, active, prior, held, curve, trimOffset, c, start);
        prior = std::move(d.plan);
        command = prior.inputs.col(0);
        active = nextActive;
        time = t;
        json estimate = nullptr;
        if (members.front().hasTarget) {
            const auto &m = members.front();
            estimate = {{"state", std::vector<double>(m.target.data(), m.target.data() + 11)},
                        {"motionInitialized", bool(m.estimate.initialized)},
                        {"restCompatible", bool(m.estimate.rest_compatible)}};
            if (m.estimate.bounds_available) {
                estimate["accelerationIntervalMetersPerSecondSquared"] = {m.estimate.motion_bounds[0],
                                                                          m.estimate.motion_bounds[1]};
                estimate["sideslipIntervalRadians"] = {m.estimate.motion_bounds[2], m.estimate.motion_bounds[3]};
            }
        }
        return {{"type", "command"},
                {"targetEstimate", estimate},
                {"nlpSeconds", d.solveSeconds},
                {"time", t},
                {"actuatorInput", {command[0], command[1]}},
                {"steeringRadians", command[0]},
                {"longitudinalRatio", command[1]},
                {"targetActive", active},
                {"source", d.source},
                {"ipoptAttempts", d.attempts},
                {"horizonSteps", d.window},
                {"pcbfDeficit", prior.slacks.sum()},
                {"posteriorCount", members.size()},
                {"elapsedMs", elapsed(start) * 1000}};
    }
};
} // namespace pc
int main(int argc, char **argv) {
    using namespace pc;
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "collision-avoidance: submit one JSON init or step per line on stdin. Commands are returned "
                     "on stdout.\n";
        return 0;
    }
    try {
        require(argc == 1, "Unexpected argument");
        auto lib = std::filesystem::canonical("/proc/self/exe").parent_path().parent_path() / "lib";
        setenv("CASADIPATH", lib.c_str(), 1);
        setenv("OPENBLAS_NUM_THREADS", "1", 1);
        setenv("OMP_NUM_THREADS", "1", 1);
        Config cfg(json::parse(controllerProfile));
        std::unique_ptr<Session> session;
        std::string line;
        bool fault = false;
        while (std::getline(std::cin, line)) {
            try {
                auto r = json::parse(line);
                auto type = r.at("type").get<std::string>();
                if (type == "profile") {
                    std::cout
                        << json{{"type", "profile"}, {"cfg", json::parse(controllerProfile).at("cfg")}}.dump()
                        << std::endl;
                    continue;
                }
                if (type == "warmup") {
                    std::cout << json{{"type", "warmup"}, {"ipoptAttempts", warmup(cfg)}}.dump() << std::endl;
                    continue;
                }
                require(type == "init" || type == "step", "Unknown request type");
                if (type == "init") {
                    session = std::make_unique<Session>(cfg);
                }
                require(bool(session), "Initialize the controller first");
                std::cout << session->step(r, type == "init").dump() << std::endl;
                fault = false;
            } catch (const std::exception &e) {
                session.reset();
                fault = true;
                std::cout << json{{"type", "error"}, {"message", e.what()}}.dump() << std::endl;
            }
        }
        return fault ? 1 : 0;
    } catch (const std::exception &e) {
        std::cout << json{{"type", "error"}, {"message", e.what()}}.dump() << std::endl;
        return 1;
    }
}
